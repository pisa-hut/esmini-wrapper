import ctypes as ct
import logging
from dataclasses import replace
from pathlib import Path
from typing import Any

from pisa_api.simulator import (
    CollisionInfoData,
    ControlCommand,
    ControlMode,
    InitRequest,
    InvalidSimulatorRequest,
    ObjectKinematicData,
    ObjectStateData,
    ResetRequest,
    ResetResponse,
    RoadObjectType,
    RuntimeFrameData,
    ShapeData,
    ShapeDimensionData,
    ShapeType,
    ShouldQuitResponse,
    SimulatorPreconditionFailed,
    SimulatorUnavailable,
    StepRequest,
    StepResponse,
)

logger = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler()],
)


SEId = ct.c_uint32


class SEScenarioObjectState(ct.Structure):
    _fields_ = [
        ("id", ct.c_int),
        ("model_id", ct.c_int),
        ("ctrl_type", ct.c_int),
        ("timestamp", ct.c_double),
        ("x", ct.c_double),
        ("y", ct.c_double),
        ("z", ct.c_double),
        ("h", ct.c_double),
        ("p", ct.c_double),
        ("r", ct.c_double),
        ("roadId", SEId),
        ("junctionId", SEId),
        ("t", ct.c_double),
        ("laneId", ct.c_int),
        ("laneOffset", ct.c_double),
        ("s", ct.c_double),
        ("speed", ct.c_double),
        ("centerOffsetX", ct.c_double),
        ("centerOffsetY", ct.c_double),
        ("centerOffsetZ", ct.c_double),
        ("width", ct.c_double),
        ("length", ct.c_double),
        ("height", ct.c_double),
        ("objectType", ct.c_int),
        ("objectCategory", ct.c_int),
        ("wheel_angle", ct.c_double),
        ("wheel_rot", ct.c_double),
        ("visibilityMask", ct.c_int),
    ]


class SESimpleVehicleState(ct.Structure):
    _fields_ = [
        ("x", ct.c_double),
        ("y", ct.c_double),
        ("z", ct.c_double),
        ("h", ct.c_double),
        ("p", ct.c_double),
        ("speed", ct.c_double),
        ("wheel_rotation", ct.c_double),
        ("wheel_angle", ct.c_double),
    ]


class Vehicle:
    """Internal helper class, only used inside Simulator."""

    def __init__(self, se, x, y, h, length, speed, cfg: dict[str, Any] | None = None):
        self._se = se
        self._cfg = cfg or {}
        self.sv_handle = self._se.SE_SimpleVehicleCreate(x, y, h, length, speed)
        self.vh_state = SESimpleVehicleState()
        self._se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.vh_state))

    def apply_control(self, ctrl: ControlCommand, dt_s: float):
        if ctrl.mode == ControlMode.NONE:
            return
        elif ctrl.mode == ControlMode.THROTTLE_STEER:
            pedal = ctrl.payload["pedal"]
            wheel = ctrl.payload["wheel"]
            self._se.SE_SimpleVehicleControlBinary(self.sv_handle, dt_s, pedal, wheel)
            # Update vehicle state
            self._se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.vh_state))

        elif ctrl.mode == ControlMode.THROTTLE_STEER_BREAK:
            # throttle = float(ctrl.payload.get("throttle", 0.0))
            # steer = float(ctrl.payload.get("steer", 0.0))
            # brake = float(ctrl.payload.get("brake", 0.0))

            throttle = ctrl.payload["throttle"]
            steer = ctrl.payload["steer"]
            brake = ctrl.payload["brake"]
            final_throttle = throttle - brake  # Simple way to combine throttle and brake

            self._se.SE_SimpleVehicleControlAnalog(self.sv_handle, dt_s, final_throttle, steer)
            # Update vehicle state
            self._se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.vh_state))

        elif ctrl.mode == ControlMode.ACKERMANN:
            steer = float(ctrl.payload.get("steer", 0.0))
            acceleration = self._ackermann_acceleration(ctrl.payload)

            self._se.SE_SimpleVehicleControlAccAndSteer(self.sv_handle, dt_s, acceleration, steer)
            # Update vehicle state
            self._se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.vh_state))

        elif ctrl.mode == ControlMode.POSITION:
            x = ctrl.payload.get("x", self.vh_state.x)
            y = ctrl.payload.get("y", self.vh_state.y)
            h = ctrl.payload.get("h", self.vh_state.h)
            speed = ctrl.payload.get("speed", self.vh_state.speed)
            # Directly set position
            self.vh_state.x = x
            self.vh_state.y = y
            self.vh_state.h = h
            self.vh_state.speed = speed

        else:
            logger.warning(f"Unsupported control mode: {ctrl.mode}")

    def _ackermann_acceleration(self, payload: dict[str, Any]) -> float:
        if "speed" in payload:
            target_speed = float(payload["speed"])
            speed_error = target_speed - float(self.vh_state.speed)
            kp = float(self._cfg.get("ackermann_speed_kp", 20.0))
            acceleration = speed_error * kp
        else:
            acceleration = float(payload.get("acceleration", 0.0))

        accel_limit = float(
            self._cfg.get(
                "ackermann_accel_limit",
                self._cfg.get("ackermann_accel_default", 20.0),
            )
        )
        decel_limit = float(self._cfg.get("ackermann_decel_limit", accel_limit))
        return max(-decel_limit, min(accel_limit, acceleration))


TYPE_MAP = {
    0: RoadObjectType.CAR,
    1: RoadObjectType.VAN,
    2: RoadObjectType.TRUCK,
    3: RoadObjectType.SEMITRAILER,
    4: RoadObjectType.TRAILER,
    5: RoadObjectType.BUS,
    6: RoadObjectType.MOTORCYCLE,
    7: RoadObjectType.BICYCLE,
    8: RoadObjectType.TRAIN,
    9: RoadObjectType.TRAM,
}


class EsminiAdapter:
    def __init__(self):
        self._time_ns = 0

        self.ego_car = None
        self.objects: list[ObjectStateData] = []

        # init
        self.cfg = None
        self.scenario = None
        self._output_base = None

        self.esmini_home = None
        self.se = None

        # reset
        self._output_dir = None
        self.obj_count = 0
        self._object_ids: list[int] = []

        self._params_obj = None
        self._params_ptr = None
        self._c_param_cb = None

    def _setup_esmini_opts(self):
        self.se.SE_SetSeed(1234)

        use_viewer = int(self.cfg.get("use_viewer", 1))
        threads = int(self.cfg.get("threads", 0))
        record = int(self.cfg.get("record", 0))

        if "log_file_path" in self.cfg:
            log_file_path = Path(self._output_dir) / self.cfg["log_file_path"]
            logger.info(
                f'Setting esmini log file path to: {log_file_path} (from cfg "log_file_path")'
            )
            self.se.SE_SetLogFilePath(str(log_file_path).encode())
        else:
            logger.info("No log_file_path specified; using default esmini_log.txt")
            self.se.SE_SetLogFilePath(b"./esmini_log.txt")

        if "path" in self.cfg:
            for extra_path in self.cfg["path"]:
                logger.info(f"Adding esmini path: {extra_path}")
                self.se.SE_AddPath(extra_path.encode())

        if "window" in self.cfg:
            win_cfg = self.cfg["window"]  # ["x", "y", "width", "height"]
            self.se.SE_SetWindowPosAndSize(
                int(win_cfg[0]), int(win_cfg[1]), int(win_cfg[2]), int(win_cfg[3])
            )

        if self.cfg.get("disable_stdout", True):
            logger.info("Disable stdout in esmini")
            self.se.SE_SetOptionPersistent(b"disable_stdout")

        if self.cfg.get("dat_file_path", None) is not None:
            dat_file_path = Path(self._output_dir) / self.cfg["dat_file_path"]
            logger.info(f"Setting esmini dat file path: {dat_file_path}")
            self.se.SE_SetDatFilePath(str(dat_file_path).encode())

        return use_viewer, threads, record

    def _setup_function_signatures(self):
        se = self.se

        # SE_DLL_API int SE_AddPath(const char *path);
        se.SE_AddPath.argtypes = [ct.c_char_p]
        se.SE_AddPath.restype = ct.c_int

        # SE_DLL_API void SE_SetLogFilePath(const char *logFilePath);
        se.SE_SetLogFilePath.argtypes = [ct.c_char_p]
        se.SE_SetLogFilePath.restype = None

        # SE_DLL_API int SE_GetObjectState(int object_id, SE_ScenarioObjectState *state);
        se.SE_GetObjectState.argtypes = [ct.c_int, ct.POINTER(SEScenarioObjectState)]
        se.SE_GetObjectState.restype = ct.c_int

        # SE_DLL_API double SE_GetObjectAcceleration(int object_id);
        se.SE_GetObjectAcceleration.argtypes = [ct.c_int]
        se.SE_GetObjectAcceleration.restype = ct.c_double

        # SE_DLL_API int SE_GetObjectAngularAcceleration(int object_id, double *h_acc, double *p_acc, double *r_acc);
        se.SE_GetObjectAngularAcceleration.argtypes = [
            ct.c_int,
            ct.POINTER(ct.c_double),
            ct.POINTER(ct.c_double),
            ct.POINTER(ct.c_double),
        ]
        se.SE_GetObjectAngularAcceleration.restype = ct.c_int

        # SE_DLL_API int SE_GetObjectAngularVelocity(int object_id, double *h_rate, double *p_rate, double *r_rate);
        se.SE_GetObjectAngularVelocity.argtypes = [
            ct.c_int,
            ct.POINTER(ct.c_double),
            ct.POINTER(ct.c_double),
            ct.POINTER(ct.c_double),
        ]
        se.SE_GetObjectAngularVelocity.restype = ct.c_int

        # SE_DLL_API const char *SE_GetObjectTypeName(int object_id)
        # se.SE_GetObjectTypeName.argtypes = [ct.c_int]
        # se.SE_GetObjectTypeName.restype = ct.c_char_p

        # SE_DLL_API void *SE_SimpleVehicleCreate(double x, double y, double h, double length, double speed);
        se.SE_SimpleVehicleCreate.argtypes = [
            ct.c_double,
            ct.c_double,
            ct.c_double,
            ct.c_double,
            ct.c_double,
        ]
        se.SE_SimpleVehicleCreate.restype = ct.c_void_p

        # SE_DLL_API void SE_SimpleVehicleDelete(void *handleSimpleVehicle);
        se.SE_SimpleVehicleDelete.argtypes = [ct.c_void_p]
        se.SE_SimpleVehicleDelete.restype = None

        # SE_DLL_API void SE_SimpleVehicleGetState(void *handleSimpleVehicle, SE_SimpleVehicleState *state);
        se.SE_SimpleVehicleGetState.argtypes = [
            ct.c_void_p,
            ct.POINTER(SESimpleVehicleState),
        ]
        se.SE_SimpleVehicleGetState.restype = None

        # SE_DLL_API void SE_SimpleVehicleControlBinary(void *handleSimpleVehicle, double dt, int throttle, int steering);
        se.SE_SimpleVehicleControlBinary.argtypes = [
            ct.c_void_p,
            ct.c_double,
            ct.c_int,
            ct.c_int,
        ]
        se.SE_SimpleVehicleControlBinary.restype = None

        # SE_DLL_API void SE_SimpleVehicleControlAnalog(void  *handleSimpleVehicle, double dt, double throttle, double steering);
        se.SE_SimpleVehicleControlAnalog.argtypes = [
            ct.c_void_p,
            ct.c_double,
            ct.c_double,
            ct.c_double,
        ]
        se.SE_SimpleVehicleControlAnalog.restype = None

        # SE_DLL_API void SE_SimpleVehicleControlAccAndSteer(void *handleSimpleVehicle, double dt, double acceleration, double steering_angle);
        se.SE_SimpleVehicleControlAccAndSteer.argtypes = [
            ct.c_void_p,
            ct.c_double,
            ct.c_double,
            ct.c_double,
        ]
        se.SE_SimpleVehicleControlAccAndSteer.restype = None

        # SE_DLL_API void SE_SimpleVehicleControlTarget(void *handleSimpleVehicle, double dt, double target_speed, double heading_to_target);
        se.SE_SimpleVehicleControlTarget.argtypes = [
            ct.c_void_p,
            ct.c_double,
            ct.c_double,
            ct.c_double,
        ]
        se.SE_SimpleVehicleControlTarget.restype = None

        se.SE_SimpleVehicleSetSpeed.argtypes = [ct.c_void_p, ct.c_double]
        se.SE_SimpleVehicleSetSpeed.restype = None

        # SE_DLL_API int SE_ReportObjectWheelStatus(int object_id, double rotation, double angle);
        se.SE_ReportObjectWheelStatus.argtypes = [
            ct.c_int,
            ct.c_double,
            ct.c_double,
        ]
        se.SE_ReportObjectWheelStatus.restype = ct.c_int

        # SE_DLL_API int SE_ReportObjectSpeed(int object_id, double speed);
        se.SE_ReportObjectSpeed.argtypes = [ct.c_int, ct.c_double]
        se.SE_ReportObjectSpeed.restype = ct.c_int

        # SE_DLL_API int SE_ReportObjectPosXYH(int object_id, double x, double y, double h);
        se.SE_ReportObjectPosXYH.argtypes = [
            ct.c_int,
            ct.c_double,
            ct.c_double,
            ct.c_double,
        ]
        se.SE_ReportObjectPosXYH.restype = ct.c_int

        # SE_DLL_API void SE_RegisterParameterDeclarationCallback(void (*fnPtr)(void *), void *user_data);
        self._PARAM_CB_TYPE = ct.CFUNCTYPE(None, ct.c_void_p)
        self.se.SE_RegisterParameterDeclarationCallback.argtypes = [
            self._PARAM_CB_TYPE,
            ct.c_void_p,
        ]
        self.se.SE_RegisterParameterDeclarationCallback.restype = None

        # SE_DLL_API void SE_ClearPaths();
        self.se.SE_ClearPaths.argtypes = []
        self.se.SE_ClearPaths.restype = None

        # SE_DLL_API void SE_SetWindowPosAndSize(int x, int y, int w, int h);
        self.se.SE_SetWindowPosAndSize.argtypes = [
            ct.c_int,
            ct.c_int,
            ct.c_int,
            ct.c_int,
        ]
        self.se.SE_SetWindowPosAndSize.restype = None

        # SE_DLL_API const char *SE_GetVariableName(int index, int *type);
        self.se.SE_GetVariableName.argtypes = [ct.c_int, ct.POINTER(ct.c_int)]
        self.se.SE_GetVariableName.restype = ct.c_char_p

        # SE_DLL_API void SE_SetSeed(unsigned int seed);
        self.se.SE_SetSeed.argtypes = [ct.c_uint]
        self.se.SE_SetSeed.restype = None

        # SE_DLL_API int SE_SetParameterBool(const char *parameterName, bool value);
        self.se.SE_SetParameterBool.argtypes = [ct.c_char_p, ct.c_bool]
        self.se.SE_SetParameterBool.restype = ct.c_int

        # SE_DLL_API int SE_SetParameterInt(const char *parameterName, int value);
        self.se.SE_SetParameterInt.argtypes = [ct.c_char_p, ct.c_int]
        self.se.SE_SetParameterInt.restype = ct.c_int

        # SE_DLL_API int SE_SetParameterDouble(const char *parameterName, double value);
        self.se.SE_SetParameterDouble.argtypes = [ct.c_char_p, ct.c_double]
        self.se.SE_SetParameterDouble.restype = ct.c_int

        # SE_DLL_API int SE_SetParameterString(const char *parameterName, const char *value);
        self.se.SE_SetParameterString.argtypes = [ct.c_char_p, ct.c_char_p]
        self.se.SE_SetParameterString.restype = ct.c_int

        # SE_DLL_API const char *SE_GetParameterName(int index, int *type);
        se.SE_GetParameterName.argtypes = [ct.c_int, ct.POINTER(ct.c_int)]
        se.SE_GetParameterName.restype = ct.c_char_p

        # SE_DLL_API int SE_GetNumberOfParameters();
        se.SE_GetNumberOfParameters.argtypes = []
        se.SE_GetNumberOfParameters.restype = ct.c_int

        # SE_DLL_API int SE_GetNumberOfObjects()
        se.SE_GetNumberOfObjects.argtypes = []
        se.SE_GetNumberOfObjects.restype = ct.c_int

        # SE_DLL_API int SE_GetId(int index);
        se.SE_GetId.argtypes = [ct.c_int]
        se.SE_GetId.restype = ct.c_int

        # SE_DLL_API void SE_CollisionDetection(bool mode);
        se.SE_CollisionDetection.argtypes = [ct.c_bool]
        se.SE_CollisionDetection.restype = None

        # SE_DLL_API int SE_GetObjectNumberOfCollisions(int object_id);
        se.SE_GetObjectNumberOfCollisions.argtypes = [ct.c_int]
        se.SE_GetObjectNumberOfCollisions.restype = ct.c_int

        # SE_DLL_API int SE_GetObjectCollision(int object_id, int index);
        se.SE_GetObjectCollision.argtypes = [ct.c_int, ct.c_int]
        se.SE_GetObjectCollision.restype = ct.c_int

        # SE_DLL_API double SE_GetSimTimeStep();
        se.SE_GetSimTimeStep.argtypes = []
        se.SE_GetSimTimeStep.restype = ct.c_double

        # SE_DLL_API double SE_GetSimulationTime();
        se.SE_GetSimulationTime.argtypes = []
        se.SE_GetSimulationTime.restype = ct.c_double

        # SE_DLL_API int SE_Init(const char *oscFilename, int disable_ctrls, int use_viewer, int threads, int record);
        se.SE_Init.argtypes = [
            ct.c_char_p,
            ct.c_int,
            ct.c_int,
            ct.c_int,
            ct.c_int,
        ]
        se.SE_Init.restype = ct.c_int

        # SE_DLL_API int SE_StepDT(double dt);
        se.SE_StepDT.argtypes = [ct.c_double]
        se.SE_StepDT.restype = ct.c_int

        se.SE_GetQuitFlag.argtypes = []
        se.SE_GetQuitFlag.restype = ct.c_int

        se.SE_SetOptionPersistent.argtypes = [ct.c_char_p]
        se.SE_SetOptionPersistent.restype = ct.c_int

        # SE_DLL_API int SE_UnsetOption(const char *name);
        se.SE_UnsetOption.argtypes = [ct.c_char_p]
        se.SE_UnsetOption.restype = ct.c_int

        # SE_DLL_API void SE_SetDatFilePath(const char *datFilePath);
        se.SE_SetDatFilePath.argtypes = [ct.c_char_p]
        se.SE_SetDatFilePath.restype = None

        # SE_DLL_API void SE_Close();
        se.SE_Close.argtypes = []
        se.SE_Close.restype = None

    def init(self, request: InitRequest) -> None:
        self.cfg = request.config
        self._output_base = request.output_dir
        self.scenario = request.scenario

        self.esmini_home = self.cfg.get("esmini_home", "/opt/esmini/")
        lib_path = Path(self.esmini_home) / "bin" / "libesminiLib.so"
        if not lib_path.is_file():
            raise SimulatorUnavailable(f"esmini shared library not found at: {lib_path}")
        try:
            self.se = ct.CDLL(str(lib_path))  # Linux
        except OSError as exc:
            raise SimulatorUnavailable(f"failed to load esmini shared library: {exc}") from exc
        self._setup_function_signatures()

    def _collision_detection_enabled(self) -> bool:
        return bool(
            self.cfg.get(
                "collision_detection",
                self.cfg.get("enable_collision_detection", True),
            )
        )

    def _set_collision_detection(self, enabled: bool) -> None:
        self.se.SE_CollisionDetection(enabled)

    def _collect_collision_info(self) -> list[CollisionInfoData]:
        collisions: list[CollisionInfoData] = []
        seen_pairs: set[tuple[int, int]] = set()

        for object_id in self._object_ids:
            collision_count = self.se.SE_GetObjectNumberOfCollisions(object_id)
            if collision_count <= 0:
                continue

            for collision_index in range(collision_count):
                other_object_id = self.se.SE_GetObjectCollision(object_id, collision_index)
                if other_object_id < 0 or other_object_id == object_id:
                    continue

                actor_a, actor_b = sorted((object_id, other_object_id))
                pair = (actor_a, actor_b)
                if pair in seen_pairs:
                    continue
                seen_pairs.add(pair)

                details = {
                    "source": "esmini",
                    "object_id_a": actor_a,
                    "object_id_b": actor_b,
                }
                collisions.append(
                    CollisionInfoData(
                        occurred=True,
                        actor_a=actor_a,
                        actor_b=actor_b,
                        details=details,
                    )
                )

        return collisions

    def reset(self, request: ResetRequest):
        self._output_dir = self._output_base / request.output_dir
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self.stop()

        # Reset time
        self._time_ns = 0

        params = request.params

        # Keep params alive as a Python object and pass it to esmini as void* user data.
        self._params_obj = ct.py_object(params)
        self._params_ptr = ct.cast(ct.pointer(self._params_obj), ct.c_void_p)

        # Create the C callback once and keep a reference to prevent GC.
        if self._c_param_cb is None:

            @self._PARAM_CB_TYPE
            def _c_param_cb(user_data):

                py_obj_ptr = ct.cast(user_data, ct.POINTER(ct.py_object))
                params_dict = py_obj_ptr.contents.value

                # Delegate to the Python parameter-setting logic.
                self.parameter_declaration_callback(params_dict)

            self._c_param_cb = _c_param_cb  # hold reference

        # Register before SE_Init so parameters can affect initial scenario state.
        self.se.SE_RegisterParameterDeclarationCallback(
            self._c_param_cb,
            self._params_ptr,
        )
        try:
            use_viewer, threads, record = self._setup_esmini_opts()
            disable_controller = int(
                self.cfg.get("disable_controller", self.cfg.get("disable_controllers", 1))
            )
        except (TypeError, ValueError) as exc:
            raise InvalidSimulatorRequest(f"invalid esmini config: {exc}") from exc

        map_name = request.scenario_pack.map_name
        if not map_name:
            raise InvalidSimulatorRequest("scenario_pack.map_name is required")
        map_path = Path(f"/mnt/map/xodr/{map_name}.xodr").resolve()
        if not map_path.is_file():
            raise InvalidSimulatorRequest(f"map not found: {map_path}")
        self.se.SE_AddPath(str(map_path.parent).encode())
        xosc_name = request.scenario_pack.name
        if not xosc_name:
            raise InvalidSimulatorRequest("scenario_pack.name is required")
        xosc_path = Path(f"/mnt/scenario/{xosc_name}.xosc").resolve()
        if not xosc_path.is_file():
            raise InvalidSimulatorRequest(f"OpenSCENARIO file not found: {xosc_path}")
        ret = self.se.SE_Init(
            str(xosc_path).encode(),
            disable_controller,
            use_viewer,
            threads,
            record,
        )
        if ret != 0:
            raise InvalidSimulatorRequest(f"esmini SE_Init failed with code {ret}")

        self._set_collision_detection(self._collision_detection_enabled())

        self.obj_count = self.se.SE_GetNumberOfObjects()
        self._object_ids = [self.se.SE_GetId(i) for i in range(self.obj_count)]
        if self.obj_count <= 0 or not self._object_ids:
            raise SimulatorPreconditionFailed("esmini scenario contains no objects")
        invalid_object_ids = [object_id for object_id in self._object_ids if object_id < 0]
        if invalid_object_ids:
            raise SimulatorPreconditionFailed(
                f"esmini returned invalid object ids: {invalid_object_ids}"
            )
        self.objects = []
        for object_id in self._object_ids:
            obj_state = SEScenarioObjectState()
            ret_state = self.se.SE_GetObjectState(object_id, ct.byref(obj_state))
            if ret_state != 0:
                raise SimulatorPreconditionFailed(
                    f"SE_GetObjectState failed for object id {object_id} (ret={ret_state})"
                )

            esmini_obj_type = int(obj_state.objectType)
            obj_category = int(obj_state.objectCategory)
            obj_type = RoadObjectType.UNKNOWN
            if esmini_obj_type == 2:  # Pedestrian type
                if obj_category == 0:  # Pedestrian
                    obj_type = RoadObjectType.PEDESTRIAN
                elif obj_category == 1:  # Wheelchair
                    obj_type = RoadObjectType.WHEELCHAIR
                elif obj_category == 2:  # Animal
                    obj_type = RoadObjectType.ANIMAL
                else:
                    obj_type = RoadObjectType.UNKNOWN
            elif esmini_obj_type == 1:  # Vehicle type
                obj_type = TYPE_MAP.get(obj_category, RoadObjectType.UNKNOWN)

            obj_kinematic = ObjectKinematicData(
                time_ns=self._time_ns,
                x=float(obj_state.x),
                y=float(obj_state.y),
                z=float(obj_state.z),
                yaw=float(obj_state.h),
                speed=float(obj_state.speed),
            )

            obj_shape = ShapeData(
                type=ShapeType.BOUNDING_BOX,
                dimensions=ShapeDimensionData(
                    x=float(obj_state.length),
                    y=float(obj_state.width),
                    z=float(obj_state.height),
                ),
            )

            obj = ObjectStateData(
                type=obj_type,
                kinematic=obj_kinematic,
                shape=obj_shape,
            )
            self.objects.append(obj)

        # Create ego vehicle helper
        self.ego_car = Vehicle(
            self.se,
            x=float(self.objects[0].kinematic.x),
            y=float(self.objects[0].kinematic.y),
            h=float(self.objects[0].kinematic.yaw),
            length=float(self.objects[0].shape.dimensions.x),
            speed=float(self.objects[0].kinematic.speed),
            cfg=self.cfg,
        )
        if not self.ego_car.sv_handle:
            raise SimulatorPreconditionFailed("failed to spawn ego vehicle")

        collisions = self._collect_collision_info()
        runtime_frame = RuntimeFrameData(
            sim_time_ns=self._time_ns,
            objects=self.objects,
            collision=collisions,
            extras={"object_ids": self._object_ids},
        )

        return ResetResponse(frame=runtime_frame)

    def step(self, request: StepRequest):
        # ctrl = Ctrl.from_pb(ctrl)
        next_time_ns = request.timestamp_ns
        if next_time_ns < self._time_ns:
            raise InvalidSimulatorRequest(
                f"step timestamp must be monotonic: got {next_time_ns}, current {self._time_ns}"
            )
        dt_s = (next_time_ns - self._time_ns) / 1e9

        se = self.se

        # Update vehicle control
        try:
            self.ego_car.apply_control(request.ctrl_cmd, dt_s)
        except (KeyError, TypeError, ValueError) as exc:
            raise InvalidSimulatorRequest(f"invalid control command: {exc}") from exc
        obj_id = self._object_ids[0]
        ret_pos = se.SE_ReportObjectPosXYH(
            obj_id,
            self.ego_car.vh_state.x,
            self.ego_car.vh_state.y,
            self.ego_car.vh_state.h,
        )
        if ret_pos != 0:
            raise SimulatorPreconditionFailed(
                f"SE_ReportObjectPosXYH failed for object id {obj_id} (ret={ret_pos})"
            )
        ret_wheel = se.SE_ReportObjectWheelStatus(
            obj_id,
            self.ego_car.vh_state.wheel_rotation,
            self.ego_car.vh_state.wheel_angle,
        )
        if ret_wheel != 0:
            raise SimulatorPreconditionFailed(
                f"SE_ReportObjectWheelStatus failed for object id {obj_id} (ret={ret_wheel})"
            )
        ret_speed = se.SE_ReportObjectSpeed(
            obj_id,
            self.ego_car.vh_state.speed,
        )
        if ret_speed != 0:
            raise SimulatorPreconditionFailed(
                f"SE_ReportObjectSpeed failed for object id {obj_id} (ret={ret_speed})"
            )

        ret_step = se.SE_StepDT(dt_s)
        if ret_step != 0:
            raise SimulatorUnavailable(f"esmini SE_StepDT failed with code {ret_step}")

        self._time_ns = next_time_ns
        # Update object state
        for i, object_id in enumerate(self._object_ids):
            # Get object state
            obj_state = SEScenarioObjectState()
            ret_state = se.SE_GetObjectState(object_id, ct.byref(obj_state))

            # Get object acceleration
            obj_accel = se.SE_GetObjectAcceleration(object_id)

            # Get object angular velocity
            h_rate = ct.c_double()
            p_rate = ct.c_double()
            r_rate = ct.c_double()
            ret_rate = se.SE_GetObjectAngularVelocity(
                object_id, ct.byref(h_rate), ct.byref(p_rate), ct.byref(r_rate)
            )

            # Get object angular acceleration
            h_acc = ct.c_double()
            p_acc = ct.c_double()
            r_acc = ct.c_double()
            ret_acc = se.SE_GetObjectAngularAcceleration(
                object_id, ct.byref(h_acc), ct.byref(p_acc), ct.byref(r_acc)
            )

            if ret_state != 0:
                logger.warning(
                    f"SE_GetObjectState failed for object id {object_id} (ret={ret_state})"
                )
                continue

            kinematic = ObjectKinematicData(
                time_ns=self._time_ns,
                x=float(obj_state.x),
                y=float(obj_state.y),
                z=float(obj_state.z),
                yaw=float(obj_state.h),
                speed=float(obj_state.speed),
                acceleration=float(obj_accel),
                yaw_rate=float(h_rate.value) if ret_rate == 0 else 0.0,
                yaw_acceleration=float(h_acc.value) if ret_acc == 0 else 0.0,
            )
            self.objects[i] = replace(self.objects[i], kinematic=kinematic)

        collisions = self._collect_collision_info()
        runtime_frame = RuntimeFrameData(
            sim_time_ns=self._time_ns,
            objects=self.objects,
            collision=collisions,
            extras={"object_ids": self._object_ids},
        )

        return StepResponse(frame=runtime_frame)

    def stop(self):
        if self.se is None:
            return

        if self.ego_car is not None:
            self.se.SE_SimpleVehicleDelete(self.ego_car.sv_handle)
            self.ego_car = None

        self._set_collision_detection(False)
        self.se.SE_UnsetOption(b"disable_stdout")
        self.se.SE_Close()
        self.se.SE_ClearPaths()
        self.objects = []
        self._object_ids = []
        self.obj_count = 0
        self._time_ns = 0
        logger.info("Esmini simulator stopped.")

    def parameter_declaration_callback(self, params: dict[str, Any]) -> int:
        """
        Apply parameter overrides from a Python dict.

        esmini only calls the C-compatible wrapper callback; this method keeps the
        Python-side parameter conversion and validation in one place.
        """
        n = self.se.SE_GetNumberOfParameters()
        param_type = {}
        for i in range(n):
            ptype = ct.c_int()
            param_name = self.se.SE_GetParameterName(i, ct.byref(ptype)).decode("utf-8")
            param_type[param_name] = ptype.value

        for name, value in params.items():
            if name not in param_type:
                logger.warning(f"Parameter {name} not found in esmini parameters. Skip.")
                continue

            ptype = param_type[name]
            if ptype == 1:  # int
                try:
                    v = int(value)
                except (TypeError, ValueError):
                    logger.warning(f"Parameter {name} value {value} is not an int. Skip.")
                    continue

                self.se.SE_SetParameterInt(name.encode("utf-8"), v)
                logger.debug(f"  set {name} = {v}")
            elif ptype == 2:  # double
                try:
                    v = float(value)
                except (TypeError, ValueError):
                    logger.warning(f"Parameter {name} value {value} is not a float. Skip.")
                    continue

                self.se.SE_SetParameterDouble(name.encode("utf-8"), v)
                logger.debug(f"  set {name} = {v}")
            elif ptype == 3:  # string
                v = str(value)
                self.se.SE_SetParameterString(name.encode("utf-8"), v.encode("utf-8"))
                logger.debug(f"  set {name} = {v}")
            elif ptype == 4:  # bool
                try:
                    v = bool(value)
                except (TypeError, ValueError):
                    logger.warning(f"Parameter {name} value {value} is not a bool. Skip.")
                    continue

                self.se.SE_SetParameterBool(name.encode("utf-8"), v)
                logger.debug(f"  set {name} = {v}")
            else:
                logger.warning(f"Parameter {name} has unknown type {ptype}. Skip.")
                continue

        # The C callback is void; this return value is only useful to Python callers.
        return 0

    # define a function returning if the simulator need to stop
    def should_quit(self) -> ShouldQuitResponse:
        if self.se is None:
            return ShouldQuitResponse(
                should_quit=False,
                msg="esmini simulator is not initialized",
            )

        should_quit = bool(self.se.SE_GetQuitFlag())
        msg = (
            "esmini simulator requested shutdown" if should_quit else "esmini simulator is running"
        )
        return ShouldQuitResponse(should_quit=should_quit, msg=msg)
