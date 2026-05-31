"""Smoke tests for import-level wrapper integration."""

from types import SimpleNamespace

import pytest
from pisa_api.simulator import (
    ControlCommand,
    ControlMode,
    RoadObjectType,
    SimulatorUnavailable,
    StepRequest,
    StepResponse,
)
from pisa_api.simulator import (
    RuntimeFrameData as PisaRuntimeFrameData,
)


def test_public_imports_use_pisa_api_simulator_contract() -> None:
    from esmini_wrapper.esmini import TYPE_MAP, EsminiAdapter

    assert EsminiAdapter.reset.__annotations__["request"].__name__ == "ResetRequest"
    assert PisaRuntimeFrameData.__name__ == "RuntimeFrameData"
    assert TYPE_MAP[3] == RoadObjectType.SEMITRAILER
    assert EsminiAdapter is not None


class FakeSE:
    def __init__(self, *, step_ret: int = 0):
        self.step_ret = step_ret
        self.control_acc_and_steer_calls = []
        self.report_pos_calls = []
        self.report_wheel_calls = []
        self.report_speed_calls = []
        self.state = SimpleNamespace(
            x=1.0,
            y=2.0,
            z=0.0,
            h=0.3,
            p=0.0,
            speed=4.0,
            wheel_rotation=0.0,
            wheel_angle=0.0,
        )

    def SE_SimpleVehicleCreate(self, x, y, h, length, speed):
        self.state.x = x
        self.state.y = y
        self.state.h = h
        self.state.speed = speed
        return 123

    def SE_SimpleVehicleGetState(self, _handle, state_ptr):
        state = state_ptr._obj
        state.x = self.state.x
        state.y = self.state.y
        state.z = self.state.z
        state.h = self.state.h
        state.p = self.state.p
        state.speed = self.state.speed
        state.wheel_rotation = self.state.wheel_rotation
        state.wheel_angle = self.state.wheel_angle

    def SE_SimpleVehicleControlAccAndSteer(self, handle, dt, acceleration, steering_angle):
        self.control_acc_and_steer_calls.append((handle, dt, acceleration, steering_angle))
        self.state.speed += acceleration * dt
        self.state.wheel_angle = steering_angle

    def SE_ReportObjectPosXYH(self, object_id, x, y, h):
        self.report_pos_calls.append((object_id, x, y, h))
        return 0

    def SE_ReportObjectWheelStatus(self, object_id, rotation, angle):
        self.report_wheel_calls.append((object_id, rotation, angle))
        return 0

    def SE_ReportObjectSpeed(self, object_id, speed):
        self.report_speed_calls.append((object_id, speed))
        return 0

    def SE_StepDT(self, _dt):
        return self.step_ret

    def SE_GetObjectState(self, object_id, state_ptr):
        state = state_ptr._obj
        state.id = object_id
        state.x = self.state.x
        state.y = self.state.y
        state.z = self.state.z
        state.h = self.state.h
        state.speed = self.state.speed
        return 0

    def SE_GetObjectAcceleration(self, _object_id):
        return 0.0

    def SE_GetObjectAngularVelocity(self, _object_id, h_rate, p_rate, r_rate):
        h_rate._obj.value = 0.0
        p_rate._obj.value = 0.0
        r_rate._obj.value = 0.0
        return 0

    def SE_GetObjectAngularAcceleration(self, _object_id, h_acc, p_acc, r_acc):
        h_acc._obj.value = 0.0
        p_acc._obj.value = 0.0
        r_acc._obj.value = 0.0
        return 0

    def SE_GetObjectNumberOfCollisions(self, _object_id):
        return 0


def test_ackermann_control_tracks_speed_target_and_steering_angle() -> None:
    from esmini_wrapper.esmini import Vehicle

    se = FakeSE()
    vehicle = Vehicle(se, x=1.0, y=2.0, h=0.3, length=4.5, speed=4.0)

    vehicle.apply_control(
        ControlCommand(
            mode=ControlMode.ACKERMANN,
            payload={"steer": 0.12, "speed": 8.0, "acceleration": 1.7},
        ),
        dt_s=0.1,
    )

    assert se.control_acc_and_steer_calls == [(123, 0.1, 20.0, 0.12)]


def test_ackermann_speed_target_ignores_acceleration_feedforward() -> None:
    from esmini_wrapper.esmini import Vehicle

    se = FakeSE()
    vehicle = Vehicle(se, x=1.0, y=2.0, h=0.3, length=4.5, speed=7.95)

    vehicle.apply_control(
        ControlCommand(
            mode=ControlMode.ACKERMANN,
            payload={"steer": 0.12, "speed": 8.0, "acceleration": -3.0},
        ),
        dt_s=0.1,
    )

    assert se.control_acc_and_steer_calls == [(123, 0.1, pytest.approx(1.0), 0.12)]


def test_ackermann_control_uses_acceleration_directly_without_speed_target() -> None:
    from esmini_wrapper.esmini import Vehicle

    se = FakeSE()
    vehicle = Vehicle(se, x=1.0, y=2.0, h=0.3, length=4.5, speed=4.0)

    vehicle.apply_control(
        ControlCommand(
            mode=ControlMode.ACKERMANN,
            payload={"steer": 0.12, "acceleration": 1.7},
        ),
        dt_s=0.1,
    )

    assert se.control_acc_and_steer_calls == [(123, 0.1, 1.7, 0.12)]


def test_step_failure_does_not_advance_wrapper_time() -> None:
    from esmini_wrapper.esmini import EsminiAdapter, ObjectKinematicData, ObjectStateData, Vehicle

    se = FakeSE(step_ret=-1)
    adapter = EsminiAdapter()
    adapter.se = se
    adapter.cfg = {}
    adapter._time_ns = 1_000_000_000
    adapter.obj_count = 1
    adapter._object_ids = [7]
    adapter.objects = [ObjectStateData(kinematic=ObjectKinematicData(time_ns=1_000_000_000))]
    adapter.ego_car = Vehicle(se, x=1.0, y=2.0, h=0.3, length=4.5, speed=4.0)

    with pytest.raises(SimulatorUnavailable, match="SE_StepDT failed"):
        adapter.step(
            StepRequest(
                ctrl_cmd=ControlCommand(mode=ControlMode.NONE),
                timestamp_ns=2_000_000_000,
            )
        )

    assert adapter._time_ns == 1_000_000_000


def test_step_returns_pisa_api_step_response() -> None:
    from esmini_wrapper.esmini import EsminiAdapter, ObjectKinematicData, ObjectStateData, Vehicle

    se = FakeSE()
    adapter = EsminiAdapter()
    adapter.se = se
    adapter.cfg = {}
    adapter._time_ns = 1_000_000_000
    adapter.obj_count = 1
    adapter._object_ids = [7]
    adapter.objects = [ObjectStateData(kinematic=ObjectKinematicData(time_ns=1_000_000_000))]
    adapter.ego_car = Vehicle(se, x=1.0, y=2.0, h=0.3, length=4.5, speed=4.0)

    response = adapter.step(
        StepRequest(
            ctrl_cmd=ControlCommand(mode=ControlMode.NONE),
            timestamp_ns=2_000_000_000,
        )
    )

    assert isinstance(response, StepResponse)
    assert response.frame.sim_time_ns == 2_000_000_000


def test_step_returns_wrapper_time_and_object_ids() -> None:
    from esmini_wrapper.esmini import EsminiAdapter, ObjectKinematicData, ObjectStateData, Vehicle

    se = FakeSE()
    adapter = EsminiAdapter()
    adapter.se = se
    adapter.cfg = {}
    adapter._time_ns = 1_000_000_000
    adapter.obj_count = 1
    adapter._object_ids = [7]
    adapter.objects = [ObjectStateData(kinematic=ObjectKinematicData(time_ns=1_000_000_000))]
    adapter.ego_car = Vehicle(se, x=1.0, y=2.0, h=0.3, length=4.5, speed=4.0)

    response = adapter.step(
        StepRequest(
            ctrl_cmd=ControlCommand(mode=ControlMode.NONE),
            timestamp_ns=2_000_000_000,
        )
    )

    frame = response.frame
    assert frame.sim_time_ns == 2_000_000_000
    assert frame.objects[0].kinematic.time_ns == 2_000_000_000
    assert frame.extras["object_ids"] == [7]
