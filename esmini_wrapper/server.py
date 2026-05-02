import logging
import os
from concurrent import futures
from pprint import pprint

import grpc
from esmini import EsminiAdapter
from google.protobuf.json_format import MessageToDict
from pisa_api import sim_server_pb2, sim_server_pb2_grpc
from pisa_api.empty_pb2 import Empty
from pisa_api.pong_pb2 import Pong

logger = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler()],
)


class EsminiService(sim_server_pb2_grpc.SimServerServicer):
    def __init__(self):
        self._esmini = EsminiAdapter()
        self.initialized = False

    def Ping(self, request, context):
        logger.debug(f"Received ping from client: {context.peer()}")
        return Pong(msg="Esmini alive")

    def Init(self, request, context):
        logger.debug(f"Received Init request from client: {context.peer()}")
        config = MessageToDict(request.config.config)
        output_base = request.output_dir.path
        pprint(config)
        scenario = request.scenario
        if scenario.format != "open_scenario1":
            logger.error(f"Unsupported scenario format: {scenario.format}")
            return sim_server_pb2.SimServerMessages.InitResponse(
                success=False, msg=f"Unsupported scenario format: {scenario.format}"
            )

        try:
            self._esmini.init(config, output_base, scenario)
        except Exception:
            logger.exception("Failed to initialize Esmini")
            return sim_server_pb2.SimServerMessages.InitResponse(
                success=False, msg="Failed to initialize Esmini"
            )

        self.initialized = True
        return sim_server_pb2.SimServerMessages.InitResponse(success=True, msg="Esmini initialized")

    def Reset(self, request, context):
        logger.debug(f"Received Reset request from client: {context.peer()}")
        if not self.initialized:
            logger.error("Esmini adapter not initialized. Call Init first.")
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            context.set_details("Esmini adapter not initialized. Call Init first.")
            return sim_server_pb2.SimServerMessages.ResetResponse()

        output_related = request.output_dir.path
        sps = request.scenario_pack
        params = request.params
        try:
            objects = self._esmini.reset(output_related, sps, params)
        except RuntimeError as e:
            logger.error(f"Failed to reset Esmini: {e}")
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            context.set_details(f"Failed to reset Esmini: {e}")
            return sim_server_pb2.SimServerMessages.ResetResponse()
        return sim_server_pb2.SimServerMessages.ResetResponse(objects=objects)

    def Step(self, request, context):
        logger.debug(f"Received Step request with timestamp_ns={request.timestamp_ns}")
        if not self.initialized:
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            context.set_details("Esmini adapter not initialized. Call Init and Reset first.")
            return sim_server_pb2.SimServerMessages.StepResponse()
        ctrl_cmd = request.ctrl_cmd
        timestamp_ns = request.timestamp_ns
        try:
            objects = self._esmini.step(ctrl_cmd, timestamp_ns)
        except RuntimeError as e:
            logger.error(f"Failed to step Esmini: {e}")
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            context.set_details(f"Failed to step Esmini: {e}")
            return sim_server_pb2.SimServerMessages.StepResponse()
        return sim_server_pb2.SimServerMessages.StepResponse(objects=objects)

    def Stop(self, request, context):
        logger.debug(f"Received Stop request from client: {context.peer()}")
        if not self.initialized:
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            context.set_details("Esmini adapter not initialized. Call Init and Reset first.")
            return Empty()
        try:
            self._esmini.stop()
        except Exception as e:
            logger.error(f"Failed to stop Esmini: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"Failed to stop Esmini: {e}")
        return Empty()

    def ShouldQuit(self, request, context):
        logger.debug(f"Received ShouldQuit request from client: {context.peer()}")
        if not self.initialized:
            return sim_server_pb2.SimServerMessages.ShouldQuitResponse(should_quit=False)
        return sim_server_pb2.SimServerMessages.ShouldQuitResponse(
            should_quit=self._esmini.should_quit()
        )


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))

    sim_server_pb2_grpc.add_SimServerServicer_to_server(EsminiService(), server)

    PORT = os.environ.get("PORT", "50051")

    server.add_insecure_port(f"[::]:{PORT}")
    server.start()

    logger.info(f"Esmini gRPC server started on port {PORT}. Waiting for clients...")

    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        logger.info("Shutting down Esmini gRPC server...")
        server.stop(0)


if __name__ == "__main__":
    serve()
