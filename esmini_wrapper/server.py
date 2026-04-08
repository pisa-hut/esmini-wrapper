from pprint import pprint

import asyncio
import grpc
from concurrent import futures
import os
import logging

from google.protobuf.json_format import MessageToDict

from pisa_api import sim_server_pb2, sim_server_pb2_grpc
from pisa_api.pong_pb2 import Pong
from pisa_api.empty_pb2 import Empty
from esmini import EsminiAdapter

logger = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler()],
)


class EsminiService(sim_server_pb2_grpc.SimServerServicer):
    def __init__(self):
        self._esmini = None

    def Ping(self, request, context):
        logger.debug(f"Received ping from client: {context.peer()}")
        return Pong(msg="Esmini alive")

    def Init(self, request, context):
        logger.debug(f"Received Init request from client: {context.peer()}")
        config = MessageToDict(request.config.config)
        output_dir = request.output_dir.path
        self.dt = request.dt

        if self._esmini is None:
            self._esmini = EsminiAdapter(output_dir, config)
        pprint(config)

        return sim_server_pb2.SimServerMessages.InitResponse(
            success=True, msg="Esmini initialized"
        )

    def Reset(self, request, context):
        logger.debug(f"Received Reset request from client: {context.peer()}")
        output_dir = request.output_dir.path
        sps = request.scenario_pack
        params = request.params
        objects = self._esmini.reset(output_dir, sps, params)
        return sim_server_pb2.SimServerMessages.ResetResponse(objects=objects)

    def Step(self, request, context):
        logger.debug(f"Received Step request with timestamp_ns={request.timestamp_ns}")
        ctrl_cmd = request.ctrl_cmd
        timestamp_ns = request.timestamp_ns
        objects = self._esmini.step(ctrl_cmd, timestamp_ns)
        return sim_server_pb2.SimServerMessages.StepResponse(objects=objects)

    def Stop(self, request, context):
        logger.debug(f"Received Stop request from client: {context.peer()}")
        self._esmini.stop()
        return Empty()

    def ShouldQuit(self, request, context):
        return sim_server_pb2.SimServerMessages.ShouldQuitResponse(
            should_quit=self._esmini.should_quit()
        )


def serve():
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=4),
        options=[
            ("grpc.keepalive_time_ms", 10000),
            ("grpc.keepalive_timeout_ms", 5000),
            ("grpc.keepalive_permit_without_calls", True),
        ],
    )

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
