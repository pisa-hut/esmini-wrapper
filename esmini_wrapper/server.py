from pisa_api.simulator import serve_simulator
from pisa_api.wrapper import setup_logging

from .esmini import EsminiAdapter

setup_logging()


if __name__ == "__main__":
    serve_simulator(
        EsminiAdapter(),
        name="Esmini",
        scenario_formats={"open_scenario1"},
    )
