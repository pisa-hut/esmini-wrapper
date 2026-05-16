# Esmini Wrapper

## Generic simulator server

Simulator implementations should not need to wire gRPC request/response objects directly.
`pisa-api` provides `serve_simulator()` and Python dataclasses for simulator lifecycle
methods:

```python
from pisa_api.simulator import (
    InitRequest,
    ResetRequest,
    RuntimeFrameData,
    StepRequest,
    serve_simulator,
)


class MySimulator:
    def init(self, request: InitRequest) -> None:
        ...

    def reset(self, request: ResetRequest):
        return RuntimeFrameData(sim_time_ns=0)

    def step(self, request: StepRequest):
        return RuntimeFrameData(sim_time_ns=request.timestamp_ns)

    def stop(self) -> None:
        ...

    def should_quit(self) -> bool:
        return False


serve_simulator(MySimulator(), name="MySimulator", scenario_formats={"open_scenario1"})
```

The generic server owns protobuf conversion, lifecycle checks, gRPC status mapping, and
request serialization through a lock. This wrapper reuses that shared `pisa-api`
contract instead of carrying its own gRPC glue.
