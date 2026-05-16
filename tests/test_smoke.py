"""Smoke tests for import-level wrapper integration."""


def test_public_imports_use_pisa_api_simulator_contract() -> None:
    from pisa_api.simulator import RuntimeFrameData as PisaRuntimeFrameData

    from esmini_wrapper.esmini import EsminiAdapter

    assert EsminiAdapter.reset.__annotations__["request"].__name__ == "ResetRequest"
    assert PisaRuntimeFrameData.__name__ == "RuntimeFrameData"
    assert EsminiAdapter is not None
