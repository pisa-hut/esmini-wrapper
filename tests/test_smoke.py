"""Phase-1 placeholder so `pytest -x` exits 0 in CI.

Real wrapper tests come in Phase 2 — at that point we'll instantiate
the gRPC servicer and Ping it (catches import/constructor breakage).
"""


def test_placeholder() -> None:
    assert True
