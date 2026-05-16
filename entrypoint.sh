#!/bin/bash
pushd /app
uv run python -m esmini_wrapper.server
popd
