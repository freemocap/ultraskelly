#!/usr/bin/env bash
# Rebuild the vendored dashboard client from the same SDK revision as the Python producer.
set -euo pipefail
exec python3 "$(dirname "$0")/build_sdk_candidate.py" "${1:?usage: $0 /path/to/sdk}" --vendor
