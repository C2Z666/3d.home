#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/env_live.sh"
export PYTHONDONTWRITEBYTECODE=1
python3 -m unittest discover -s "$ROOT/tests" -p 'test_*.py' -v
python3 "$ROOT/tests/verify_assets.py"
