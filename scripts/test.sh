#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PYTHON="${PYTHON:-$ROOT/.venv/bin/python}"

if [[ ! -x "$PYTHON" ]]; then
    echo "Python environment not found: $PYTHON" >&2
    echo "Run ./scripts/bootstrap.sh --dev first." >&2
    exit 1
fi

"$PYTHON" -m unittest discover \
  -s tests/firmware/servo2040 \
  -p 'test_*.py' \
  -v

"$PYTHON" -m unittest discover \
  -s tests/pi \
  -p 'test_*.py' \
  -v

"$PYTHON" -m unittest discover \
  -s tests/contracts \
  -p 'test_*.py' \
  -v
