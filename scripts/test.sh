#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PYTHON="${PYTHON:-python3}"

if ! command -v "$PYTHON" >/dev/null 2>&1; then
    echo "Python interpreter not found: $PYTHON" >&2
    echo "Activate a project environment or set PYTHON explicitly." >&2
    exit 1
fi

PYTHON="$(command -v "$PYTHON")"

echo "Using Python: $("$PYTHON" -c 'import sys; print(sys.executable)')"

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
