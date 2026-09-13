#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MODE="runtime"
if [[ "${1:-}" == "--dev" ]]; then
    MODE="dev"
elif [[ $# -gt 0 ]]; then
    echo "usage: $0 [--dev]" >&2
    exit 2
fi

PYTHON="${PYTHON:-python3}"

"$PYTHON" - <<'PY'
import sys

if sys.version_info[:2] != (3, 13):
    raise SystemExit(
        "Hexapod currently requires Python 3.13.x; "
        f"found {sys.version.split()[0]}"
    )
PY

if [[ ! -d .venv ]]; then
    "$PYTHON" -m venv .venv
fi

VENV_PY="$ROOT/.venv/bin/python"

"$VENV_PY" -m pip install --upgrade pip setuptools wheel

if [[ "$MODE" == "dev" ]]; then
    if [[ -f requirements/dev.lock ]]; then
        "$VENV_PY" -m pip install --require-hashes -r requirements/dev.lock
        "$VENV_PY" -m pip install --no-deps -e .
    else
        "$VENV_PY" -m pip install -e '.[dev]'
    fi
else
    if [[ -f requirements/runtime.lock ]]; then
        "$VENV_PY" -m pip install --require-hashes -r requirements/runtime.lock
        "$VENV_PY" -m pip install --no-deps -e .
    else
        "$VENV_PY" -m pip install -e .
    fi
fi

echo
echo "Bootstrap complete."
echo "Activate with:"
echo "  source .venv/bin/activate"
