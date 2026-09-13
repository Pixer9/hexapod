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

if ! command -v "$PYTHON" >/dev/null 2>&1; then
    echo "Python interpreter not found: $PYTHON" >&2
    exit 1
fi

PYTHON="$(command -v "$PYTHON")"

check_python_313() {
    "$1" - <<'PY'
import sys

if sys.version_info[:2] != (3, 13):
    raise SystemExit(
        "Hexapod currently requires Python 3.13.x; "
        f"found {sys.version.split()[0]}"
    )
PY
}

check_python_313 "$PYTHON"

if [[ ! -d .venv ]]; then
    "$PYTHON" -m venv .venv
fi

VENV_PY="$ROOT/.venv/bin/python"

if [[ ! -x "$VENV_PY" ]]; then
    echo "Existing .venv is not a usable Python virtual environment." >&2
    echo "Remove .venv and rerun bootstrap." >&2
    exit 1
fi

# Validate the environment itself rather than assuming an existing .venv was
# created by the currently selected host interpreter.
check_python_313 "$VENV_PY"

# Dependency locks are compiled against PyPI with host pip configuration
# disabled. Bootstrap uses the same package-source policy so machine-level
# settings such as Raspberry Pi OS's PiWheels extra index cannot change the
# resolved artifacts.
export PIP_CONFIG_FILE=/dev/null
export PIP_INDEX_URL=https://pypi.org/simple
unset PIP_EXTRA_INDEX_URL
unset PIP_FIND_LINKS
unset PIP_NO_INDEX

"$VENV_PY" -m pip install \
    "pip==26.2.1" \
    "setuptools==84.0.0" \
    "wheel==0.48.0"

if [[ "$MODE" == "dev" ]]; then
    if [[ -f requirements/dev.lock ]]; then
        "$VENV_PY" -m pip install \
            --require-hashes \
            -r requirements/dev.lock
        "$VENV_PY" -m pip install --no-deps -e .
    else
        echo "Development lock not found: requirements/dev.lock" >&2
        exit 1
    fi
else
    if [[ -f requirements/runtime.lock ]]; then
        "$VENV_PY" -m pip install \
            --require-hashes \
            -r requirements/runtime.lock
        "$VENV_PY" -m pip install --no-deps -e .
    else
        echo "Runtime lock not found: requirements/runtime.lock" >&2
        exit 1
    fi
fi

echo
echo "Bootstrap complete."
echo "Activate with:"
echo "  source .venv/bin/activate"
