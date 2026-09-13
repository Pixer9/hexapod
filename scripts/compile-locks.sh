#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PYTHON="${PYTHON:-python3}"

if ! command -v "$PYTHON" >/dev/null 2>&1; then
    echo "Python interpreter not found: $PYTHON" >&2
    echo "Activate the project development environment or set PYTHON explicitly." >&2
    exit 1
fi

PYTHON="$(command -v "$PYTHON")"

"$PYTHON" - <<'PY'
import sys

if sys.version_info[:2] != (3, 13):
    raise SystemExit(
        "Dependency locks must currently be compiled with Python 3.13; "
        f"found {sys.version_info.major}.{sys.version_info.minor}"
    )
PY

if ! "$PYTHON" -c 'import piptools' >/dev/null 2>&1; then
    echo "pip-tools is not installed in the selected Python environment." >&2
    echo "Run ./scripts/bootstrap.sh --dev first." >&2
    exit 1
fi

export PIP_CONFIG_FILE=/dev/null
export CUSTOM_COMPILE_COMMAND="./scripts/compile-locks.sh"

COMMON_ARGS=(
    --resolver=backtracking
    --strip-extras
    --generate-hashes
    --index-url=https://pypi.org/simple
    --no-emit-index-url
)

echo "Compiling runtime dependency lock..."
"$PYTHON" -m piptools compile \
    "${COMMON_ARGS[@]}" \
    --output-file=requirements/runtime.lock \
    pyproject.toml

echo "Compiling development dependency lock..."
"$PYTHON" -m piptools compile \
    "${COMMON_ARGS[@]}" \
    --extra=dev \
    --output-file=requirements/dev.lock \
    pyproject.toml

echo
echo "Dependency locks compiled successfully."
