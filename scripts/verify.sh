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
        "Verification currently requires Python 3.13; "
        f"found {sys.version_info.major}.{sys.version_info.minor}"
    )
PY

echo "Using Python: $("$PYTHON" -c 'import sys; print(sys.executable)')"
echo

mapfile -t PYTHON_FILES < <(
    git ls-files '*.py' |
        grep -v '^firmware/servo2040/legacy-backup/'
)

if [[ ${#PYTHON_FILES[@]} -eq 0 ]]; then
    echo "No tracked Python files found." >&2
    exit 1
fi

echo "==> Checking working-tree whitespace"
git diff HEAD --check
echo

echo "==> Checking Ruff formatting"
"$PYTHON" -m ruff format --check "${PYTHON_FILES[@]}"
echo

echo "==> Running Ruff lint"
"$PYTHON" -m ruff check "${PYTHON_FILES[@]}"
echo

echo "==> Running Pyright"
"$PYTHON" -m pyright
echo

echo "==> Running regression tests"
PYTHON="$PYTHON" ./scripts/test.sh
echo

echo "==> Checking installed dependency consistency"
"$PYTHON" -m pip check
echo

echo "==> Auditing runtime dependencies"
"$PYTHON" -m pip_audit -r requirements/runtime.lock
echo

echo "==> Auditing development dependencies"
"$PYTHON" -m pip_audit -r requirements/dev.lock
echo

BUILD_OUT="$(mktemp -d)"

cleanup() {
    rm -rf \
        "$BUILD_OUT" \
        "$ROOT/build" \
        "$ROOT/src/hexapod.egg-info"
}

trap cleanup EXIT

echo "==> Building source distribution and wheel"
"$PYTHON" -m build --outdir "$BUILD_OUT"
echo

echo "==> Built artifacts"
find "$BUILD_OUT" -maxdepth 1 -type f -printf '%f\n' | sort
echo

echo "All verification checks passed."
