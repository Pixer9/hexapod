# Hexapod

Control software and firmware for a six-legged Raspberry Pi 5 robot using a
Pimoroni Servo 2040 as the deterministic actuator/safety controller.

## Architecture

The control boundary is:

> The Raspberry Pi decides what motion the robot should perform. The Servo 2040
> decides whether actuator commands are safe to execute.

The Raspberry Pi owns robot geometry, kinematics, locomotion, command
arbitration, controller input, and future autonomy/navigation.

The Servo 2040 owns actuator mapping, physical hard limits, hard command-rate
limits, lifecycle enforcement, watchdogs, and fail-safe output behavior.

See `docs/architecture/` and `docs/decisions/` for the detailed design.

## Known-good development baseline

The current validated host baseline is:

- Debian 13 (trixie), 64-bit ARM;
- Python 3.13.5;
- `evdev==2.0.0`;
- `pyserial==3.5`;
- `mpremote==1.29.0` installed with `pipx`.

The repository currently declares Python 3.13 as the supported CPython runtime.
Broader Python-version support should be declared only after CI validates it.

## Fresh Raspberry Pi setup

See:

```text
docs/setup/raspberry-pi.md
```

After the system prerequisites are installed:

```bash
git clone https://github.com/Pixer9/hexapod.git
cd hexapod

./scripts/bootstrap.sh --dev
source .venv/bin/activate

./scripts/test.sh
```

When committed lock files are present, `bootstrap.sh` uses them to reproduce the
known dependency set. Before the first lock generation, it installs directly
from `pyproject.toml`.

## Python package

The Raspberry Pi application code uses a standard `src/` package layout:

```text
src/hexapod/
```

The editable development install is created by the bootstrap script.

Runtime Python dependencies are declared in `pyproject.toml`.

## Servo 2040 firmware

Active firmware lives in:

```text
firmware/servo2040/src/
```

The archived V4 firmware under:

```text
firmware/servo2040/legacy-backup/
```

is historical reference material and is excluded from active quality gates.

See:

```text
docs/setup/servo2040.md
```

for the current board/tooling baseline.

## Tests

The repository currently has three host-side suites:

```bash
python -m unittest discover -s tests/firmware/servo2040 -p 'test_*.py' -v
python -m unittest discover -s tests/pi -p 'test_*.py' -v
python -m unittest discover -s tests/contracts -p 'test_*.py' -v
```

Or run all three using:

```bash
./scripts/test.sh
```

The `unittest` suites remain the known-good regression gate while pytest is
introduced as the common runner.

## Quality tooling

The development environment includes:

- Ruff for formatting and linting;
- Pyright for CPython static analysis;
- pytest and pytest-cov;
- pip-audit;
- pip-tools for reproducible dependency locks;
- pre-commit;
- Python package build tooling.

Quality policy is intentionally staged. The active Servo 2040 firmware is
linted, but CPython type checking initially targets `src/hexapod` only because
MicroPython/Pimoroni APIs require a separate typing strategy.

## Dependency locks

`pyproject.toml` is the source of dependency intent.

Exact environment locks are stored under:

```text
requirements/
```

See `requirements/README.md` for regeneration commands.

## Hardware-in-the-loop

HIL checks are separate from host CI. GitHub-hosted runners must never be
treated as substitutes for physical Servo 2040, actuator, watchdog, or E-stop
validation.
