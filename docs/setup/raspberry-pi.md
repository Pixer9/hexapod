# Raspberry Pi Setup

## Validated baseline

The current known-good host environment is:

```text
OS             Debian 13 (trixie), Raspberry Pi OS family
Architecture   aarch64
Python         3.13.5
Git            2.47.3
pipx           1.7.1
BlueZ          5.82
```

The project venv baseline that passed the rc3 regression/HIL checkpoint used:

```text
evdev       2.0.0
pyserial    3.5
```

`mpremote` is intentionally installed with `pipx`, outside the project venv:

```text
mpremote    1.29.0
```

## System prerequisites

On a fresh Debian 13 / Raspberry Pi OS installation:

```bash
sudo apt update

sudo apt install \
  git \
  python3 \
  python3-venv \
  python3-dev \
  build-essential \
  pipx \
  bluez
```

The current robot user belongs to the Linux device-access groups used by the
hardware stack, including:

```text
dialout
input
gpio
i2c
spi
```

If a fresh user is missing them:

```bash
sudo usermod -aG dialout,input,gpio,i2c,spi "$USER"
```

Log out and back in after changing group membership.

## mpremote

Install the validated Servo 2040 maintenance/deployment CLI with pipx:

```bash
pipx ensurepath
pipx install 'mpremote==1.29.0'
```

Open a new shell after `pipx ensurepath` if needed.

## Clone and bootstrap

```bash
git clone https://github.com/Pixer9/hexapod.git
cd hexapod

./scripts/bootstrap.sh --dev
source .venv/bin/activate
```

The bootstrap process currently requires Python 3.13 and installs the committed,
hash-verified dependency lock for the selected environment. Package installation
uses the same PyPI source policy as lock generation rather than inheriting
machine-level pip index configuration.

## Verify

Run the canonical full repository verification gate:

```bash
./scripts/verify.sh
```

This checks formatting, linting, CPython static analysis, all host-side
regression tests, dependency consistency and security, and package builds.

For the faster regression-test-only gate:

```bash
./scripts/test.sh
```

## Device paths

The Servo 2040 transport configuration uses the stable USB by-id path rather
than `/dev/ttyACM0`.

The current configured Servo 2040 path is stored in:

```text
config/hardware/servo2040.json
```

Do not replace stable by-id paths with transient tty device numbers.
