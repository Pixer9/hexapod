# Legacy Servo 2040 Firmware Backup

Backup captured from the existing Pimoroni Servo 2040 before the
Hexapod software architecture was rebuilt.

## Hardware

- Board: Pimoroni Servo 2040
- MCU: RP2040
- USB VID:PID: `2e8a:0005`
- USB serial: `e661410403724132`
- Stable Linux device path:
  `/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e661410403724132-if00`
- MicroPython `machine.unique_id()`:
  `e661410403724132`

## Runtime

- MicroPython: `1.23.0`
- Reported machine: `Raspberry Pi Pico with RP2040`
- Build:
  `MicroPython v1.23.0, pico v1.23.0-1 on 2024-06-06`

## Purpose

This directory is an archival copy of the firmware that was present on
the Servo 2040 before development of the new Hexapod control stack.

Do not modify these files in place. New Servo 2040 firmware should be
developed separately.