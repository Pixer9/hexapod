# Vendored MicroPython USB support

These files are vendored for the Servo 2040 firmware so deployment does not
depend on running `mip install` on the controller.

Qualified package versions:

- `usb-device` 0.2.1
- `usb-device-cdc` 0.1.4

Upstream repository: `micropython/micropython-lib`

Source checkpoint used while preparing this candidate:
`9ec1830baa96aa8cad12324f45ff9f952016aac7`

Files included here:

- `usb/device/__init__.py`
- `usb/device/core.py`
- `usb/device/cdc.py`

The upstream modules are MIT licensed. The license text is included in
`LICENSE.micropython-lib.txt`.

Artifact SHA-256 checksums:

```text
c8d3315a67057aa55ba5addb3dc28a690bb468c1f518d17e89d8679bed098a10  device/__init__.py
8b57e9c9676bacecd18b773cd9b94271580051740a57881c8a2dbdf4c375e33d  device/core.py
9150679bb586e1200b50b4d8671996bb10c00f4832cec779e09214c7986b732b  device/cdc.py
```
