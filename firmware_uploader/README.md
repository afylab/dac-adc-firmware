# GateKeeper Firmware Installation Guide

This directory contains the user-facing uploader for compiled GateKeeper firmware releases. If there's no `firmware` directory present, download it from the [latest firmware release](https://github.com/sp2quantum/gatekeeper-firmware/releases).

## 1. Make sure you have `dfu-util` installed

- **Linux (Debian)**: `sudo apt install dfu-util`
- **MacOS**: `brew install dfu-util`
- **Windows**:
  - Download the `dfu-util` to your local system, e.g., under `D:\dfu-util`.
  - Rename it to `dfu-util.exe`.
  - Append the path of the `dfu-util.exe` to the system environment variable PATH.

## 2. Run `pip install -r requirements.txt`

## 3. Plug in Arduino Giga

## 4. Run `python3 upload_firmware.py {serial suffix}`

The serial suffix is optional and can be up to 3 characters. If omitted, the uploader reuses the connected device suffix and updates the year dynamically.

## 5. To modify the serial number of an existing device, run `python3 patch_serial_number.py {serial suffix}`

If more than one Arduino GIGA is connected, the scripts list the detected serial
ports and ask which board to use. You can also pass `--port <port>` to select a
port explicitly.

## 6. Feel free to email [markzakharyan@sp2quantum.com](mailto:markzakharyan@sp2quantum.com) if something isn't working
