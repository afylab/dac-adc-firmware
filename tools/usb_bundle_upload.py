#!/usr/bin/env python3
import argparse
import importlib.util
import shutil
import subprocess
import sys
import time
from pathlib import Path

import serial


ROOT_DIR = Path(__file__).resolve().parents[1]
M4_PROJECT_DIR = ROOT_DIR / "m4"
M7_PROJECT_DIR = ROOT_DIR / "m7"

DEFAULT_M4_ENV = "giga_r1_m4_new_hardware"
DEFAULT_M7_ENV = "giga_r1_m7"
DEFAULT_M7_GUARD_ENV = "giga_r1_m7_usb_upload_guard"

ARDUINO_GIGA_DFU_DEVICE_ID = "2341:0366"
M7_ADDRESS = "0x08040000"
M4_ADDRESS = "0x08100000"
M7_LEAVE_ADDRESS = f"{M7_ADDRESS}:leave"
DFU_AUTO_WAIT_S = 8
DFU_MANUAL_WAIT_S = 60


def log(message):
    print(f"[usb-bundle-upload] {message}", flush=True)


class UploadEnv:
    def __init__(self, project_dir, pioenv, upload_port=None, backup_dir=None):
        self.project_dir = Path(project_dir)
        self.pioenv = pioenv
        self.upload_port = upload_port
        self.backup_dir = backup_dir

    def subst(self, value):
        if value == "$PROJECT_DIR":
            return str(self.project_dir)
        if value == "$PIOENV":
            return self.pioenv
        if value == "$UPLOAD_PORT":
            return self.upload_port or "$UPLOAD_PORT"
        if value == "$BUILD_DIR":
            return str(self.project_dir / ".pio" / "build" / self.pioenv)

        result = value
        replacements = {
            "$PROJECT_DIR": str(self.project_dir),
            "$PIOENV": self.pioenv,
            "$UPLOAD_PORT": self.upload_port or "$UPLOAD_PORT",
            "$BUILD_DIR": str(self.project_dir / ".pio" / "build" / self.pioenv),
            "${PROGNAME}": "firmware",
            "$PROGNAME": "firmware",
        }
        for key, replacement in replacements.items():
            result = result.replace(key, replacement)
        return result

    def get(self, key, default=None):
        if key == "CALIBRATION_BACKUP_DIR" and self.backup_dir is not None:
            return str(self.backup_dir)
        return default


def load_upload_persistence():
    helper_path = ROOT_DIR / "tools" / "platformio_upload_persistence.py"
    spec = importlib.util.spec_from_file_location(
        "platformio_upload_persistence", helper_path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def find_executable(*names):
    for name in names:
        path = shutil.which(name)
        if path:
            return path
    return None


def find_dfu_util():
    path = find_executable("dfu-util")
    if path:
        return path

    candidates = [
        Path.home() / ".platformio" / "packages" / "tool-dfuutil" / "bin" / "dfu-util",
        Path.home() / ".platformio" / "packages" / "tool-dfuutil-arduino" / "dfu-util",
        Path.home()
        / ".platformio"
        / "packages"
        / "tool-stm32duino"
        / "dfu-util"
        / "dfu-util",
    ]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return None


def run(command, cwd=None):
    log("$ " + " ".join(str(part) for part in command))
    subprocess.run(command, cwd=cwd, check=True)


def build_platformio_env(project_dir, pioenv):
    pio = find_executable("pio", "platformio")
    if pio is None:
        raise RuntimeError("PlatformIO executable not found.")
    run([pio, "run", "-e", pioenv], cwd=project_dir)


def firmware_bin(project_dir, pioenv):
    path = project_dir / ".pio" / "build" / pioenv / "firmware.bin"
    if not path.exists():
        raise RuntimeError(f"Built firmware binary not found: {path}")
    return path


def dfu_list_output(dfu_util):
    result = subprocess.run(
        [dfu_util, "-l", "-d", ARDUINO_GIGA_DFU_DEVICE_ID],
        check=False,
        capture_output=True,
        text=True,
    )
    return f"{result.stdout}\n{result.stderr}"


def dfu_device_present(dfu_util):
    return "Found DFU" in dfu_list_output(dfu_util)


def wait_for_dfu_device(dfu_util, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if dfu_device_present(dfu_util):
            return True
        time.sleep(0.5)
    return False


def trigger_dfu_mode(dfu_util, port):
    log(f"Triggering DFU mode from {port}...")
    try:
        with serial.Serial(port, 1200, timeout=1):
            pass
    except serial.SerialException as exc:
        log(f"1200-baud touch failed: {exc}")
    if wait_for_dfu_device(dfu_util, DFU_AUTO_WAIT_S):
        return

    log(
        "DFU did not appear after the 1200-baud touch. "
        "Double-tap reset now if this board is running older firmware."
    )
    if not wait_for_dfu_device(dfu_util, DFU_MANUAL_WAIT_S):
        raise RuntimeError("Timed out waiting for USB DFU mode.")


def dfu_download(dfu_util, address, binary_path):
    run(
        [
            dfu_util,
            "-d",
            ARDUINO_GIGA_DFU_DEVICE_ID,
            "-a",
            "0",
            "-s",
            address,
            "-D",
            str(binary_path),
        ]
    )


def patch_binary_if_supported(persistence, binary_path, serial_number):
    if persistence.binary_contains_serial_marker(binary_path):
        persistence.patch_binary_serial(binary_path, serial_number)
        log(f"Patched {binary_path.name} with serial {serial_number}.")
        return True
    return False


def upload_serial_number(persistence, state):
    if state.get("skip"):
        serial_number = state.get("serial_number")
        if serial_number:
            return persistence.serial_with_current_year(serial_number)
        return persistence.default_serial_with_current_year()
    return state["serial_number"]


def verify_basic_firmware(persistence, upload_env, expected_serial_number):
    ready_port = persistence.wait_for_device_ready(
        upload_env, expected_serial_number=expected_serial_number
    )
    with persistence.open_command_port(ready_port) as ser:
        nop = persistence.send_command(ser, "NOP")
        ready = persistence.send_command(ser, "*RDY?")
        environment = persistence.send_command(ser, "GET_ENVIRONMENT")
        serial_number = persistence.send_command(ser, "SERIAL_NUMBER")

    if nop != "NOP":
        raise RuntimeError(f"NOP check failed after upload: {nop}")
    if ready != "READY":
        raise RuntimeError(f"READY check failed after upload: {ready}")
    if expected_serial_number and serial_number != expected_serial_number:
        raise RuntimeError(
            "Serial number changed across upload: "
            f"expected {expected_serial_number}, got {serial_number}"
        )

    log(
        "Basic firmware verification passed "
        f"on {ready_port} (env={environment}, serial={serial_number})."
    )
    return ready_port


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Upload M4 and M7 firmware to Arduino GIGA over USB DFU as one "
            "safe bundle."
        )
    )
    parser.add_argument(
        "--port",
        help="Serial port to trigger DFU from. If omitted, a connected GIGA is detected.",
    )
    parser.add_argument("--m4-env", default=DEFAULT_M4_ENV)
    parser.add_argument("--m7-env", default=DEFAULT_M7_ENV)
    parser.add_argument("--m7-guard-env", default=DEFAULT_M7_GUARD_ENV)
    parser.add_argument(
        "--skip-real-m7-build",
        action="store_true",
        help="Use the already-built real M7 binary. PlatformIO's usb_upload target uses this.",
    )
    parser.add_argument(
        "--skip-m4-build",
        action="store_true",
        help="Use the already-built M4 binary. PlatformIO's M4 upload target uses this.",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Use already-built M4, guard M7, and real M7 binaries.",
    )
    parser.add_argument(
        "--no-restore",
        action="store_true",
        help="Do not restore or verify calibration after upload.",
    )
    parser.add_argument(
        "--calibration-backup-dir",
        type=Path,
        help="Optional calibration backup directory.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    persistence = load_upload_persistence()
    upload_env = UploadEnv(
        M7_PROJECT_DIR,
        args.m7_env,
        upload_port=args.port,
        backup_dir=args.calibration_backup_dir,
    )

    dfu_util = find_dfu_util()
    if dfu_util is None:
        raise RuntimeError("dfu-util not found.")

    port = args.port or persistence.find_giga_port(upload_env)
    if port is not None:
        log(f"Backing up current calibration/serial state from {port}...")
        state = persistence.backup_device_state(port)
        state["source_port"] = port
        if state.get("skip"):
            log(
                "Current firmware did not provide calibration/serial state; "
                f"continuing without persistence ({state.get('skip_reason')})."
            )
        else:
            backup_path = persistence.write_calibration_backup_file(upload_env, state)
            state["calibration_backup_path"] = str(backup_path)
            log(
                "Backed up device state "
                f"(env={state['source_environment']}, serial={state['serial_number']})."
            )
    elif dfu_device_present(dfu_util):
        log("Found Arduino GIGA already in USB DFU mode.")
        state = {
            "skip": True,
            "skip_reason": "already_in_dfu",
            "source_port": None,
        }
    else:
        log(
            "No Arduino GIGA serial port or DFU device found. "
            "Double-tap reset now to enter the bootloader."
        )
        if not wait_for_dfu_device(dfu_util, DFU_MANUAL_WAIT_S):
            raise RuntimeError("Arduino GIGA not found as serial or USB DFU.")
        state = {
            "skip": True,
            "skip_reason": "manual_dfu",
            "source_port": None,
        }

    if not args.skip_build:
        build_platformio_env(M7_PROJECT_DIR, args.m7_guard_env)
        if not args.skip_m4_build:
            build_platformio_env(M4_PROJECT_DIR, args.m4_env)
        if not args.skip_real_m7_build:
            build_platformio_env(M7_PROJECT_DIR, args.m7_env)

    guard_m7_bin = firmware_bin(M7_PROJECT_DIR, args.m7_guard_env)
    m4_bin = firmware_bin(M4_PROJECT_DIR, args.m4_env)
    real_m7_bin = firmware_bin(M7_PROJECT_DIR, args.m7_env)

    serial_number = upload_serial_number(persistence, state)
    state["serial_number"] = serial_number
    patch_binary_if_supported(persistence, m4_bin, serial_number)
    patch_binary_if_supported(persistence, real_m7_bin, serial_number)

    if port is not None:
        trigger_dfu_mode(dfu_util, port)
    elif not dfu_device_present(dfu_util):
        raise RuntimeError("Arduino GIGA USB DFU device disappeared before upload.")
    else:
        log("Using existing Arduino GIGA USB DFU device.")

    log("Uploading guard M7 without leaving DFU...")
    dfu_download(dfu_util, M7_ADDRESS, guard_m7_bin)
    log("Uploading M4 without leaving DFU...")
    dfu_download(dfu_util, M4_ADDRESS, m4_bin)
    log("Uploading real M7 and leaving DFU...")
    dfu_download(dfu_util, M7_LEAVE_ADDRESS, real_m7_bin)

    if args.no_restore:
        log("USB bundle upload complete.")
        return

    if state.get("skip"):
        verify_basic_firmware(persistence, upload_env, serial_number)
        log("USB bundle upload and basic verification complete.")
        return

    ready_port = persistence.wait_for_device_ready(
        upload_env, expected_serial_number=state["serial_number"]
    )
    log(f"Restoring calibration on {ready_port}...")
    persistence.restore_calibration(ready_port, state)
    persistence.verify_calibration(ready_port, state)
    log("USB bundle upload, calibration restore, and verification complete.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001
        print(f"[usb-bundle-upload] ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
