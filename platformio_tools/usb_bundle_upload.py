#!/usr/bin/env python3
import argparse
import importlib.util
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import serial


ROOT_DIR = Path(__file__).resolve().parents[1]
M4_PROJECT_DIR = ROOT_DIR / "m4"
M7_PROJECT_DIR = ROOT_DIR / "m7"

DEFAULT_HARDWARE_VARIANT = "new_hardware"
DEFAULT_M4_ENV = "gatekeeper_m4_usb_gateway"
DEFAULT_M7_ENV = "gatekeeper_new_hardware"

HARDWARE_VARIANTS = {
    "new_hardware": {
        "m4_env": DEFAULT_M4_ENV,
        "m7_env": "gatekeeper_new_hardware",
        "environment": "NEW_HARDWARE",
    },
    "old_hardware": {
        "m4_env": DEFAULT_M4_ENV,
        "m7_env": "gatekeeper_old_hardware",
        "environment": "OLD_HARDWARE",
    },
    "new_shield_old_dac_adc": {
        "m4_env": DEFAULT_M4_ENV,
        "m7_env": "gatekeeper_new_shield_old_dac_adc",
        "environment": "NEW_SHIELD_OLD_DAC_ADC",
    },
}

M7_ENVIRONMENT_MAP = {
    "giga_r1_m7": "NEW_HARDWARE",
    "gatekeeper_new_hardware": "NEW_HARDWARE",
    "gatekeeper_old_hardware": "OLD_HARDWARE",
    "gatekeeper_new_shield_old_dac_adc": "NEW_SHIELD_OLD_DAC_ADC",
}

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
    helper_path = ROOT_DIR / "firmware_uploader" / "gatekeeper_upload.py"
    spec = importlib.util.spec_from_file_location(
        "gatekeeper_upload", helper_path
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


def run(command, cwd=None, env=None):
    log("$ " + " ".join(str(part) for part in command))
    subprocess.run(command, cwd=cwd, env=env, check=True)


def platformio_env(build_dir=None):
    if build_dir is None:
        return None
    child_env = os.environ.copy()
    child_env["PLATFORMIO_BUILD_DIR"] = str(build_dir)
    return child_env


def build_platformio_env(project_dir, pioenv, build_dir=None):
    pio = find_executable("pio", "platformio")
    if pio is None:
        raise RuntimeError("PlatformIO executable not found.")
    run([pio, "run", "-e", pioenv], cwd=project_dir, env=platformio_env(build_dir))


def clean_platformio_env(project_dir, pioenv, build_dir=None):
    pio = find_executable("pio", "platformio")
    if pio is None:
        raise RuntimeError("PlatformIO executable not found.")
    run(
        [pio, "run", "-e", pioenv, "-t", "clean"],
        cwd=project_dir,
        env=platformio_env(build_dir),
    )


def firmware_bin(project_dir, pioenv, build_dir=None):
    path = (build_dir or (project_dir / ".pio" / "build")) / pioenv / "firmware.bin"
    if not path.exists():
        raise RuntimeError(f"Built firmware binary not found: {path}")
    return path


def resolve_upload_selection(args):
    if args.hardware is None and args.m4_env is None and args.m7_env is None:
        args.hardware = DEFAULT_HARDWARE_VARIANT

    expected_environment = None
    if args.hardware is not None:
        variant = HARDWARE_VARIANTS[args.hardware]
        args.m4_env = args.m4_env or variant["m4_env"]
        args.m7_env = args.m7_env or variant["m7_env"]
        expected_environment = variant["environment"]

    args.m4_env = args.m4_env or DEFAULT_M4_ENV
    args.m7_env = args.m7_env or DEFAULT_M7_ENV
    return expected_environment or M7_ENVIRONMENT_MAP.get(args.m7_env)


def bundle_build_dirs(args):
    if args.hardware is None:
        return None, None

    root_build_dir = ROOT_DIR / ".pio" / "build" / args.hardware
    return (
        root_build_dir / "m4",
        root_build_dir / "m7",
    )


def build_bundle(args):
    m4_build_dir, real_m7_build_dir = bundle_build_dirs(args)
    if not args.skip_build:
        if not args.skip_m4_build:
            build_platformio_env(M4_PROJECT_DIR, args.m4_env, m4_build_dir)
        if not args.skip_real_m7_build:
            build_platformio_env(M7_PROJECT_DIR, args.m7_env, real_m7_build_dir)
        refresh_intellisense(args.hardware)

    m4_bin = firmware_bin(M4_PROJECT_DIR, args.m4_env, m4_build_dir)
    real_m7_bin = firmware_bin(M7_PROJECT_DIR, args.m7_env, real_m7_build_dir)
    return m4_bin, real_m7_bin


def clean_bundle(args):
    m4_build_dir, real_m7_build_dir = bundle_build_dirs(args)
    clean_platformio_env(M4_PROJECT_DIR, args.m4_env, m4_build_dir)
    clean_platformio_env(M7_PROJECT_DIR, args.m7_env, real_m7_build_dir)


def refresh_intellisense(hardware):
    if hardware is None:
        return

    script_path = ROOT_DIR / "platformio_tools" / "update_compile_commands.py"
    if not script_path.exists():
        return

    try:
        run(
            [
                sys.executable,
                str(script_path),
                "--hardware",
                hardware,
                "--no-compiledb",
            ]
        )
    except subprocess.CalledProcessError as exc:
        log(f"IntelliSense refresh failed after build: {exc}")


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
            "Upload a complete GateKeeper firmware bundle to Arduino GIGA "
            "over USB DFU."
        )
    )
    parser.add_argument(
        "--hardware",
        choices=sorted(HARDWARE_VARIANTS),
        help=(
            "GateKeeper hardware variant to build and upload. Defaults to "
            f"{DEFAULT_HARDWARE_VARIANT} when no explicit M4/M7 env is given."
        ),
    )
    parser.add_argument(
        "--port",
        help="Serial port to trigger DFU from. If omitted, a connected GIGA is detected.",
    )
    parser.add_argument("--m4-env")
    parser.add_argument("--m7-env")
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
        help="Use already-built M4 and M7 binaries.",
    )
    parser.add_argument(
        "--build-only",
        action="store_true",
        help="Build the complete firmware bundle without uploading.",
    )
    parser.add_argument(
        "--clean-only",
        action="store_true",
        help="Clean the complete firmware bundle without uploading.",
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
    expected_environment = resolve_upload_selection(args)
    persistence = load_upload_persistence()
    upload_env = UploadEnv(
        M7_PROJECT_DIR,
        args.m7_env,
        upload_port=args.port,
        backup_dir=args.calibration_backup_dir,
    )

    if args.build_only:
        build_bundle(args)
        log(
            "Firmware bundle build complete "
            f"(m4={args.m4_env}, m7={args.m7_env})."
        )
        return

    if args.clean_only:
        clean_bundle(args)
        log(
            "Firmware bundle clean complete "
            f"(m4={args.m4_env}, m7={args.m7_env})."
        )
        return

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

    if (
        expected_environment is not None
        and not state.get("skip")
        and state.get("source_environment") != expected_environment
    ):
        raise RuntimeError(
            "Connected board environment "
            f"{state['source_environment']} does not match requested upload "
            f"environment {expected_environment}."
        )

    m4_bin, real_m7_bin = build_bundle(args)

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

    log("Uploading M4 without leaving DFU...")
    dfu_download(dfu_util, M4_ADDRESS, m4_bin)
    log("Uploading M7 and leaving DFU...")
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
