import argparse
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

from gatekeeper_upload import (
    DEFAULT_CALIBRATION_PATH,
    SERIAL_PATTERN,
    backup_device_state,
    binary_contains_serial_marker,
    choose_giga_port,
    default_serial_with_current_year,
    get_available_path,
    load_calibration_state,
    open_command_port,
    patch_binary_serial,
    restore_calibration,
    send_command,
    serial_with_current_year,
    verify_calibration,
    wait_for_giga_port,
    wait_for_ready,
    write_calibration_state_file,
)


ARDUINO_GIGA_DFU_DEVICE_ID = "2341:0366"
DFU_AUTO_WAIT_S = 8
DFU_MANUAL_WAIT_S = 60
M4_ADDRESS = "0x08100000"
M7_ADDRESS = "0x08040000"
M7_LEAVE_ADDRESS = f"{M7_ADDRESS}:leave"

M4_FIRMWARE_NAME = "firmwareM4_new_hardware.bin"
M7_FIRMWARE_NAME = "firmwareM7.bin"


def log(message):
    print(f"[upload-firmware] {message}")


def find_dfu_util():
    return shutil.which("dfu-util")


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
        import serial

        with serial.Serial(port, 1200, timeout=1):
            pass
    except Exception as exc:  # noqa: BLE001
        log(f"1200-baud touch failed: {exc}")

    if wait_for_dfu_device(dfu_util, DFU_AUTO_WAIT_S):
        return

    log(
        "DFU did not appear after the 1200-baud touch. "
        "Double-tap reset now to enter the bootloader."
    )
    if not wait_for_dfu_device(dfu_util, DFU_MANUAL_WAIT_S):
        raise RuntimeError("Timed out waiting for USB DFU mode.")


def dfu_download(dfu_util, address, firmware_path):
    command = [
        dfu_util,
        "-d",
        ARDUINO_GIGA_DFU_DEVICE_ID,
        "-a",
        "0",
        "-s",
        address,
        "-D",
        str(firmware_path),
    ]
    log("$ " + " ".join(command))
    subprocess.run(command, check=True)


def resolve_serial_number(port, serial_suffix):
    if serial_suffix is not None:
        if len(serial_suffix) > 3:
            raise RuntimeError("Serial suffix must be at most 3 characters.")
        return f"{default_serial_with_current_year()[:-3]}{serial_suffix.zfill(3)}"

    if port is None:
        return default_serial_with_current_year()

    try:
        with open_command_port(port) as ser:
            wait_for_ready(ser)
            existing_serial = send_command(ser, "SERIAL_NUMBER")
        return serial_with_current_year(existing_serial)
    except Exception as exc:  # noqa: BLE001
        log(f"Could not read existing GateKeeper serial; using default: {exc}")
        return default_serial_with_current_year()


def backup_calibration_if_available(port):
    if port is None:
        return None

    try:
        state = backup_device_state(port)
    except Exception as exc:  # noqa: BLE001
        log(f"Skipping calibration backup: {exc}")
        return None

    if state.get("skip"):
        log(f"Skipping calibration backup: {state.get('skip_reason')}")
        return None
    return state


def prepare_patched_firmware(source_path, serial_number, output_dir):
    if not SERIAL_PATTERN.fullmatch(serial_number):
        raise RuntimeError("Invalid serial number format. Expected DA_<year>_XXX.")

    patched_path = Path(output_dir) / Path(source_path).name
    shutil.copy2(source_path, patched_path)
    if not binary_contains_serial_marker(patched_path):
        raise RuntimeError(f"Serial marker not found in {source_path}.")
    patch_binary_serial(patched_path, serial_number)
    return patched_path


def upload_bundle(dfu_util, m4_path, m7_path):
    log("Uploading M4 without leaving DFU...")
    dfu_download(dfu_util, M4_ADDRESS, m4_path)
    log("Uploading M7 and leaving DFU...")
    dfu_download(dfu_util, M7_LEAVE_ADDRESS, m7_path)


def nop_test(port, expected_serial_number):
    with open_command_port(port) as ser:
        log(f"Testing NOP on {ser.port}...")
        wait_for_ready(ser)
        response = send_command(ser, "NOP")
        if response != "NOP":
            raise RuntimeError(f"NOP test failed: expected NOP, got {response}")

        device_id = send_command(ser, "*IDN?")
        serial_number = send_command(ser, "SERIAL_NUMBER")
        if serial_number != expected_serial_number:
            raise RuntimeError(
                f"Serial number mismatch: expected {expected_serial_number}, got {serial_number}"
            )
        log(f"Firmware uploaded successfully. ID={device_id}, serial={serial_number}.")


def main():
    parser = argparse.ArgumentParser(
        description="Upload compiled GateKeeper firmware to an Arduino GIGA."
    )
    parser.add_argument(
        "serial_number",
        nargs="?",
        default=None,
        help="Optional serial suffix, up to 3 characters. If omitted, reuse the current device suffix.",
    )
    parser.add_argument(
        "--port",
        help="Serial port to use. If omitted, detected Arduino GIGA ports are listed for selection.",
    )
    parser.add_argument(
        "--calibration-backup",
        type=Path,
        default=DEFAULT_CALIBRATION_PATH,
        help=f"Path for the pre-upload calibration backup JSON. Defaults to {DEFAULT_CALIBRATION_PATH}.",
    )
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    firmware_dir = script_dir / "firmware"
    firmware_path_m7 = firmware_dir / M7_FIRMWARE_NAME
    firmware_path_m4 = firmware_dir / M4_FIRMWARE_NAME

    for firmware_path in (firmware_path_m7, firmware_path_m4):
        if not firmware_path.exists():
            raise SystemExit(f"Firmware file not found: {firmware_path}")

    dfu_util = find_dfu_util()
    if dfu_util is None:
        raise SystemExit("dfu-util not found.")

    port = choose_giga_port(args.port, prompt="Select Arduino GIGA to upload to")
    already_in_dfu = port is None and dfu_device_present(dfu_util)
    if port is None and not already_in_dfu:
        log("No serial port or DFU device found. Double-tap reset now.")
        if not wait_for_dfu_device(dfu_util, DFU_MANUAL_WAIT_S):
            raise SystemExit("Arduino GIGA not found as serial or USB DFU.")

    serial_number = resolve_serial_number(port, args.serial_number)
    calibration_state = backup_calibration_if_available(port)
    if calibration_state:
        backup_path = get_available_path(args.calibration_backup)
        write_calibration_state_file(backup_path, calibration_state)
        log(f"Calibration backup saved to {backup_path}.")

    with tempfile.TemporaryDirectory(prefix="gatekeeper_firmware_") as temp_dir:
        patched_m7 = prepare_patched_firmware(firmware_path_m7, serial_number, temp_dir)
        patched_m4 = prepare_patched_firmware(firmware_path_m4, serial_number, temp_dir)

        if port is not None:
            trigger_dfu_mode(dfu_util, port)
        elif not dfu_device_present(dfu_util):
            raise RuntimeError("Arduino GIGA USB DFU device disappeared before upload.")
        else:
            log("Using existing Arduino GIGA USB DFU device.")

        upload_bundle(dfu_util, patched_m4, patched_m7)

    port = wait_for_giga_port(serial_number)
    if calibration_state:
        log("Restoring calibration...")
        restore_calibration(port, calibration_state)
        log("Verifying calibration...")
        verify_state = load_calibration_state(backup_path)
        verify_calibration(port, verify_state)

    nop_test(port, serial_number)


if __name__ == "__main__":
    main()
