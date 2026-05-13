#!/usr/bin/env python3
import argparse
import subprocess
from pathlib import Path

from gatekeeper_upload import (
    choose_giga_port,
    current_serial_prefix,
    open_command_port,
    patch_binary_serial,
    read_binary_serial,
    send_command,
    wait_for_giga_port,
    wait_for_ready,
)
from upload_firmware import (
    ARDUINO_GIGA_DFU_DEVICE_ID,
    find_dfu_util,
    trigger_dfu_mode,
)


DFU_ADDRESS_M7_READ = "0x08040000:"
DFU_ADDRESS_M7_WRITE = "0x08040000"
DFU_ADDRESS_M4_READ = "0x08100000:"
DFU_ADDRESS_M4_WRITE = "0x08100000:leave"
TEMP_FIRMWARE_M7 = Path("temp_firmware_m7.bin")
TEMP_FIRMWARE_M4 = Path("temp_firmware_m4.bin")


def serial_from_suffix(suffix):
    if len(suffix) > 3:
        raise RuntimeError("Serial suffix must be at most 3 characters.")
    return f"{current_serial_prefix()}_{suffix.zfill(3)}"


def dfu_read(dfu_util, output_path, address):
    print(f"Reading firmware from {address}...")
    subprocess.run(
        [
            dfu_util,
            "-d",
            ARDUINO_GIGA_DFU_DEVICE_ID,
            "-a",
            "0",
            "-s",
            address,
            "-U",
            str(output_path),
        ],
        check=True,
    )


def dfu_write(dfu_util, firmware_path, address):
    print(f"Flashing updated firmware to {address}...")
    subprocess.run(
        [
            dfu_util,
            "-d",
            ARDUINO_GIGA_DFU_DEVICE_ID,
            "-a",
            "0",
            "-s",
            address,
            "-D",
            str(firmware_path),
        ],
        check=True,
    )


def nop_test(port, expected_serial_number):
    with open_command_port(port) as ser:
        print()
        print(f"Testing NOP on {ser.port}...")
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
        print(f"Firmware successfully patched. ID={device_id}, serial={serial_number}.")


def remove_temp_files():
    for path in (TEMP_FIRMWARE_M7, TEMP_FIRMWARE_M4):
        if path.exists():
            path.unlink()


def main():
    parser = argparse.ArgumentParser(
        description="Patch the GateKeeper serial number in the firmware currently on an Arduino GIGA."
    )
    parser.add_argument(
        "serial_number",
        help="New serial suffix, up to 3 characters.",
    )
    parser.add_argument(
        "--port",
        help="Serial port to use. If omitted, detected Arduino GIGA ports are listed for selection.",
    )
    args = parser.parse_args()

    dfu_util = find_dfu_util()
    if dfu_util is None:
        raise SystemExit("dfu-util not found.")

    serial_number = serial_from_suffix(args.serial_number)
    port = choose_giga_port(args.port, prompt="Select Arduino GIGA to patch")
    if port is None:
        raise SystemExit("Arduino GIGA not found. Make sure it is connected.")

    remove_temp_files()
    trigger_dfu_mode(dfu_util, port)

    try:
        dfu_read(dfu_util, TEMP_FIRMWARE_M7, DFU_ADDRESS_M7_READ)
        dfu_read(dfu_util, TEMP_FIRMWARE_M4, DFU_ADDRESS_M4_READ)
        for path in (TEMP_FIRMWARE_M7, TEMP_FIRMWARE_M4):
            current_serial = read_binary_serial(path)
            if current_serial is None:
                raise RuntimeError(f"No serial marker found in {path}.")
            print(f"{path} currently contains serial number: {current_serial}")
            patch_binary_serial(path, serial_number)

        dfu_write(dfu_util, TEMP_FIRMWARE_M7, DFU_ADDRESS_M7_WRITE)
        dfu_write(dfu_util, TEMP_FIRMWARE_M4, DFU_ADDRESS_M4_WRITE)
    finally:
        remove_temp_files()

    print("Validating...")
    port = wait_for_giga_port(serial_number)
    nop_test(port, serial_number)


if __name__ == "__main__":
    main()
