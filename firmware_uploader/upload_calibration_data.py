import argparse
from pathlib import Path

from gatekeeper_upload import (
    DEFAULT_CALIBRATION_PATH,
    choose_giga_port,
    get_dac_channel_count_for_environment,
    load_calibration_state,
    open_command_port,
    restore_calibration,
    send_command,
    verify_calibration,
    wait_for_ready,
)


def log(message):
    print(f"[upload-calibration] {message}")


def state_for_connected_device(port, state, force=False):
    with open_command_port(port) as ser:
        wait_for_ready(ser)
        actual_environment = send_command(ser, "GET_ENVIRONMENT")
        actual_serial = send_command(ser, "SERIAL_NUMBER")

    expected_dac_channels = get_dac_channel_count_for_environment(actual_environment)
    if len(state["dac_offsets"]) != expected_dac_channels:
        raise RuntimeError(
            "Calibration file DAC channel count does not match the connected device "
            f"environment ({actual_environment})."
        )

    if not force:
        if actual_environment != state["source_environment"]:
            raise RuntimeError(
                "Connected device environment does not match calibration file: "
                f"expected {state['source_environment']}, got {actual_environment}. "
                "Use --force to override."
            )
        if actual_serial != state["serial_number"]:
            raise RuntimeError(
                "Connected device serial number does not match calibration file: "
                f"expected {state['serial_number']}, got {actual_serial}. "
                "Use --force to override."
            )
        return state

    forced_state = dict(state)
    forced_state["source_environment"] = actual_environment
    forced_state["serial_number"] = actual_serial
    return forced_state


def main():
    parser = argparse.ArgumentParser(
        description="Upload GateKeeper calibration data to a connected Arduino GIGA."
    )
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        default=DEFAULT_CALIBRATION_PATH,
        help=f"Input JSON path. Defaults to {DEFAULT_CALIBRATION_PATH}.",
    )
    parser.add_argument(
        "--port",
        help="Serial port to use. If omitted, detected Arduino GIGA ports are listed for selection.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Allow uploading to a different serial number or environment.",
    )
    args = parser.parse_args()

    if not args.input.exists():
        raise SystemExit(f"Calibration file not found: {args.input}")

    port = choose_giga_port(args.port, prompt="Select Arduino GIGA to upload to")
    if port is None:
        raise SystemExit("Arduino GIGA not found. Make sure it is connected.")

    state = load_calibration_state(args.input)
    verify_state = state_for_connected_device(port, state, force=args.force)
    log(
        "Writing calibration "
        f"from serial={state['serial_number']} env={state['source_environment']}."
    )
    restore_calibration(port, state)
    log("Verifying uploaded calibration...")
    verify_calibration(port, verify_state)
    log("Calibration upload complete.")


if __name__ == "__main__":
    main()

