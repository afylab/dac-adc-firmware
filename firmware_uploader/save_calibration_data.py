import argparse
from pathlib import Path

from gatekeeper_upload import (
    DEFAULT_CALIBRATION_PATH,
    backup_device_state,
    choose_giga_port,
    get_available_path,
    write_calibration_state_file,
)


DEFAULT_OUTPUT_PATH = DEFAULT_CALIBRATION_PATH


def log(message):
    print(f"[save-calibration] {message}")


def get_available_calibration_path(path):
    return get_available_path(path)


def read_calibration_state(port):
    state = backup_device_state(port)
    if state.get("skip"):
        raise RuntimeError(
            "Current firmware did not provide calibration data: "
            f"{state.get('skip_reason')}"
        )
    return state


def save_calibration(port, output_path):
    log("Reading calibration values...")
    state = read_calibration_state(port)
    output_path = get_available_calibration_path(output_path)
    write_calibration_state_file(output_path, state)
    log(f"Calibration saved to {output_path}.")
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description="Save GateKeeper calibration data from a connected Arduino GIGA."
    )
    parser.add_argument(
        "output",
        nargs="?",
        type=Path,
        default=DEFAULT_OUTPUT_PATH,
        help=f"Output JSON path. Defaults to {DEFAULT_OUTPUT_PATH}.",
    )
    parser.add_argument(
        "--port",
        help="Serial port to use. If omitted, detected Arduino GIGA ports are listed for selection.",
    )
    args = parser.parse_args()

    port = choose_giga_port(args.port, prompt="Select Arduino GIGA to read from")
    if port is None:
        raise SystemExit("Arduino GIGA not found. Make sure it is connected.")

    save_calibration(port, args.output)


if __name__ == "__main__":
    main()

