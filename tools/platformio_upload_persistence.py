import json
import re
import signal
import threading
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

import serial
import serial.tools.list_ports


SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 2
READY_RETRY_COUNT = 60
READY_RETRY_DELAY_S = 0.5
CALIBRATION_WRITE_DELAY_S = 0.05
CALIBRATION_DOWNLOAD_TIMEOUT_S = 10
CALIBRATION_UPLOAD_TIMEOUT_S = 10
CALIBRATION_VERIFY_TIMEOUT_S = 10
FLOAT_VERIFY_ABS_TOL = 1e-6
ADC_CHANNEL_COUNT = 8
MAX_DAC_CALIBRATION_RESPONSE_CHANNELS = 16
SERIAL_PATTERN = re.compile(r"DA_2025_.{3}$")
SERIAL_MARKER = b"__SERIAL_NUMBER__"
SERIAL_FIELD_LENGTH = 12
NO_CALIBRATION_UPLOAD_SERIAL = "DA_2025____"
DEFAULT_CALIBRATION_BACKUP_DIR_NAME = "calibration_backups"

M4_ENVIRONMENT_MAP = {
    "giga_r1_m4_old_hardware": "OLD_HARDWARE",
    "giga_r1_m4_new_hardware": "NEW_HARDWARE",
    "giga_r1_m4_new_shield_old_dac_adc": "NEW_SHIELD_OLD_DAC_ADC",
}


def log(message):
    print(f"[upload-persistence] {message}")


class CalibrationOperationTimeout(RuntimeError):
    pass


@contextmanager
def operation_timeout(seconds, operation_name):
    if threading.current_thread() is not threading.main_thread():
        yield
        return

    def handle_timeout(_signum, _frame):
        raise CalibrationOperationTimeout(
            f"Timed out {operation_name} after {seconds} seconds."
        )

    previous_handler = signal.getsignal(signal.SIGALRM)
    signal.signal(signal.SIGALRM, handle_timeout)
    previous_timer = signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous_handler)
        if previous_timer[0] > 0:
            signal.setitimer(signal.ITIMER_REAL, previous_timer[0], previous_timer[1])


def get_state_path(env):
    return Path(env.subst("$BUILD_DIR")) / "upload_state.json"


def get_binary_path(env):
    return Path(env.subst("$BUILD_DIR")) / f"{env.subst('${PROGNAME}')}.bin"


def get_project_root(env):
    return Path(env.subst("$PROJECT_DIR")).parent


def get_calibration_backup_dir(env):
    configured = env.get("CALIBRATION_BACKUP_DIR")
    if configured:
        return Path(env.subst(configured)).expanduser()
    return get_project_root(env) / DEFAULT_CALIBRATION_BACKUP_DIR_NAME


def get_calibration_backup_path(env, state):
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    serial_number = state["serial_number"].replace("/", "_")
    backup_dir = get_calibration_backup_dir(env)
    backup_path = backup_dir / f"calibration_{serial_number}_{timestamp}.json"
    if not backup_path.exists():
        return backup_path

    index = 2
    while True:
        candidate = (
            backup_dir / f"calibration_{serial_number}_{timestamp}_{index}.json"
        )
        if not candidate.exists():
            return candidate
        index += 1


KNOWN_GIGA_VIDS_PIDS = {
    (0x2341, 0x0266),
}


def is_m4_project(env):
    return env.subst("$PIOENV") in M4_ENVIRONMENT_MAP


def parse_vid_pid_from_hwid(hwid):
    if not hwid:
        return None, None
    match = re.search(r"VID(?:[:_=]|_)?([0-9A-Fa-f]{4}).*?PID(?:[:_=]|_)?([0-9A-Fa-f]{4})", hwid)
    if not match:
        return None, None
    return int(match.group(1), 16), int(match.group(2), 16)


def is_giga_port(port):
    vid, pid = port.vid, port.pid
    if vid is None or pid is None:
        vid, pid = parse_vid_pid_from_hwid(port.hwid)
    return (vid, pid) in KNOWN_GIGA_VIDS_PIDS


def find_giga_port(env=None):
    ports = list(serial.tools.list_ports.comports())
    if env is not None:
        upload_port = env.subst("$UPLOAD_PORT")
        if upload_port and upload_port != "$UPLOAD_PORT":
            existing = next((p for p in ports if p.device == upload_port), None)
            if existing:
                if is_giga_port(existing):
                    log(f"Using configured upload port: {upload_port}")
                    return upload_port
                log(
                    f"Configured upload port {upload_port} is present but not a recognized GIGA device; scanning actual ports"
                )
            else:
                log(
                    f"Configured upload port {upload_port} not present; scanning actual ports"
                )

    for port in ports:
        if is_giga_port(port):
            log(f"Found GIGA port {port.device} ({port.hwid})")
            return port.device

    log(
        "No GIGA serial port found. "
        f"Available ports: {[p.device for p in ports]}"
    )
    return None


def open_command_port(port, timeout=SERIAL_TIMEOUT_S):
    return serial.Serial(port, SERIAL_BAUD, timeout=timeout, write_timeout=timeout)


def read_line(ser, timeout=None):
    old_timeout = ser.timeout
    if timeout is not None:
        ser.timeout = timeout
    try:
        response = ser.readline()
    finally:
        if timeout is not None:
            ser.timeout = old_timeout

    if not response:
        raise RuntimeError("Timed out waiting for a serial response.")
    return response.decode(errors="replace").strip()


def send_command(ser, command, timeout=None):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(f"{command}\r\n".encode("ascii"))
    ser.flush()
    response = read_line(ser, timeout=timeout)
    if response.startswith("FAILURE"):
        raise RuntimeError(f"Command '{command}' failed: {response}")
    return response


def wait_for_ready(ser):
    for _ in range(READY_RETRY_COUNT):
        try:
            if send_command(ser, "*RDY?") == "READY":
                return
        except RuntimeError:
            pass
        time.sleep(READY_RETRY_DELAY_S)
    raise RuntimeError("Device never reported READY.")


def wait_for_device_ready(env=None, port=None):
    last_error = None
    if port is not None:
        log(f"Waiting for same port to become ready: {port}")
        for _ in range(READY_RETRY_COUNT):
            try:
                with open_command_port(port) as ser:
                    wait_for_ready(ser)
                return port
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                time.sleep(READY_RETRY_DELAY_S)

        raise RuntimeError(f"Device never became ready after upload: {last_error}")

    for _ in range(READY_RETRY_COUNT):
        current_port = find_giga_port(env)
        if current_port is None:
            time.sleep(READY_RETRY_DELAY_S)
            continue

        try:
            with open_command_port(current_port) as ser:
                wait_for_ready(ser)
            return current_port
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            time.sleep(READY_RETRY_DELAY_S)

    if last_error is not None:
        raise RuntimeError(f"Device never became ready after upload: {last_error}")
    raise RuntimeError("Arduino GIGA not found after upload.")


def get_dac_channel_count_for_environment(environment):
    if environment == "OLD_HARDWARE":
        return 4
    if environment in ("NEW_HARDWARE", "NEW_SHIELD_OLD_DAC_ADC"):
        return 8
    raise RuntimeError(f"Unknown device environment '{environment}'.")


def read_float_stream(ser, expected_count):
    values = []
    while len(values) < expected_count:
        line = read_line(ser, timeout=SERIAL_TIMEOUT_S)
        if not line:
            continue
        for token in line.split():
            try:
                values.append(float(token))
            except ValueError as exc:
                raise RuntimeError(f"Could not parse float response '{line}'.") from exc
            if len(values) == expected_count:
                break
    return values


def read_dac_float_stream(ser, active_dac_channel_count):
    min_count = active_dac_channel_count * 2
    max_count = MAX_DAC_CALIBRATION_RESPONSE_CHANNELS * 2
    values = []
    deadline = time.monotonic() + SERIAL_TIMEOUT_S

    while len(values) < min_count:
        line = read_line(ser, timeout=SERIAL_TIMEOUT_S)
        for token in line.split():
            try:
                values.append(float(token))
            except ValueError as exc:
                raise RuntimeError(f"Could not parse float response '{line}'.") from exc
        deadline = time.monotonic() + 0.2

    while len(values) < max_count and time.monotonic() < deadline:
        try:
            line = read_line(ser, timeout=0.05)
        except RuntimeError:
            continue
        for token in line.split():
            try:
                values.append(float(token))
            except ValueError as exc:
                raise RuntimeError(f"Could not parse float response '{line}'.") from exc
        deadline = time.monotonic() + 0.2

    if len(values) % 2 != 0:
        raise RuntimeError(f"Expected offset/gain pairs, got {len(values)} values.")
    return values


def read_dac_calibration(ser, dac_channel_count):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(b"INQUIRY_OSG\r\n")
    ser.flush()
    values = read_dac_float_stream(ser, dac_channel_count)
    response_channel_count = len(values) // 2
    if response_channel_count < dac_channel_count:
        raise RuntimeError(
            f"Expected at least {dac_channel_count} DAC calibration channels, "
            f"got {response_channel_count}."
        )
    offsets = values[:response_channel_count]
    gains = values[response_channel_count:]
    return offsets[:dac_channel_count], gains[:dac_channel_count]


def read_adc_calibration_with_commands(ser, zero_cmd, full_cmd):
    adc_zero_scale = []
    adc_full_scale = []
    for channel in range(ADC_CHANNEL_COUNT):
        adc_zero_scale.append(int(send_command(ser, f"{zero_cmd},{channel}")))
    for channel in range(ADC_CHANNEL_COUNT):
        adc_full_scale.append(int(send_command(ser, f"{full_cmd},{channel}")))
    return adc_zero_scale, adc_full_scale


def read_adc_calibration(ser):
    try:
        return read_adc_calibration_with_commands(
            ser, "GET_SAVED_ZERO_SCALE_CAL", "GET_SAVED_FULL_SCALE_CAL"
        )
    except RuntimeError:
        return read_adc_calibration_with_commands(
            ser, "GET_ZERO_SCALE_CAL", "GET_FULL_SCALE_CAL"
        )


def make_calibration_state(
    source_environment,
    serial_number,
    firmware_version,
    device_id,
    dac_offsets,
    dac_gains,
    adc_zero_scale,
    adc_full_scale,
):
    return {
        "schema_version": 1,
        "saved_at": datetime.now(timezone.utc).isoformat(),
        "source_environment": source_environment,
        "serial_number": serial_number,
        "firmware_version": firmware_version,
        "device_id": device_id,
        "dac_channel_count": len(dac_offsets),
        "adc_channel_count": ADC_CHANNEL_COUNT,
        "dac_offsets": dac_offsets,
        "dac_gains": dac_gains,
        "adc_zero_scale": adc_zero_scale,
        "adc_full_scale": adc_full_scale,
    }


def validate_calibration_state(state):
    required_keys = {
        "schema_version",
        "saved_at",
        "source_environment",
        "serial_number",
        "firmware_version",
        "device_id",
        "dac_channel_count",
        "adc_channel_count",
        "dac_offsets",
        "dac_gains",
        "adc_zero_scale",
        "adc_full_scale",
    }
    missing = sorted(required_keys - set(state))
    if missing:
        raise RuntimeError(f"Calibration state is missing keys: {', '.join(missing)}")
    if state["schema_version"] != 1:
        raise RuntimeError(
            f"Unsupported calibration schema_version: {state['schema_version']}"
        )
    if state["adc_channel_count"] != ADC_CHANNEL_COUNT:
        raise RuntimeError(
            f"Unsupported ADC channel count: {state['adc_channel_count']}"
        )
    if len(state["dac_offsets"]) != state["dac_channel_count"]:
        raise RuntimeError("DAC offset count does not match dac_channel_count.")
    if len(state["dac_gains"]) != state["dac_channel_count"]:
        raise RuntimeError("DAC gain count does not match dac_channel_count.")
    if len(state["adc_zero_scale"]) != ADC_CHANNEL_COUNT:
        raise RuntimeError("ADC zero-scale count does not match adc_channel_count.")
    if len(state["adc_full_scale"]) != ADC_CHANNEL_COUNT:
        raise RuntimeError("ADC full-scale count does not match adc_channel_count.")


def comparable_calibration_state(state):
    comparable = dict(state)
    comparable.pop("saved_at", None)
    return comparable


def remove_older_duplicate_calibration_files(backup_path, state):
    backup_dir = backup_path.parent
    if not backup_dir.exists():
        return []

    comparable_state = comparable_calibration_state(state)
    removed_paths = []
    for candidate in backup_dir.glob("*.json"):
        if candidate == backup_path:
            continue
        try:
            candidate_state = json.loads(candidate.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        if comparable_calibration_state(candidate_state) != comparable_state:
            continue
        candidate.unlink()
        removed_paths.append(candidate)
    return removed_paths


def write_calibration_backup_file(env, state):
    validate_calibration_state(state)
    backup_path = get_calibration_backup_path(env, state)
    backup_path.parent.mkdir(parents=True, exist_ok=True)
    backup_path.write_text(json.dumps(state, indent=2) + "\n")
    saved_state = json.loads(backup_path.read_text())
    if saved_state != state:
        raise RuntimeError(
            f"Calibration backup file verification failed: {backup_path}"
        )

    removed_paths = remove_older_duplicate_calibration_files(backup_path, state)
    if removed_paths:
        log(
            "Removed older duplicate calibration backup(s): "
            + ", ".join(str(path) for path in removed_paths)
        )
    log(f"Calibration backup saved to {backup_path}.")
    return backup_path


def write_adc_calibration_with_commands(ser, zero_cmd, full_cmd, state):
    for channel, value in enumerate(state["adc_zero_scale"]):
        send_command(ser, f"{zero_cmd},{channel},{value}")
        time.sleep(CALIBRATION_WRITE_DELAY_S)

    for channel, value in enumerate(state["adc_full_scale"]):
        send_command(ser, f"{full_cmd},{channel},{value}")
        time.sleep(CALIBRATION_WRITE_DELAY_S)


def restore_calibration(port, state):
    with open_command_port(port) as ser:
        wait_for_ready(ser)

        for channel, (offset, gain) in enumerate(
            zip(state["dac_offsets"], state["dac_gains"])
        ):
            send_command(ser, f"SET_OSG,{channel},{offset:.8f},{gain:.8f}")
            time.sleep(CALIBRATION_WRITE_DELAY_S)

        try:
            write_adc_calibration_with_commands(
                ser,
                "SET_SAVED_ZERO_SCALE_CAL",
                "SET_SAVED_FULL_SCALE_CAL",
                state,
            )
        except RuntimeError:
            write_adc_calibration_with_commands(
                ser,
                "SET_ZERO_SCALE_CAL",
                "SET_FULL_SCALE_CAL",
                state,
            )


def verify_calibration(port, state):
    with open_command_port(port) as ser:
        wait_for_ready(ser)
        actual_offsets, actual_gains = read_dac_calibration(
            ser, len(state["dac_offsets"])
        )
        actual_zero_scale, actual_full_scale = read_adc_calibration(ser)
        actual_environment = send_command(ser, "GET_ENVIRONMENT")
        actual_serial = send_command(ser, "SERIAL_NUMBER")

    if actual_environment != state["source_environment"]:
        raise RuntimeError(
            "Environment changed across upload: "
            f"expected {state['source_environment']}, got {actual_environment}"
        )
    if actual_serial != state["serial_number"]:
        raise RuntimeError(
            "Serial number changed across upload: "
            f"expected {state['serial_number']}, got {actual_serial}"
        )

    for channel, (expected, actual) in enumerate(
        zip(state["dac_offsets"], actual_offsets)
    ):
        if abs(expected - actual) > FLOAT_VERIFY_ABS_TOL:
            raise RuntimeError(
                f"DAC {channel} offset mismatch: expected {expected:.8f}, got {actual:.8f}"
            )

    for channel, (expected, actual) in enumerate(
        zip(state["dac_gains"], actual_gains)
    ):
        if abs(expected - actual) > FLOAT_VERIFY_ABS_TOL:
            raise RuntimeError(
                f"DAC {channel} gain mismatch: expected {expected:.8f}, got {actual:.8f}"
            )

    for channel, (expected, actual) in enumerate(
        zip(state["adc_zero_scale"], actual_zero_scale)
    ):
        if expected != actual:
            raise RuntimeError(
                f"ADC {channel} zero scale mismatch: expected {expected}, got {actual}"
            )

    for channel, (expected, actual) in enumerate(
        zip(state["adc_full_scale"], actual_full_scale)
    ):
        if expected != actual:
            raise RuntimeError(
                f"ADC {channel} full scale mismatch: expected {expected}, got {actual}"
            )


def backup_device_state(port):
    with open_command_port(port) as ser:
        if firmware_hangs_on_noop(ser):
            return {
                "skip": True,
                "skip_reason": "firmware_hangs_on_nop",
            }

        wait_for_ready(ser)
        source_environment = send_command(ser, "GET_ENVIRONMENT")
        serial_number = send_command(ser, "SERIAL_NUMBER")
        firmware_version = send_command(ser, "GET_FIRMWARE_VERSION")
        device_id = send_command(ser, "*IDN?")
        if is_no_calibration_upload_serial(serial_number):
            return {
                "skip": True,
                "skip_reason": "no_calibration_upload_serial",
            }

        if not SERIAL_PATTERN.fullmatch(serial_number or ""):
            raise RuntimeError(
                "Could not determine a valid serial number from the current firmware."
            )

        dac_offsets, dac_gains = read_dac_calibration(
            ser, get_dac_channel_count_for_environment(source_environment)
        )
        adc_zero_scale, adc_full_scale = read_adc_calibration(ser)

    return make_calibration_state(
        source_environment,
        serial_number,
        firmware_version,
        device_id,
        dac_offsets,
        dac_gains,
        adc_zero_scale,
        adc_full_scale,
    )


def patch_binary_serial(binary_path, serial_number):
    if not SERIAL_PATTERN.fullmatch(serial_number or ""):
        raise RuntimeError(f"Invalid serial number '{serial_number}'.")

    data = binary_path.read_bytes()
    index = data.find(SERIAL_MARKER)
    if index == -1:
        raise RuntimeError(f"Serial marker not found in {binary_path}.")

    field_start = index
    field_end = field_start + len(SERIAL_MARKER) + SERIAL_FIELD_LENGTH
    replacement = SERIAL_MARKER + serial_number.encode("ascii")
    replacement = replacement.ljust(field_end - field_start, b"\x00")
    data = data[:field_start] + replacement + data[field_end:]
    binary_path.write_bytes(data)


def binary_contains_serial_marker(binary_path):
    return SERIAL_MARKER in binary_path.read_bytes()


def save_state(env, state):
    state_path = get_state_path(env)
    state_path.parent.mkdir(parents=True, exist_ok=True)
    state_path.write_text(json.dumps(state, indent=2))


def load_state(env):
    state_path = get_state_path(env)
    if not state_path.exists():
        raise RuntimeError(f"Upload state file not found: {state_path}")
    return json.loads(state_path.read_text())


def is_no_calibration_upload_serial(serial_number):
    return serial_number == NO_CALIBRATION_UPLOAD_SERIAL


def firmware_hangs_on_noop(ser):
    try:
        return send_command(ser, "NOP", timeout=1) != "NOP"
    except RuntimeError:
        return True


def delete_state(env):
    state_path = get_state_path(env)
    if state_path.exists():
        state_path.unlink()


def run_pre_upload(env):
    pioenv = env.subst("$PIOENV")
    port = find_giga_port(env)
    if port is None:
        raise RuntimeError("Arduino GIGA not found before upload.")

    try:
        with operation_timeout(
            CALIBRATION_DOWNLOAD_TIMEOUT_S, "downloading calibration data"
        ):
            state = backup_device_state(port)
    except CalibrationOperationTimeout as exc:
        log(f"{exc} Proceeding with firmware upload without calibration backup.")
        save_state(
            env,
            {
                "skip": True,
                "skip_reason": "calibration_download_timeout",
                "source_port": port,
            },
        )
        return
    state["source_port"] = port
    if state.get("skip"):
        log(
            "Skipping calibration/serial persistence: "
            f"{state.get('skip_reason')}"
        )
        save_state(env, state)
        return

    backup_path = write_calibration_backup_file(env, state)
    state["calibration_backup_path"] = str(backup_path)

    log(
        "Backed up device state "
        f"(env={state['source_environment']}, serial={state['serial_number']})."
    )

    if is_m4_project(env):
        expected_environment = M4_ENVIRONMENT_MAP[pioenv]
        if state["source_environment"] != expected_environment:
            raise RuntimeError(
                f"Connected board environment {state['source_environment']} "
                f"does not match selected PlatformIO environment {expected_environment}."
            )

    binary_path = get_binary_path(env)
    if binary_contains_serial_marker(binary_path):
        patch_binary_serial(binary_path, state["serial_number"])
        log(f"Patched {binary_path.name} with serial {state['serial_number']}.")

    save_state(env, state)


def run_post_upload(env):
    state = load_state(env)
    if state.get("skip"):
        log(
            "Skipping post-upload calibration/serial verification: "
            f"{state.get('skip_reason')}"
        )
        delete_state(env)
        return

    port = wait_for_device_ready(env, port=state.get("source_port"))
    log(f"Restoring calibration on {port}...")
    try:
        with operation_timeout(
            CALIBRATION_UPLOAD_TIMEOUT_S, "uploading calibration data"
        ):
            restore_calibration(port, state)
        with operation_timeout(
            CALIBRATION_VERIFY_TIMEOUT_S, "verifying calibration data"
        ):
            verify_calibration(port, state)
    except Exception:
        backup_path = state.get("calibration_backup_path")
        if backup_path:
            log(f"Calibration backup remains saved at {backup_path}.")
        raise
    delete_state(env)
    log("Calibration and serial verification passed after upload.")
