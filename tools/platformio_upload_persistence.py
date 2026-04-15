import json
import re
import time
from pathlib import Path

import serial
import serial.tools.list_ports


SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 2
READY_RETRY_COUNT = 60
READY_RETRY_DELAY_S = 0.5
CALIBRATION_WRITE_DELAY_S = 0.05
FLOAT_VERIFY_ABS_TOL = 1e-6
ADC_CHANNEL_COUNT = 8
SERIAL_PATTERN = re.compile(r"DA_2025_.{3}$")
SERIAL_MARKER = b"__SERIAL_NUMBER__"
SERIAL_FIELD_LENGTH = 12
ABC_SERIAL_NUMBER = "DA_2025_ABC"

M4_ENVIRONMENT_MAP = {
    "giga_r1_m4_old_hardware": "OLD_HARDWARE",
    "giga_r1_m4_new_hardware": "NEW_HARDWARE",
    "giga_r1_m4_new_shield_old_dac_adc": "NEW_SHIELD_OLD_DAC_ADC",
}


def log(message):
    print(f"[upload-persistence] {message}")


def get_state_path(env):
    return Path(env.subst("$BUILD_DIR")) / "upload_state.json"


def get_binary_path(env):
    return Path(env.subst("$BUILD_DIR")) / f"{env.subst('${PROGNAME}')}.bin"


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
        return 16
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


def read_dac_calibration(ser, dac_channel_count):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(b"INQUIRY_OSG\r\n")
    ser.flush()
    values = read_float_stream(ser, dac_channel_count * 2)
    return values[:dac_channel_count], values[dac_channel_count:]


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
        if is_placeholder_serial(serial_number):
            return {
                "skip": True,
                "skip_reason": "placeholder_serial",
            }

        if not SERIAL_PATTERN.fullmatch(serial_number or ""):
            raise RuntimeError(
                "Could not determine a valid serial number from the current firmware."
            )

        dac_offsets, dac_gains = read_dac_calibration(
            ser, get_dac_channel_count_for_environment(source_environment)
        )
        adc_zero_scale, adc_full_scale = read_adc_calibration(ser)

    return {
        "source_environment": source_environment,
        "serial_number": serial_number,
        "dac_offsets": dac_offsets,
        "dac_gains": dac_gains,
        "adc_zero_scale": adc_zero_scale,
        "adc_full_scale": adc_full_scale,
    }


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


def save_state(env, state):
    state_path = get_state_path(env)
    state_path.parent.mkdir(parents=True, exist_ok=True)
    state_path.write_text(json.dumps(state, indent=2))


def load_state(env):
    state_path = get_state_path(env)
    if not state_path.exists():
        raise RuntimeError(f"Upload state file not found: {state_path}")
    return json.loads(state_path.read_text())


def is_placeholder_serial(serial_number):
    return serial_number == ABC_SERIAL_NUMBER


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

    state = backup_device_state(port)
    state["source_port"] = port
    if state.get("skip"):
        log(
            "Skipping calibration/serial persistence: "
            f"{state.get('skip_reason')}"
        )
        save_state(env, state)
        return
    
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
    restore_calibration(port, state)
    verify_calibration(port, state)
    delete_state(env)
    log("Calibration and serial verification passed after upload.")
