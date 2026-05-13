#!/usr/bin/env python3
import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
M4_PROJECT_DIR = ROOT_DIR / "m4"
M7_PROJECT_DIR = ROOT_DIR / "m7"
M4_ENV = "gatekeeper_m4_usb_gateway"

HARDWARE_M7_ENV = {
    "new_hardware": "gatekeeper_new_hardware",
    "old_hardware": "gatekeeper_old_hardware",
    "new_shield_old_dac_adc": "gatekeeper_new_shield_old_dac_adc",
}


def log(message):
    print(f"[intellisense] {message}", flush=True)


def find_platformio():
    for name in ("pio", "platformio"):
        path = shutil.which(name)
        if path:
            return path
    raise RuntimeError("PlatformIO executable not found.")


def run_compiledb(platformio, project_dir, env_name):
    log(f"Refreshing {project_dir.name} compile database ({env_name})...")
    subprocess.run(
        [platformio, "run", "-e", env_name, "-t", "compiledb"],
        cwd=project_dir,
        check=True,
    )


def normalized_entries(compile_db_path):
    data = json.loads(compile_db_path.read_text())
    entries = []
    for entry in data:
        normalized = dict(entry)
        directory = Path(normalized["directory"])
        file_path = Path(normalized["file"])
        if not file_path.is_absolute():
            normalized["file"] = str((directory / file_path).resolve())
        if "output" in normalized:
            output_path = Path(normalized["output"])
            if not output_path.is_absolute():
                normalized["output"] = str((directory / output_path).resolve())
        entries.append(normalized)
    return entries


def merge_compile_databases(hardware):
    compile_databases = [
        M4_PROJECT_DIR / "compile_commands.json",
        M7_PROJECT_DIR / "compile_commands.json",
    ]
    missing = [path for path in compile_databases if not path.exists()]
    if missing:
        missing_list = ", ".join(str(path) for path in missing)
        raise RuntimeError(f"Missing compile database: {missing_list}")

    entries = []
    for path in compile_databases:
        entries.extend(normalized_entries(path))

    output_path = ROOT_DIR / "compile_commands.json"
    output_path.write_text(json.dumps(entries, indent=2) + "\n")
    log(
        "Wrote combined compile database "
        f"{output_path} ({len(entries)} entries, hardware={hardware})."
    )


def write_vscode_cpp_config():
    vscode_dir = ROOT_DIR / ".vscode"
    vscode_dir.mkdir(exist_ok=True)
    config_path = vscode_dir / "c_cpp_properties.json"
    config = {
        "configurations": [
            {
                "name": "GateKeeper Firmware",
                "compileCommands": "${workspaceFolder}/compile_commands.json",
                "compilerPath": (
                    "${userHome}/.platformio/packages/"
                    "toolchain-gccarmnoneeabi/bin/arm-none-eabi-g++"
                ),
                "intelliSenseMode": "gcc-arm",
                "cStandard": "gnu11",
                "cppStandard": "gnu++14",
            }
        ],
        "version": 4,
    }
    config_path.write_text(json.dumps(config, indent=4) + "\n")
    log(f"Wrote VSCode C++ config {config_path}.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Build a root compile_commands.json for M4/M7 IntelliSense."
    )
    parser.add_argument(
        "--hardware",
        choices=sorted(HARDWARE_M7_ENV),
        default="new_hardware",
        help="Hardware target whose M7 defines should be used for IntelliSense.",
    )
    parser.add_argument(
        "--no-compiledb",
        action="store_true",
        help="Only merge existing M4/M7 compile databases.",
    )
    parser.add_argument(
        "--no-vscode-config",
        action="store_true",
        help="Do not write .vscode/c_cpp_properties.json.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.no_compiledb:
        platformio = find_platformio()
        run_compiledb(platformio, M4_PROJECT_DIR, M4_ENV)
        run_compiledb(platformio, M7_PROJECT_DIR, HARDWARE_M7_ENV[args.hardware])

    merge_compile_databases(args.hardware)
    if not args.no_vscode_config:
        write_vscode_cpp_config()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001
        print(f"[intellisense] ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
