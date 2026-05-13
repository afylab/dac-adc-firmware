import shlex
import subprocess
from pathlib import Path
from SCons.Script import COMMAND_LINE_TARGETS

Import("env")


def _bundle_command(build_only=False, clean_only=False):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    script_path = project_dir / "platformio_tools" / "usb_bundle_upload.py"
    hardware = env.GetProjectOption("custom_gatekeeper_hardware")
    command = [
        env.subst("$PYTHONEXE"),
        str(script_path),
        "--hardware",
        hardware,
    ]
    if build_only:
        command.append("--build-only")
    if clean_only:
        command.append("--clean-only")

    upload_port = env.subst("$UPLOAD_PORT")
    if upload_port and upload_port != "$UPLOAD_PORT":
        command.extend(["--port", upload_port])

    return command


def _run_bundle_build(source=None, target=None, **_kwargs):
    del source, target
    project_dir = Path(env.subst("$PROJECT_DIR"))
    subprocess.run(_bundle_command(build_only=True), cwd=str(project_dir), check=True)


def _run_bundle_clean(source=None, target=None, **_kwargs):
    del source, target
    project_dir = Path(env.subst("$PROJECT_DIR"))
    subprocess.run(_bundle_command(clean_only=True), cwd=str(project_dir), check=True)


def _run_bundle_upload(source=None, target=None, **_kwargs):
    del source, target
    project_dir = Path(env.subst("$PROJECT_DIR"))
    subprocess.run(_bundle_command(), cwd=str(project_dir), check=True)


env.Replace(UPLOADCMD=" ".join(shlex.quote(part) for part in _bundle_command()))

if "upload" in COMMAND_LINE_TARGETS:
    _run_bundle_upload()
    env.Exit(0)
elif env.IsCleanTarget():
    _run_bundle_clean()
elif not COMMAND_LINE_TARGETS:
    _run_bundle_build()

env.AddCustomTarget(
    "build_firmware_bundle",
    dependencies=None,
    actions=[_run_bundle_build],
    title="Build GateKeeper firmware bundle",
    description="Build USB gateway M4 and selected worker M7 firmware.",
)

env.AddCustomTarget(
    "clean_firmware_bundle",
    dependencies=None,
    actions=[_run_bundle_clean],
    title="Clean GateKeeper firmware bundle",
    description="Clean USB gateway M4 and selected worker M7 firmware builds.",
)
