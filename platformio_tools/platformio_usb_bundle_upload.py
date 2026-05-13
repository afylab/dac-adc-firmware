import subprocess
import shlex
from pathlib import Path

Import("env")


def _usb_bundle_command(env):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    script_path = project_dir.parent / "platformio_tools" / "usb_bundle_upload.py"
    command = [
        env.subst("$PYTHONEXE"),
        str(script_path),
    ]
    pioenv = env.subst("$PIOENV")

    if project_dir.name == "m4":
        command.extend(["--m4-env", pioenv, "--skip-m4-build"])
    elif project_dir.name == "m7":
        command.extend(["--m7-env", pioenv, "--skip-real-m7-build"])

    upload_port = env.subst("$UPLOAD_PORT")
    if upload_port and upload_port != "$UPLOAD_PORT":
        command.extend(["--port", upload_port])

    return command


def _usb_bundle_upload(source, target, env):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    command = _usb_bundle_command(env)
    subprocess.run(command, cwd=str(project_dir.parent), check=True)


if env.subst("$UPLOAD_PROTOCOL") == "custom":
    env.Replace(GATEKEEPER_USB_BUNDLE_UPLOAD="1")
    env.Replace(
        UPLOADCMD=" ".join(shlex.quote(part) for part in _usb_bundle_command(env))
    )


env.AddCustomTarget(
    "usb_upload",
    dependencies=[env.subst("$BUILD_DIR/${PROGNAME}.bin")],
    actions=[_usb_bundle_upload],
    title="GateKeeper firmware bundle upload",
    description="Upload USB gateway M4 and selected worker M7 through USB DFU.",
)
