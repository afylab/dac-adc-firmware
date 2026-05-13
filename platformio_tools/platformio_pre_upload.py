Import("env")

import importlib.util
from pathlib import Path

if not env.get("GATEKEEPER_USB_BUNDLE_UPLOAD"):
    helper_path = Path(env.subst("$PROJECT_DIR")).parent / "firmware_uploader" / "gatekeeper_upload.py"
    spec = importlib.util.spec_from_file_location("gatekeeper_upload", helper_path)
    helper_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper_module)
    run_pre_upload = helper_module.run_pre_upload


    def _before_upload(source, target, env):
        run_pre_upload(env)


    env.AddPreAction("upload", _before_upload)
