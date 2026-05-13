import importlib.util
from pathlib import Path


def _load_gatekeeper_upload_module():
    try:
        repo_root = Path(__file__).resolve().parents[1]
    except NameError:
        repo_root = Path.cwd().resolve()
        if repo_root.name in ("m4", "m7"):
            repo_root = repo_root.parent

    helper_path = repo_root / "firmware_uploader" / "gatekeeper_upload.py"
    spec = importlib.util.spec_from_file_location("gatekeeper_upload", helper_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_gatekeeper_upload = _load_gatekeeper_upload_module()

for _name in dir(_gatekeeper_upload):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_gatekeeper_upload, _name)
