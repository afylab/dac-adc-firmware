Import("env")

from pathlib import Path

helper_path = Path(env.subst("$PROJECT_DIR")).parent / "tools" / "platformio_upload_persistence.py"
helper_globals = {}
exec(compile(helper_path.read_text(), str(helper_path), "exec"), helper_globals)
run_post_upload = helper_globals["run_post_upload"]


def _after_upload(source, target, env):
    run_post_upload(env)


env.AddPostAction("upload", _after_upload)
