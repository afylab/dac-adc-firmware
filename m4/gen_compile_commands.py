import os
Import("env")

# include toolchain paths
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)

# override compilation DB path
env.Replace(COMPILATIONDB_PATH="compile_commands.json")

# Inject latest Git commit hash as firmware version define
import subprocess

def _get_git_commit_hash():
    try:
        commit_hash = subprocess.check_output([
            "git", "rev-parse", "--short", "HEAD"
        ]).decode("utf-8").strip()
        if not commit_hash:
            return "UNKNOWN"
        return commit_hash
    except Exception:
        return "UNKNOWN"

_firmware_version = _get_git_commit_hash()

# Define macro as raw token; we'll stringize it in code
env.Append(CPPDEFINES=[("__FIRMWARE_VERSION__", _firmware_version)])
