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

def _get_git_tag():
    try:
        # Try exact tag first
        tag = subprocess.check_output([
            "git", "describe", "--tags", "--exact-match"
        ], stderr=subprocess.STDOUT).decode("utf-8").strip()
        if tag:
            return tag
    except Exception:
        pass
    try:
        # Fallback to nearest tag if available
        tag = subprocess.check_output([
            "git", "describe", "--tags", "--abbrev=0"
        ], stderr=subprocess.STDOUT).decode("utf-8").strip()
        if tag:
            # Ensure tag actually points to HEAD; otherwise we don't use it
            head_tag = subprocess.check_output([
                "git", "tag", "--points-at", "HEAD"
            ]).decode("utf-8").strip()
            if head_tag:
                # Choose the first tag pointing at HEAD
                return head_tag.splitlines()[0]
    except Exception:
        pass
    return None

_firmware_version = _get_git_commit_hash()
_tag = _get_git_tag()

# Always define the raw hash token; code stringizes it if no tag string is present
env.Append(CPPDEFINES=[("__FIRMWARE_VERSION__", _firmware_version)])

# If we have a tag on HEAD, provide a fully quoted string macro for direct use
if _tag:
    env.Append(CPPDEFINES=[("__FIRMWARE_VERSION_STRING__", '"%s"' % _tag)])
