# Source after /opt/ros/humble/setup.bash, before ROS state launch/test.
# Keep ROS using system Python; expose this checkout's pinned Python dependencies.
_exo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
_exo_python="$_exo_root/.local/venv/bin/python"
if [[ ! -x "$_exo_python" ]]; then
  echo 'Restore MuJoCo first: python3 tools/project/restore.py setup-python' >&2
  return 2
fi
_exo_site="$("$_exo_python" -I -B -c 'import sys, sysconfig; assert sys.version_info[:2] == (3,10); print(sysconfig.get_path("purelib"))')" || return 2
export PYTHONPATH="$_exo_site${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -B -c 'import mujoco,numpy; assert mujoco.__version__ == "3.17.0"; assert numpy.__version__ == "2.2.6"' || return 2
unset _exo_root _exo_python _exo_site
