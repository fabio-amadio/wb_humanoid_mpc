from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
import pytest


LAUNCH_DIR = Path(get_package_share_directory("g1_centroidal_mpc")) / "launch"


@pytest.mark.parametrize(
    "launch_file", sorted(LAUNCH_DIR.glob("*.launch.py")), ids=lambda path: path.name
)
def test_launch_description_loads(launch_file):
    result = subprocess.run(
        ["ros2", "launch", "g1_centroidal_mpc", launch_file.name, "--show-args"],
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
