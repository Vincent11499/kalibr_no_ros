"""Docker runtime deployment asset checks."""

from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "kalibr_docker.sh"
DOCKERFILE = ROOT / "tools" / "docker" / "Dockerfile.runtime"


class KalibrDockerAssetTest(unittest.TestCase):
    def test_runner_has_valid_shell_syntax_and_help_without_docker(self):
        subprocess.run(["bash", "-n", str(SCRIPT)], check=True)
        completed = subprocess.run(
            ["bash", str(SCRIPT), "--help"],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
        )
        self.assertIn("build [BASE_IMAGE]", completed.stdout)
        self.assertIn("machine INSTALL_DIR", completed.stdout)

    def test_runtime_image_keeps_program_and_data_external(self):
        content = DOCKERFILE.read_text(encoding="utf-8")
        self.assertNotIn("COPY ", content)
        self.assertNotIn("ADD ", content)
        for dependency in (
                "python3-numpy", "python3-opencv", "python3-scipy",
                "libboost-python1.74.0", "libtbb2"):
            self.assertIn(dependency, content)
        self.assertIn("ARG UBUNTU_MIRROR=", content)


if __name__ == "__main__":
    unittest.main()
