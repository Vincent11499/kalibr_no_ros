"""Minimal runtime publisher checks."""

import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "install_runtime", ROOT / "tools" / "install_runtime.py")
install_runtime = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(install_runtime)


class RuntimeInstallTest(unittest.TestCase):
    def test_assembly_excludes_development_tree_and_preserves_local_yaml(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source"
            for name in ("bin", "lib", "libexec", "licenses"):
                path = source / name
                path.mkdir(parents=True)
                (path / "sentinel").write_text(name, encoding="utf-8")
            machine = source / "config" / "mechine"
            machine.mkdir(parents=True)
            (machine / "built_in.yaml").write_text("value: new\n", encoding="utf-8")
            managed = machine / "workflow_v1.0.0"
            managed.mkdir()
            (managed / "new_task.yaml").write_text(
                "value: new\n", encoding="utf-8")
            (source / "LICENSE").write_text("license\n", encoding="utf-8")
            (source / "include").mkdir()
            (source / "share").mkdir()

            existing = root / "existing"
            existing_machine = existing / "config" / "mechine"
            existing_machine.mkdir(parents=True)
            (existing_machine / "built_in.yaml").write_text(
                "value: old\n", encoding="utf-8")
            (existing_machine / "local_target.yaml").write_text(
                "target_type: aprilgrid\n", encoding="utf-8")
            legacy = existing_machine / "workflow_v1.0.0"
            legacy.mkdir()
            (legacy / "legacy_task.yaml").write_text(
                "value: old\n", encoding="utf-8")
            (existing / "config" / "user.yaml").write_text(
                "machine_dir: batch\n", encoding="utf-8")

            destination = root / "destination"
            destination.mkdir()
            install_runtime.assemble_runtime(
                source, destination, existing=existing)

            self.assertFalse((destination / "include").exists())
            self.assertFalse((destination / "share").exists())
            self.assertEqual(
                (destination / "config" / "mechine" / "built_in.yaml")
                .read_text(encoding="utf-8"), "value: new\n")
            self.assertTrue(
                (destination / "config" / "mechine" / "local_target.yaml")
                .is_file())
            self.assertFalse(
                (destination / "config" / "mechine" / "workflow_v1.0.0"
                 / "legacy_task.yaml").exists())
            self.assertTrue((destination / "config" / "user.yaml").is_file())


if __name__ == "__main__":
    unittest.main()
