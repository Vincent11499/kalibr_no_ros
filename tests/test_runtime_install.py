"""Minimal runtime publisher checks."""

import importlib.util
from pathlib import Path
import stat
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "install_runtime", ROOT / "tools" / "install_runtime.py")
install_runtime = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(install_runtime)


def configuration_snapshot(root):
    return {
        path.relative_to(root).as_posix(): (
            "directory" if path.is_dir() else path.read_bytes(),
            stat.S_IMODE(path.stat().st_mode),
        )
        for path in root.rglob("*")
    }


class RuntimeInstallTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.source = self.root / "source"
        for name in ("bin", "lib", "libexec", "licenses"):
            path = self.source / name
            path.mkdir(parents=True)
            (path / "sentinel").write_text(name, encoding="utf-8")
        machine = self.source / "config" / "mechine"
        machine.mkdir(parents=True)
        (machine / "built_in.yaml").write_text("value: new\n", encoding="utf-8")
        managed = machine / "workflow_v1.0.0"
        managed.mkdir()
        (managed / "new_task.yaml").write_text("value: new\n", encoding="utf-8")
        (self.source / "config" / "user.yaml").write_text(
            "machine_dir: default\n", encoding="utf-8")
        (self.source / "LICENSE").write_text("license\n", encoding="utf-8")
        (self.source / "include").mkdir()
        (self.source / "share").mkdir()

        self.existing = self.root / "existing"
        existing_machine = self.existing / "config" / "mechine"
        existing_machine.mkdir(parents=True)
        (existing_machine / "built_in.yaml").write_bytes(b"value: old\r\n")
        (existing_machine / "现场标定板.yaml").write_text(
            "target_type: aprilgrid\n", encoding="utf-8")
        legacy = existing_machine / "workflow_v1.0.0"
        legacy.mkdir()
        (legacy / "legacy_task.yaml").write_text("value: old\n", encoding="utf-8")
        (legacy / "new_task.yaml").write_text("value: onsite\n", encoding="utf-8")
        (self.existing / "config" / "user.yaml").write_text(
            "machine_dir: batch\n", encoding="utf-8")
        note = self.existing / "config" / "现场说明.bin"
        note.write_bytes(b"\x00\xff\r\n")
        note.chmod(0o640)
        (self.existing / "config" / "empty").mkdir()
        self.destination = self.root / "destination"
        self.destination.mkdir()

    def test_replacement_preserves_complete_configuration_without_new_defaults(self):
        expected = configuration_snapshot(self.existing / "config")
        install_runtime.assemble_runtime(
            self.source, self.destination, existing=self.existing)

        self.assertFalse((self.destination / "include").exists())
        self.assertFalse((self.destination / "share").exists())
        self.assertEqual(
            configuration_snapshot(self.destination / "config"), expected)
        self.assertEqual(
            (self.destination / "bin" / "sentinel").read_text(), "bin")
        self.assertEqual(configuration_snapshot(self.existing / "config"), expected)

    def test_first_install_uses_complete_built_configuration(self):
        expected = configuration_snapshot(self.source / "config")
        install_runtime.assemble_runtime(self.source, self.destination)
        self.assertEqual(
            configuration_snapshot(self.destination / "config"), expected)

    def test_explicit_user_config_changes_only_requested_file(self):
        expected = configuration_snapshot(self.existing / "config")
        user = self.root / "explicit.yaml"
        user.write_text("machine_dir: explicitly_selected\n", encoding="utf-8")
        install_runtime.assemble_runtime(
            self.source, self.destination, existing=self.existing,
            user_config=user)
        actual = configuration_snapshot(self.destination / "config")
        self.assertEqual(actual.pop("user.yaml")[0], user.read_bytes())
        expected.pop("user.yaml")
        self.assertEqual(actual, expected)

    def test_existing_install_requires_its_configuration(self):
        missing = self.root / "missing_config"
        missing.mkdir()
        with self.assertRaisesRegex(RuntimeError, "directory is missing"):
            install_runtime.assemble_runtime(
                self.source, self.destination, existing=missing)

    def test_configuration_links_are_rejected_without_following_targets(self):
        external = self.root / "external"
        external.mkdir()
        target = external / "secret.txt"
        target.write_bytes(b"private sentinel")
        for name, link_target, directory in (
                ("file-link", target, False),
                ("directory-link", external, True),
                ("broken-link", external / "absent", False)):
            with self.subTest(name=name):
                link = self.existing / "config" / name
                link.symlink_to(link_target, target_is_directory=directory)
                destination = self.root / ("destination-" + name)
                destination.mkdir()
                with self.assertRaisesRegex(RuntimeError, "symlink"):
                    install_runtime.assemble_runtime(
                        self.source, destination, existing=self.existing)
                self.assertEqual(target.read_bytes(), b"private sentinel")
                self.assertTrue((destination / "config" / name).is_symlink())
                link.unlink()

    def test_configuration_root_symlink_is_rejected(self):
        existing = self.root / "linked_config"
        existing.mkdir()
        (existing / "config").symlink_to(
            self.existing / "config", target_is_directory=True)
        with self.assertRaisesRegex(RuntimeError, "directory is missing"):
            install_runtime.assemble_runtime(
                self.source, self.destination, existing=existing)

    def test_unsafe_prefixes_are_rejected(self):
        for prefix in (Path("/"), Path.home()):
            with self.subTest(prefix=prefix):
                with self.assertRaisesRegex(RuntimeError, "unsafe runtime prefix"):
                    install_runtime._safe_prefix(prefix)
        link = self.root / "prefix-link"
        link.symlink_to(self.existing, target_is_directory=True)
        with self.assertRaisesRegex(RuntimeError, "symlink"):
            install_runtime._safe_prefix(link)


if __name__ == "__main__":
    unittest.main()
