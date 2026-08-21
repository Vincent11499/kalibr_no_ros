import tempfile
import unittest
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from compare_kalibr_outputs import compare_file


class CompareOutputsTest(unittest.TestCase):
    def test_yaml_allows_serialization_roundoff(self):
        with tempfile.TemporaryDirectory() as directory:
            reference = Path(directory) / "reference.yaml"
            candidate = Path(directory) / "candidate.yaml"
            reference.write_text("value: [1.0, 2.0]\n", encoding="utf-8")
            candidate.write_text("value: [1.0000000000001, 2.0]\n", encoding="utf-8")
            matched, detail = compare_file(reference, candidate)
            self.assertTrue(matched, detail)

    def test_text_requires_exact_match(self):
        with tempfile.TemporaryDirectory() as directory:
            reference = Path(directory) / "reference.txt"
            candidate = Path(directory) / "candidate.txt"
            reference.write_text("result\n", encoding="utf-8")
            candidate.write_text("different\n", encoding="utf-8")
            matched, _ = compare_file(reference, candidate)
            self.assertFalse(matched)
