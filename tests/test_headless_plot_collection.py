import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OVERLAY = ROOT / "core" / "overlays" / "sm" / "PlotCollection.py"


class HeadlessPlotCollectionTest(unittest.TestCase):
    def test_import_and_empty_show_do_not_require_wx(self):
        spec = importlib.util.spec_from_file_location(
            "kalibr_headless_plot_collection_test", OVERLAY
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        collection = module.PlotCollection("test")
        collection.show()
        if module.wx is None:
            collection.add_figure("dummy", object())
            with self.assertRaisesRegex(RuntimeError, "requires wxPython"):
                collection.show()


if __name__ == "__main__":
    unittest.main()
