import unittest

try:
    import igraph
    from kalibr_camera_calibration.MulticamGraph import MulticamCalibrationGraph
except ImportError:
    igraph = None
    MulticamCalibrationGraph = None


@unittest.skipIf(
    MulticamCalibrationGraph is None,
    "the native build-tree Python packages are not on PYTHONPATH",
)
class IgraphPlotCompatibilityTest(unittest.TestCase):
    def test_labeled_edge_renders_with_modern_igraph(self):
        calibration_graph = MulticamCalibrationGraph.__new__(
            MulticamCalibrationGraph
        )
        calibration_graph.G = igraph.Graph(2, [(0, 1)])
        calibration_graph.G.es["weight"] = [17]
        calibration_graph.optimal_baseline_edges = [0]

        output = calibration_graph.plotGraph(noShow=True)

        self.assertEqual(output, "/tmp/graph.png")
        self.assertEqual(calibration_graph.G.es["curved"], [False])


if __name__ == "__main__":
    unittest.main(verbosity=2)
