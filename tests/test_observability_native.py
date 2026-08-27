import ctypes
import ctypes.util
import unittest

import numpy as np


# libbsplines contains inline sparse-block helpers from the frozen source that
# do not record CHOLMOD as a direct ELF dependency.  The production launcher
# loads the optimizer stack first; make that transitive dependency explicit in
# this isolated import test as well.
_cholmod = ctypes.util.find_library("cholmod")
if _cholmod:
    ctypes.CDLL(_cholmod, mode=ctypes.RTLD_GLOBAL)

try:
    import aslam_backend as backend
    import incremental_calibration as incremental
    import aslam_splines
    import bsplines
    import sm
except ImportError:
    backend = None
    aslam_splines = None
    bsplines = None
    incremental = None
    sm = None


@unittest.skipIf(backend is None, "native calibration modules are unavailable")
class NativeObservabilityAnalysisTest(unittest.TestCase):
    def _transformation_spectrum(self, weight_rotation, weight_translation):
        problem = backend.OptimizationProblem()
        transforms = []
        for index in range(2):
            transform = backend.TransformationDv(sm.Transformation())
            transforms.append(transform)
            problem.addDesignVariable(transform.q)
            problem.addDesignVariable(transform.t)
            rotation_weight, translation_weight = (
                (1.0, 1.0) if index == 0 else
                (weight_rotation, weight_translation)
            )
            problem.addErrorTerm(backend.ErrorTermTransformation(
                transform.toExpression(),
                sm.Transformation(),
                rotation_weight,
                translation_weight,
            ))

        result = incremental.analyzeObservability(
            problem, [transforms[1].q, transforms[1].t], 1)
        return np.asarray(result["singular_values"])

    def test_analysis_uses_native_l2_column_scaling(self):
        reference = self._transformation_spectrum(1.0, 1.0)
        rotation_small = self._transformation_spectrum(1.0e-8, 1.0e8)
        translation_small = self._transformation_spectrum(1.0e8, 1.0e-8)

        # Native LinearSolver::solve() normalizes every Jacobian column before
        # forming the reduced spectrum.  Calling analyzeMarginal() directly
        # would bypass that step and make these spectra differ by many orders
        # of magnitude, so this test protects the diagnostic's scale policy.
        np.testing.assert_allclose(rotation_small, reference, rtol=1.0e-12,
                                   atol=1.0e-12)
        np.testing.assert_allclose(translation_small, reference, rtol=1.0e-12,
                                   atol=1.0e-12)

    def test_analysis_is_read_only_and_detects_known_null_direction(self):
        problem = backend.OptimizationProblem()
        transforms = []
        for _ in range(2):
            transform = backend.TransformationDv(sm.Transformation())
            transforms.append(transform)
            problem.addDesignVariable(transform.q)
            problem.addDesignVariable(transform.t)
            problem.addErrorTerm(backend.ErrorTermTransformation(
                transform.toExpression(), sm.Transformation(), 1.0, 1.0))

        before_values = [transform.T().copy() for transform in transforms]
        before_order = [
            problem.designVariable(index).blockIndex()
            for index in range(problem.numDesignVariables())
        ]
        result = incremental.analyzeObservability(
            problem, [transforms[1].q, transforms[1].t], 2)

        self.assertEqual(result["calibration_dimensions"], 6)
        self.assertEqual(result["rank"], 5)
        self.assertEqual(result["deficiency"], 1)
        self.assertEqual(result["operational_rank"], 5)
        self.assertEqual(result["operational_deficiency"], 1)
        self.assertEqual(np.asarray(result["nullspace"]).shape, (6, 1))
        for transform, expected in zip(transforms, before_values):
            np.testing.assert_array_equal(transform.T(), expected)
        self.assertEqual([
            problem.designVariable(index).blockIndex()
            for index in range(problem.numDesignVariables())
        ], before_order)

    def test_quadratic_spline_prior_uses_jacobian_equivalent_terms(self):
        problem = backend.OptimizationProblem()
        transform = backend.TransformationDv(sm.Transformation())
        problem.addDesignVariable(transform.q)
        problem.addDesignVariable(transform.t)
        problem.addErrorTerm(backend.ErrorTermTransformation(
            transform.toExpression(), sm.Transformation(), 1.0, 1.0))

        spline = bsplines.BSpline(4)
        spline.initConstantSpline(0.0, 1.0, 6, np.zeros(3))
        spline_dv = aslam_splines.EuclideanBSplineDesignVariable(spline)
        for index in range(spline_dv.numDesignVariables()):
            design_variable = spline_dv.designVariable(index)
            design_variable.setActive(True)
            problem.addDesignVariable(design_variable)
        problem.addErrorTerm(aslam_splines.BSplineEuclideanMotionError(
            spline_dv, np.eye(3), 0))

        result = incremental.analyzeObservability(
            problem, [transform.q, transform.t], 2)

        self.assertEqual(result["rank"], 6)
        self.assertEqual(result["deficiency"], 0)
        self.assertEqual(result["operational_rank"], 6)
        self.assertEqual(result["operational_deficiency"], 0)
        self.assertEqual(result["expanded_quadratic_error_terms"], 1)
        self.assertGreater(
            result["linearized_error_terms"],
            result["original_error_terms"],
        )


if __name__ == "__main__":
    unittest.main()
