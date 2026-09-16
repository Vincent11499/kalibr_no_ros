#encoding:UTF-8
import sm
import aslam_backend as aopt
import aslam_cv_backend as acvb
import aslam_cv as acv
import aslam_splines as asp
import incremental_calibration as inc
from kalibr_common import ConfigReader as cr
import bsplines
import numpy as np
import multiprocessing
import os
import sys
import gc
import math
from .ReprojectionErrorKnotSequenceUpdateStrategy import *
from .RsPlot import plotSpline
from .RsPlot import plotSplineValues
import pylab as pl
import pdb

try:
    import kalibr_runtime as native_runtime
except ImportError:
    native_runtime = None

# make numpy print prettier
np.set_printoptions(suppress=True)

CALIBRATION_GROUP_ID = 0
LANDMARK_GROUP_ID = 2
NATIVE_TIME_EXPRESSION_BUFFER_S = 0.5
NATIVE_TIME_PADDING_S = 0.5


def requireOptimizerSuccess(result, maxIterations, stage):
    """Reject a native Optimizer2 failure before any result is serialized."""
    if result is None:
        raise RuntimeError(
            "native rolling-shutter optimization returned no result during {0}"
            .format(stage))
    if result.linearSolverFailure:
        raise RuntimeError(
            "native rolling-shutter optimization failed during {0}"
            .format(stage))
    if int(result.iterations) >= int(maxIterations):
        raise RuntimeError(
            "native rolling-shutter optimization did not converge during {0} "
            "within max_iterations={1} (iterations={2}, dXFinal={3:.17g}, "
            "dJFinal={4:.17g})".format(
                stage, int(maxIterations), int(result.iterations),
                float(result.dXFinal), float(result.dJFinal)))
    return result

class RsCalibratorConfiguration(object):
    deltaX = 1e-8
    """Stopping criterion for the optimizer on x"""

    deltaJ = 1e-4
    """Stopping criterion for the optimizer on J"""

    maxNumberOfIterations = 20
    """Maximum number of iterations of the batch optimizer"""

    maxKnotPlacementIterations = 10
    """Maximum number of iterations to take in the adaptive knot-placement step"""

    adaptiveKnotPlacement = True
    """Whether to enable adaptive knot placement"""

    knotUpdateStrategy = ReprojectionErrorKnotSequenceUpdateStrategy
    """The adaptive knot placement strategy to use"""

    timeOffsetConstantSparsityPattern = 0.08
    """A time offset to pad the blocks generated in the hessian/jacobian to ensure a constant symbolic representation
    of the batch estimation problem, even when a change in the shutter timing shifts the capture time to another
    spline segment.
    """

    inverseFeatureCovariance = 1/0.26
    """The inverse covariance of the feature detector. Used to standardize the error terms."""

    estimateParameters = {'shutter': True, 'intrinsics': True, 'distortion': True, 'pose': True, 'landmarks': False}
    """Which parameters to estimate. Dictionary with shutter, intrinsics, distortion, pose, landmarks as bool"""

    splineOrder = 4
    """Order of the spline to use for ct-parametrization"""

    timeOffsetPadding = 0.05
    """Time offset to add to the beginning and end of the spline to ensure we remain
    in-bounds while estimating time-depending parameters that shift the spline.
    """

    numberOfKnots = None
    """Set to an integer to start with a fixed number of uniformly distributed knots on the spline."""

    W = None
    """6x6 diagonal matrix with a weak motion prior"""

    framerate = 30
    """The approximate framerate of the camera. Required as approximate threshold in adaptive
    knot placement and for initializing a knot sequence if no number of knots is given.
    """

    def __init__(self):
        # ``estimateParameters`` was historically a class attribute.  Give
        # every run an independent copy while retaining all upstream defaults.
        self.estimateParameters = dict(type(self).estimateParameters)
        self.cameraInitialization = None
        """Optional cam0 intrinsics/distortion seed mapping."""
        self.initializationStrategy = None
        """Provenance for the project initialization strategy."""
        self.lineDelaySeed = None
        """Optional explicit rolling-shutter line-delay seed in seconds."""
        self.maxAbsLineDelay = None
        """Optional post-solve line-delay validity limit in seconds."""

    def validate(self, isRollingShutter):
        """Validate the configuration."""
        # only rolling shutters can be estimated
        if (not isRollingShutter):
            self.estimateParameters['shutter'] = False
            self.adaptiveKnotPlacement = False
        for value, name in (
                (self.timeOffsetConstantSparsityPattern,
                 "timeOffsetConstantSparsityPattern"),
                (self.timeOffsetPadding, "timeOffsetPadding")):
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not np.isfinite(value) or value <= 0.0):
                raise ValueError(name + " must be a positive finite number")
        if (not isinstance(self.framerate, (int, float)) or
                isinstance(self.framerate, bool) or
                not np.isfinite(self.framerate) or self.framerate <= 0.0):
            raise ValueError("framerate must be a positive finite number")
        if self.lineDelaySeed is not None:
            if (not isinstance(self.lineDelaySeed, (int, float)) or
                    isinstance(self.lineDelaySeed, bool) or
                    not np.isfinite(self.lineDelaySeed)):
                raise ValueError(
                    "lineDelaySeed must be a finite number in seconds")
            if not isRollingShutter:
                raise ValueError(
                    "lineDelaySeed requires a rolling-shutter camera model")
        if self.maxAbsLineDelay is not None:
            if (not isinstance(self.maxAbsLineDelay, (int, float)) or
                    isinstance(self.maxAbsLineDelay, bool) or
                    not np.isfinite(self.maxAbsLineDelay) or
                    self.maxAbsLineDelay <= 0.0):
                raise ValueError(
                    "maxAbsLineDelay must be a positive finite number in seconds")
            if not isRollingShutter:
                raise ValueError(
                    "maxAbsLineDelay requires a rolling-shutter camera model")

    def validateTimeSupport(self, sensorRows, lineDelay):
        """Ensure the initial row span plus trial buffer fits the spline ends."""
        if type(sensorRows) is not int or sensorRows <= 0:
            raise ValueError("sensorRows must be a positive integer")
        lineDelay = float(lineDelay)
        if not np.isfinite(lineDelay):
            raise ValueError("initial line delay must be finite")
        endpointSupport = 2.0 * float(self.timeOffsetPadding)
        requiredSupport = (
            float(self.timeOffsetConstantSparsityPattern)
            + abs(lineDelay) * (sensorRows - 1))
        if endpointSupport <= requiredSupport:
            raise ValueError(
                "rolling-shutter spline endpoint support {0:.17g} s must "
                "strictly exceed time-expression buffer plus initial row span "
                "{1:.17g} s".format(endpointSupport, requiredSupport))

class RsCalibrator(object):

    __observations = None
    """Store the list of observations"""

    __cameraGeometry = None
    """The geometry container of which the calibration is performed."""

    __camera = None
    """The camera geometry itself."""

    __camera_dv = None
    """The camera design variable"""

    __cameraModelFactory = None
    """Factory object that can create a typed objects for a camera (error terms, frames, design variables etc)"""

    __poseSpline = None
    """The spline describing the pose of the camera"""

    __poseSpline_dv = None
    """The design variable representation of the pose spline of the camera"""

    __config = None
    """Configuration container \see RsCalibratorConfiguration"""

    __frames = []
    """All frames observed"""

    __reprojection_errors = []
    """Reprojection errors of the latest optimizer iteration"""

    __residual_records = []
    """Observation/corner metadata paired with the final error terms."""

    __optimizerResult = None
    """Return value from the latest optimizer invocation."""

    def calibrate(self,
        cameraGeometry,
        observations,
        config
    ):
        """
        A Motion regularization term is added with low a priori knowledge to avoid
        diverging parts in the spline of too many knots are selected/provided or if
        no image information is available for long sequences and to regularize the
        last few frames (which typically contain no image information but need to have
        knots to /close/ the spline).

        Kwargs:
            cameraGeometry (kcc.CameraGeometry): a camera geometry object with an initialized target
            observations ([]: The list of observation \see extractCornersFromDataset
            config (RsCalibratorConfiguration): calibration configuration
        """

        ## set internal objects
        self.__observations = observations
        self.__cameraGeometry = cameraGeometry
        self.__cameraModelFactory = cameraGeometry.model
        self.__camera_dv = cameraGeometry.dv
        self.__camera = cameraGeometry.geometry
        self.__config = config

        self.__config.validate(self.__isRollingShutter())
        if not self.__observations:
            raise RuntimeError(
                "native rolling-shutter calibration received no target "
                "observations")

        # obtain initial guesses for extrinsics and intrinsics
        if (not self.__generateIntrinsicsInitialGuess()):
            raise RuntimeError("Could not generate intrinsic initial guess.")

        # obtain the extrinsic initial guess for every observation
        self.__generateExtrinsicsInitialGuess()

        # set the value for the motion prior term or uses the defaults
        W = self.__getMotionModelPriorOrDefault()

        self.__poseSpline = self.__generateInitialSpline(
            self.__config.splineOrder,
            self.__config.timeOffsetPadding,
            self.__config.numberOfKnots,
            self.__config.framerate
        )

        # build estimator problem
        optimisation_problem = self.__buildOptimizationProblem(W)

        self.__optimizerResult = self.__runOptimization(
            optimisation_problem,
            self.__config.deltaJ,
            self.__config.deltaX,
            self.__config.maxNumberOfIterations
        )
        requireOptimizerSuccess(
            self.__optimizerResult,
            self.__config.maxNumberOfIterations,
            "initial solve")

        # continue with knot replacement
        if self.__config.adaptiveKnotPlacement:
            knotUpdateStrategy = self.__config.knotUpdateStrategy(self.__config.framerate)

            for iteration in range(self.__config.maxKnotPlacementIterations):

                # generate the new knots list
                [knots, requiresUpdate] = knotUpdateStrategy.generateKnotList(
                    self.__reprojection_errors,
                    self.__poseSpline_dv.spline()
                )
                # if no new knotlist was generated, we are done.
                if (not requiresUpdate):
                    break;

                # otherwise update the spline dv and rebuild the problem
                self.__poseSpline = knotUpdateStrategy.getUpdatedSpline(self.__poseSpline_dv.spline(), knots, self.__config.splineOrder)

                optimisation_problem = self.__buildOptimizationProblem(W)
                self.__optimizerResult = self.__runOptimization(
                    optimisation_problem,
                    self.__config.deltaJ,
                    self.__config.deltaX,
                    self.__config.maxNumberOfIterations
                )
                requireOptimizerSuccess(
                    self.__optimizerResult,
                    self.__config.maxNumberOfIterations,
                    "adaptive knot iteration {0}".format(iteration + 1))

        self.__printResults()
        self.__validateFinalLineDelay()
        self.__saveParametersYaml()
        self.__saveResultText()
        return self.getResult()

    def __generateExtrinsicsInitialGuess(self):
        """Keep only finite PnP poses used to initialize the native spline.

        Corner extraction may succeed while pose initialization fails for an
        individual frame.  The upstream implementation warned about that frame
        but subsequently called ``T_t_c()`` on it unconditionally.  Preserve
        the native solver stages while removing only unusable initialization
        samples and failing explicitly when too few remain for the spline.
        """
        initialized = []
        failed = []
        for idx, observation in enumerate(self.__observations):
            try:
                success, T_t_c = self.__camera.estimateTransformation(
                    observation)
            except (RuntimeError, ValueError) as error:
                success = False
                failed.append((idx, str(error)))
            if success:
                matrix = np.asarray(T_t_c.T(), dtype=float)
                timestamp = float(observation.time().toSec())
                if matrix.shape == (4, 4) and np.all(np.isfinite(matrix)) \
                        and np.isfinite(timestamp):
                    observation.set_T_t_c(T_t_c)
                    initialized.append((timestamp, idx, observation))
                    continue
                failed.append((idx, "non-finite pose or timestamp"))
            elif not failed or failed[-1][0] != idx:
                failed.append((idx, "PnP returned failure"))

        # Dataset readers normally provide monotonic unique timestamps.  Sort
        # defensively and discard exact duplicate pose samples because the
        # spline initializer requires increasing abscissae.
        retained = []
        duplicate_count = 0
        for timestamp, unused_idx, observation in sorted(
                initialized, key=lambda item: (item[0], item[1])):
            if retained and abs(timestamp - retained[-1][0]) <= 1e-12:
                duplicate_count += 1
                continue
            retained.append((timestamp, observation))

        minimum = max(4, int(self.__config.splineOrder))
        if len(retained) < minimum:
            raise RuntimeError(
                "native rolling-shutter spline initialization requires at "
                "least {0} finite, unique PnP target poses; retained {1} of "
                "{2} observations ({3} PnP failures, {4} duplicate "
                "timestamps)".format(
                    minimum, len(retained), len(self.__observations),
                    len(failed), duplicate_count))
        if failed or duplicate_count:
            sm.logWarn(
                "Native rolling-shutter initialization retained {0} of {1} "
                "observations ({2} PnP failures, {3} duplicate timestamps)"
                .format(len(retained), len(self.__observations), len(failed),
                        duplicate_count))
        self.__observations = [item[1] for item in retained]

    def __generateIntrinsicsInitialGuess(self):
        """
        Get an initial guess for the camera geometry (intrinsics, distortion). Distortion is typically left as 0,0,0,0.
        The parameters of the geometryModel are updated in place.
        """
        if (self.__isRollingShutter()):
            sensorRows = self.__observations[0].imRows()
            lineDelay = self.__config.lineDelaySeed
            if lineDelay is None:
                # Preserve the native default.  An explicit seed deliberately
                # breaks this dependency on the selected observation rate.
                lineDelay = 1.0 / self.__config.framerate / float(sensorRows)
            self.__config.validateTimeSupport(sensorRows, lineDelay)
            self.__camera.shutter().setParameters(
                np.array([float(lineDelay)]))

        seed = self.__config.cameraInitialization
        if seed:
            # The native RS solver has one joint K/D/shutter/pose solve rather
            # than the staged intrinsic LM used by the ordinary camera task.
            # Bootstrap only the image metadata, then install the requested
            # initial K/D values before the original joint problem is built.
            if 'intrinsics' in seed:
                self.__cameraGeometry._bootstrapObservationMetadata(
                    self.__observations, 'cam0')
            else:
                if not self.__camera.initializeIntrinsics(
                        self.__observations):
                    return False
            self.__cameraGeometry._restoreUnseededInitializationDefaults(seed)
            self.__cameraGeometry._applyInitializationSeed('cam0', seed)
            return True

        return self.__camera.initializeIntrinsics(self.__observations)

    def __getMotionModelPriorOrDefault(self):
        """Get the motion model prior or the default value"""
        W = self.__config.W
        if W is None:
            W = np.eye(6)
            W[:3,:3] *= 1e-3
            W[3:,3:] *= 1
            W *= 1e-2
        return W

    def __generateInitialSpline(self, splineOrder, timeOffsetPadding, numberOfKnots = None, framerate = None):
        poseSpline = bsplines.BSplinePose(splineOrder, sm.RotationVector())

        # Get the observation times.
        times = np.array([observation.time().toSec() for observation in self.__observations ])
        # get the pose values of the initial transformations at observation time
        curve = np.matrix([ poseSpline.transformationToCurveValue( observation.T_t_c().T() ) for observation in self.__observations]).T
        # make sure all values are well defined
        if np.isnan(curve).any():
            raise RuntimeError("Nans in curve values")
            sys.exit(0)
        # Add 2 seconds on either end to allow the spline to slide during optimization
        times = np.hstack((times[0] - (timeOffsetPadding * 2.0), times, times[-1] + (timeOffsetPadding * 2.0)))
        curve = np.hstack((curve[:,0], curve, curve[:,-1]))

        self.__ensureContinuousRotationVectors(curve)

        seconds = times[-1] - times[0]

        # fixed number of knots
        if (numberOfKnots is not None):
            knots = numberOfKnots
        # otherwise with framerate estimate
        else:
            knots = int(round(seconds * framerate/3))

        print("")
        print("Initializing a pose spline with %d knots (%f knots per second over %f seconds)" % ( knots, 100, seconds))
        poseSpline.initPoseSplineSparse(times, curve, knots, 1e-4)

        return poseSpline

    def __buildOptimizationProblem(self, W):
        """Build the optimisation problem"""
        problem = inc.CalibrationOptimizationProblem()

        # Initialize all design variables.
        self.__initPoseDesignVariables(problem)

        #####
        ## build error terms and add to problem

        # store all frames
        self.__frames = []
        self.__reprojection_errors = []
        self.__residual_records = []

        # This code assumes that the order of the landmarks in the observations
        # is invariant across all observations. At least for the chessboards it is true.

        #####
        # add all the landmarks once
        landmarks = []
        landmarks_expr = []
        target = self.__cameraGeometry.ctarget.detector.target()
        for idx in range(0, target.size()):
            # design variable for landmark
            landmark_w_dv = aopt.HomogeneousPointDv(sm.toHomogeneous(target.point(idx)))
            landmark_w_dv.setActive(self.__config.estimateParameters['landmarks'])
            landmarks.append(landmark_w_dv)
            landmarks_expr.append(landmark_w_dv.toExpression())
            problem.addDesignVariable(landmark_w_dv, LANDMARK_GROUP_ID)

        #####
        # activate design variables
        self.__camera_dv.setActive(
            self.__config.estimateParameters['intrinsics'],
            self.__config.estimateParameters['distortion'],
            self.__config.estimateParameters['shutter']
        )

        #####
        # Add design variables

        # add the camera design variables last for optimal sparsity patterns
        problem.addDesignVariable(self.__camera_dv.shutterDesignVariable(), CALIBRATION_GROUP_ID)
        problem.addDesignVariable(self.__camera_dv.projectionDesignVariable(), CALIBRATION_GROUP_ID)
        problem.addDesignVariable(self.__camera_dv.distortionDesignVariable(), CALIBRATION_GROUP_ID)

        #####
        # Regularization term / motion prior
        motionError = asp.BSplineMotionError(self.__poseSpline_dv, W)
        problem.addErrorTerm(motionError)

        #####
        # add a reprojection error for every corner of each observation
        for observation in self.__observations:
            # only process successful observations of a pattern
            if observation.hasSuccessfulObservation():
                # add a frame
                frame = self.__cameraModelFactory.frameType()
                frame.setGeometry(self.__camera)
                frame.setTime(observation.time())
                self.__frames.append(frame)

                #####
                # add an error term for every observed corner
                corner_ids = observation.getCornersIdx()
                for index, point in enumerate(observation.getCornersImageFrame()):

                    # keypoint time offset by line delay as expression type
                    keypoint_time = self.__camera_dv.keypointTime(frame.time(), point)

                    # from target to world transformation.
                    T_w_t = self.__poseSpline_dv.transformationAtTime(
                        keypoint_time,
                        self.__config.timeOffsetConstantSparsityPattern,
                        self.__config.timeOffsetConstantSparsityPattern
                    )
                    T_t_w = T_w_t.inverse()

                    # transform target point to camera frame
                    p_t = T_t_w * landmarks_expr[corner_ids[index]]

                    # create the keypoint
                    keypoint_index = frame.numKeypoints()
                    keypoint = acv.Keypoint2()
                    keypoint.setMeasurement(point)
                    inverseFeatureCovariance = self.__config.inverseFeatureCovariance
                    keypoint.setInverseMeasurementCovariance(np.eye(len(point)) * inverseFeatureCovariance)
                    frame.addKeypoint(keypoint)

                    # create reprojection error
                    reprojection_error = self.__buildErrorTerm(
                        frame,
                        keypoint_index,
                        p_t,
                        self.__camera_dv,
                        self.__poseSpline_dv
                    )
                    self.__reprojection_errors.append(reprojection_error)
                    self.__residual_records.append({
                        'observation': observation,
                        'corner_id': int(corner_ids[index]),
                        'measurement_px': np.asarray(
                            point, dtype=float).reshape(-1).copy(),
                        'error': reprojection_error,
                    })
                    problem.addErrorTerm(reprojection_error)

        return problem

    def __buildErrorTerm(self, frame, keypoint_index, p_t, camera_dv, poseSpline_dv):
        """
        Build an error term that considers the shutter type. A Global Shutter camera gets the standard reprojection error
        a Rolling Shutter gets the adaptive covariance error term that considers the camera motion.
        """
        # it is a global shutter camera -> no covariance error
        if (self.__isRollingShutter()):
            return self.__cameraModelFactory.reprojectionErrorAdaptiveCovariance(
                frame,
                keypoint_index,
                p_t,
                camera_dv,
                poseSpline_dv
            )
        else:
            return self.__cameraModelFactory.reprojectionError(
                frame,
                keypoint_index,
                p_t,
                camera_dv
            )

    def __ensureContinuousRotationVectors(self, curve):
        """
        Ensures that the rotation vector does not flip and enables a continuous trajectory modeling.
        Updates curves in place.
        """
        for i in range(1, curve.shape[1]):
            previousRotationVector = curve[3:6,i-1]
            r = curve[3:6,i]
            angle = np.linalg.norm(r)
            axis = r/angle
            best_r = r
            best_dist = np.linalg.norm( best_r - previousRotationVector)

            for s in range(-3,4):
                aa = axis * (angle + math.pi * 2.0 * s)
                dist = np.linalg.norm( aa - previousRotationVector )
                if dist < best_dist:
                    best_r = aa
                    best_dist = dist
            curve[3:6,i] = best_r

    def __initPoseDesignVariables(self, problem):
        """Get the design variable representation of the pose spline and add them to the problem"""
        # get the design variable
        self.__poseSpline_dv = asp.BSplinePoseDesignVariable(self.__poseSpline)
        # activate all contained dv and add to problem
        for i in range(0, self.__poseSpline_dv.numDesignVariables()):
            dv = self.__poseSpline_dv.designVariable(i)
            dv.setActive(self.__config.estimateParameters['pose'])
            problem.addDesignVariable(dv, CALIBRATION_GROUP_ID)

    def __runOptimization(self, problem ,deltaJ, deltaX, maxIt):
        """Run the given optimization problem problem"""

        print("run new optimisation with initial values:")
        self.__printResults()

        # verbose and choldmod solving with schur complement trick
        options = aopt.Optimizer2Options()
        options.verbose = True
        options.nThreads = max(1,multiprocessing.cpu_count()-1)
        if native_runtime is not None:
            native_runtime.apply_optimizer_threads(options)
        options.doSchurComplement = True
        options.linearSolver = aopt.BlockCholeskyLinearSystemSolver()  #does not have multi-threading support

        # stopping criteria
        options.maxIterations = maxIt
        options.convergenceDeltaJ = deltaJ
        options.convergenceDeltaX = deltaX

        # use the dogleg trustregion policy
        options.trustRegionPolicy = aopt.DogLegTrustRegionPolicy()

        # create the optimizer
        optimizer = aopt.Optimizer2(options)
        optimizer.setProblem(problem)

        # go for it:
        if native_runtime is not None:
            return native_runtime.run_optimizer(optimizer)
        return optimizer.optimize()

    def __isRollingShutter(self):
        return self.__cameraModelFactory.shutterType == acv.RollingShutter

    def __printResults(self):
        shutter = self.__camera_dv.shutterDesignVariable().value()
        proj = self.__camera_dv.projectionDesignVariable().value()
        dist = self.__camera_dv.distortionDesignVariable().value()
        print("")
        if (self.__isRollingShutter()):
            print("LineDelay:")
            print(shutter.lineDelay())
        print("Intrinsics:")
        print(proj.getParameters().flatten())
        print("Distortion:")
        print(dist.getParameters().flatten())

    def __validateFinalLineDelay(self):
        bound = self.__config.maxAbsLineDelay
        if bound is None:
            return
        lineDelay = float(self.__camera.shutter().lineDelay())
        if not np.isfinite(lineDelay) or abs(lineDelay) > float(bound):
            raise RuntimeError(
                "estimated line delay {0:.17g} s exceeds configured "
                "post-solve limit {1:.17g} s".format(
                    lineDelay, float(bound)))

    def __saveParametersYaml(self):
        # Create new config file
        bagtag = os.path.splitext(self.__cameraGeometry.dataset.bagfile)[0]
        resultFile = bagtag + "-camchain.yaml"
        chain = cr.CameraChainParameters(resultFile, createYaml=True)
        camParams = cr.CameraParameters(resultFile, createYaml=True)
        camParams.setRosTopic(self.__cameraGeometry.dataset.topic)

        # Intrinsics
        cameraModels = {
            # Rolling shutter
            acvb.DistortedPinholeRs: 'pinhole',
            acvb.EquidistantPinholeRs: 'pinhole',
            acvb.DistortedOmniRs: 'omni',
            # Global shutter
            acvb.DistortedPinhole: 'pinhole',
            acvb.EquidistantPinhole: 'pinhole',
            acvb.DistortedOmni: 'omni'}
        cameraModel = cameraModels[self.__cameraGeometry.model]
        proj = self.__camera_dv.projectionDesignVariable().value()
        camParams.setIntrinsics(cameraModel, proj.getParameters().flatten())
        camParams.setResolution([proj.ru(), proj.rv()])

        # Distortion
        distortionModels = {
            # Rolling shutter
            acvb.DistortedPinholeRs: 'radtan',
            acvb.EquidistantPinholeRs: 'equidistant',
            acvb.DistortedOmniRs: 'radtan',
            # Global shutter
            acvb.DistortedPinhole: 'radtan',
            acvb.EquidistantPinhole: 'equidistant',
            acvb.DistortedOmni: 'radtan'}
        distortionModel = distortionModels[self.__cameraGeometry.model]
        dist = self.__camera_dv.distortionDesignVariable().value()
        camParams.setDistortion(distortionModel, dist.getParameters().flatten())

        # Shutter
        shutter = self.__camera_dv.shutterDesignVariable().value()
        camParams.setLineDelay(shutter.lineDelay())

        chain.addCameraAtEnd(camParams)
        chain.writeYaml()

    def getResiduals(self):
        """Return a detached snapshot of final per-corner pixel residuals.

        Residuals use Kalibr's camera convention ``measurement - prediction``.
        The adaptive-covariance weighting remains available through the native
        error terms, but does not alter these physical pixel values.
        """
        if not self.__residual_records:
            raise RuntimeError(
                "rolling-shutter calibration produced no reprojection residuals")

        lineDelay = 0.0
        if self.__isRollingShutter():
            lineDelay = float(self.__camera.shutter().lineDelay())
        residuals = []
        for record in self.__residual_records:
            error = record['error']
            error.evaluateError()
            measurement = np.asarray(
                record['measurement_px'], dtype=float).reshape(-1)
            residual = np.asarray(error.error(), dtype=float).reshape(-1)
            prediction = measurement - residual
            observation = record['observation']
            values = np.hstack((measurement, prediction, residual))
            if (measurement.size != 2 or residual.size != 2 or
                    prediction.size != 2 or not np.all(np.isfinite(values))):
                raise RuntimeError(
                    "native rolling-shutter solver produced a non-finite "
                    "two-dimensional pixel residual")
            timestampNs = int(observation.time().toNSec())
            rowTimeOffset = float(measurement[1] * lineDelay)
            residuals.append({
                'timestamp_ns': timestampNs,
                'solver_timestamp_s': float(
                    observation.time().toSec() + rowTimeOffset),
                'corner_id': int(record['corner_id']),
                'measurement_px': measurement.tolist(),
                'prediction_px': prediction.tolist(),
                'residual_px': residual.tolist(),
                'row_time_offset_s': rowTimeOffset,
            })
        return residuals

    def getResult(self):
        """Return a detached structured snapshot of the optimized camera."""
        residuals = self.getResiduals()
        squaredNorms = [
            float(np.dot(entry['residual_px'], entry['residual_px']))
            for entry in residuals
        ]
        rms = float(np.sqrt(np.mean(squaredNorms)))
        if not np.isfinite(rms):
            raise RuntimeError(
                "native rolling-shutter solver produced a non-finite RMS")
        projection = self.__camera.projection()
        result = {
            'intrinsics': np.asarray(
                projection.getParameters(), dtype=float).reshape(-1).tolist(),
            'distortion_coeffs': np.asarray(
                projection.distortion().getParameters(),
                dtype=float).reshape(-1).tolist(),
            'resolution': [int(projection.ru()), int(projection.rv())],
            'line_delay_s': float(self.__camera.shutter().lineDelay()),
            'line_delay_estimated': bool(
                self.__config.estimateParameters['shutter']),
            'rms_px': rms,
            'residual_count': len(residuals),
            'frame_count': len(set(
                entry['timestamp_ns'] for entry in residuals)),
        }
        numeric = (result['intrinsics'] + result['distortion_coeffs'] +
                   [result['line_delay_s']])
        if not all(np.isfinite(value) for value in numeric):
            raise RuntimeError(
                "native rolling-shutter solver produced non-finite parameters")
        return result

    def getOptimizerResult(self):
        """Return the final native optimizer status for run diagnostics."""
        return self.__optimizerResult

    def __saveResultText(self):
        """Write the legacy human-readable sidecar from structured values."""
        bagtag = os.path.splitext(self.__cameraGeometry.dataset.bagfile)[0]
        resultFile = bagtag + "-results-cam.txt"
        result = self.getResult()
        with open(resultFile, 'w') as stream:
            print("Rolling-shutter camera calibration results", file=stream)
            print("==========================================", file=stream)
            print("topic: {0}".format(
                self.__cameraGeometry.dataset.topic), file=stream)
            print("intrinsics: {0}".format(
                result['intrinsics']), file=stream)
            print("distortion: {0}".format(
                result['distortion_coeffs']), file=stream)
            print("line delay [s]: {0:.17g}".format(
                result['line_delay_s']), file=stream)
            print("reprojection RMS [px]: {0:.17g}".format(
                result['rms_px']), file=stream)
            print("reprojection residuals: {0}".format(
                result['residual_count']), file=stream)
            print("used frames: {0}".format(
                result['frame_count']), file=stream)
