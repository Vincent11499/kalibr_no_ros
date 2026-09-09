import aslam_backend as aopt
import kalibr_native_optimizer as native_runtime
from kalibr_no_ros import artifacts as run_artifacts
# kalibr-native-source-overlay
import aslam_splines as asp
from . import IccUtil as util
import incremental_calibration as inc
import kalibr_common as kc
import sm

import gc
import numpy as np
import multiprocessing
import sys

# make numpy print prettier
np.set_printoptions(suppress=True)

CALIBRATION_GROUP_ID = 0
HELPER_GROUP_ID = 1

def addSplineDesignVariables(problem, dvc, setActive=True, group_id=HELPER_GROUP_ID):
    for i in range(0,dvc.numDesignVariables()):
        dv = dvc.designVariable(i)
        dv.setActive(setActive)
        problem.addDesignVariable(dv, group_id)

class IccCalibrator(object):
    def __init__(self):
        self.ImuList = []

    def initDesignVariables(self, problem, poseSpline, noTimeCalibration, noChainExtrinsics=True, \
                            estimateGravityLength=False, initialGravityEstimate=np.array([0.0,9.81,0.0])):        
        # Initialize the system pose spline (always attached to imu0) 
        self.poseDv = asp.BSplinePoseDesignVariable( poseSpline )
        addSplineDesignVariables(problem, self.poseDv)

        # Add the calibration target orientation design variable. (expressed as gravity vector in target frame)
        if estimateGravityLength:
            self.gravityDv = aopt.EuclideanPointDv( initialGravityEstimate )
        else:
            self.gravityDv = aopt.EuclideanDirection( initialGravityEstimate )
        self.gravityExpression = self.gravityDv.toExpression()  
        self.gravityDv.setActive( True )
        problem.addDesignVariable(self.gravityDv, HELPER_GROUP_ID)
        
        #Add all DVs for all IMUs
        for imu in self.ImuList:
            imu.addDesignVariables( problem )
        
        #Add all DVs for the camera chain    
        self.CameraChain.addDesignVariables( problem, noTimeCalibration, noChainExtrinsics )

    def addPoseMotionTerms(self, problem, tv, rv):
        wt = 1.0/tv;
        wr = 1.0/rv
        W = np.diag([wt,wt,wt,wr,wr,wr])
        asp.addMotionErrorTerms(problem, self.poseDv, W, errorOrder)
        
    #add camera to sensor list (create list if necessary)
    def registerCamChain(self, sensor):
        self.CameraChain = sensor

    def registerImu(self, sensor):
        self.ImuList.append( sensor )
            
    def buildProblem( self, 
                      splineOrder=6, 
                      poseKnotsPerSecond=70, 
                      biasKnotsPerSecond=70, 
                      doPoseMotionError=False, 
                      mrTranslationVariance=1e6,
                      mrRotationVariance=1e5,
                      doBiasMotionError=True,
                      blakeZisserCam=-1,
                      huberAccel=-1,
                      huberGyro=-1,
                      noTimeCalibration=False,
                      noChainExtrinsics=True,
                      maxIterations=20,
                      gyroNoiseScale=1.0,
                      accelNoiseScale=1.0,
                      timeOffsetPadding=0.02,
                      verbose=False  ):

        print("\tSpline order: %d" % (splineOrder))
        print("\tPose knots per second: %d" % (poseKnotsPerSecond))
        print("\tDo pose motion regularization: %s" % (doPoseMotionError))
        print("\t\txddot translation variance: %f" % (mrTranslationVariance))
        print("\t\txddot rotation variance: %f" % (mrRotationVariance))
        print("\tBias knots per second: %d" % (biasKnotsPerSecond))
        print("\tDo bias motion regularization: %s" % (doBiasMotionError))
        print("\tBlake-Zisserman on reprojection errors %s" % blakeZisserCam)
        print("\tAcceleration Huber width (sigma): %f" % (huberAccel))
        print("\tGyroscope Huber width (sigma): %f" % (huberGyro))
        print("\tDo time calibration: %s" % (not noTimeCalibration))
        print("\tMax iterations: %d" % (maxIterations))
        print("\tTime offset padding: %f" % (timeOffsetPadding))


        ############################################
        ## initialize camera chain
        ############################################
        #estimate the timeshift for all cameras to the main imu
        self.noTimeCalibration = noTimeCalibration
        if (noTimeCalibration and
                getattr(self.CameraChain, 'hasInitialization', False)):
            for cam in self.CameraChain.camList:
                if (getattr(cam, 'hasTimeshiftInitialization', False) and
                        getattr(cam, 'initializationStrategy', None) ==
                        'refine'):
                    raise ValueError(
                        "refine initialization of camera-IMU time shifts "
                        "requires time calibration to be enabled")
        if not noTimeCalibration:
            for cam in self.CameraChain.camList:
                native_runtime.timed_call(
                    "time_offset_initialization", "initialization",
                    {"topic": cam.dataset.topic},
                    cam.findTimeshiftCameraImuPrior,
                    self.ImuList[0], verbose)
        
        #obtain orientation prior between main imu and camera chain (if no external input provided)
        #and initial estimate for the direction of gravity
        native_runtime.timed_call(
            "orientation_prior_initialization", "initialization", {},
            self.CameraChain.findOrientationPriorCameraChainToImu,
            self.ImuList[0])
        estimatedGravity = self.CameraChain.getEstimatedGravity()

        ############################################
        ## init optimization problem
        ############################################
        #initialize a pose spline using the camera poses in the camera chain
        poseSpline = native_runtime.timed_call(
            "pose_spline_init", "initialization",
            {"spline_order": splineOrder,
             "knots_per_second": poseKnotsPerSecond},
            self.CameraChain.initializePoseSplineFromCameraChain,
            splineOrder, poseKnotsPerSecond, timeOffsetPadding)
        
        # Initialize bias splines for all IMUs
        for imu in self.ImuList:
            native_runtime.timed_call(
                "bias_spline_init", "initialization",
                {"topic": imu.dataset.topic,
                 "spline_order": splineOrder,
                 "knots_per_second": biasKnotsPerSecond},
                imu.initBiasSplines, poseSpline, splineOrder,
                biasKnotsPerSecond)
        
        # Now I can build the problem
        problem = inc.CalibrationOptimizationProblem()

        # Initialize all design variables.
        native_runtime.timed_call(
            "design_variable_initialization", "problem_build", {},
            self.initDesignVariables, problem, poseSpline,
            noTimeCalibration, noChainExtrinsics,
            initialGravityEstimate=estimatedGravity)
        
        ############################################
        ## add error terms
        ############################################
        #Add calibration target reprojection error terms for all camera in chain
        native_runtime.timed_call(
            "camera_error_build", "problem_build", {},
            self.CameraChain.addCameraChainErrorTerms,
            problem, self.poseDv, blakeZissermanDf=blakeZisserCam,
            timeOffsetPadding=timeOffsetPadding)
        
        # Initialize IMU error terms.
        for imu in self.ImuList:
            native_runtime.timed_call(
                "accel_error_build", "problem_build",
                {"topic": imu.dataset.topic},
                imu.addAccelerometerErrorTerms, problem, self.poseDv,
                self.gravityExpression, mSigma=huberAccel,
                accelNoiseScale=accelNoiseScale)
            native_runtime.timed_call(
                "gyro_error_build", "problem_build",
                {"topic": imu.dataset.topic},
                imu.addGyroscopeErrorTerms, problem, self.poseDv,
                mSigma=huberGyro, gyroNoiseScale=gyroNoiseScale,
                g_w=self.gravityExpression)

            # Add the bias motion terms.
            if doBiasMotionError:
                native_runtime.timed_call(
                    "bias_motion_error_build", "problem_build",
                    {"topic": imu.dataset.topic},
                    imu.addBiasMotionTerms, problem)
            
        # Add the pose motion terms.
        if doPoseMotionError:
            self.addPoseMotionTerms(problem, mrTranslationVariance, mrRotationVariance)
        
        # Add a gravity prior
        self.problem = problem


    def optimize(self, options=None, maxIterations=30, recoverCov=False, profileOptimizer=False):

        if options is None:
            options = aopt.Optimizer2Options()
            options.verbose = True
            options.doLevenbergMarquardt = True
            options.levenbergMarquardtLambdaInit = 10.0
            options.nThreads = max(1,multiprocessing.cpu_count()-1)
            options.convergenceDeltaX = 1e-5
            options.convergenceDeltaJ = 1e-2
            options.maxIterations = maxIterations
            options.trustRegionPolicy = aopt.LevenbergMarquardtTrustRegionPolicy(options.levenbergMarquardtLambdaInit)
            options.linearSolver = aopt.BlockCholeskyLinearSystemSolver() #does not have multi-threading support

        #run the optimization
        native_runtime.apply_optimizer_threads(options)
        self.optimizer = aopt.Optimizer2(options)
        self.optimizer.setProblem(self.problem)

        optimizationFailed=False
        try:
            retval = native_runtime.run_optimizer(self.optimizer)
            artifact_context = run_artifacts.current_context()
            if artifact_context is not None:
                artifact_context.record_optimizer(retval)
            if profileOptimizer:
                self.optimizer.printTiming()
            if retval.linearSolverFailure:
                optimizationFailed = True
        except Exception as e:
            sm.logError(str(e))
            optimizationFailed = True

        if optimizationFailed:
            sm.logError("Optimization failed!")
            raise RuntimeError("Optimization failed!")
        
        #free some memory
        del self.optimizer
        gc.collect()
        if recoverCov:
            self.recoverCovariance()
        

    def recoverCovariance(self):
        #Covariance ordering (=dv ordering)
        #ORDERING:   N=num cams
        #            1. transformation imu-cam0 --> 6
        #            2. camera time2imu --> 1*numCams (only if enabled)
        
        print("Recovering covariance...")
        estimator = inc.IncrementalEstimator(CALIBRATION_GROUP_ID)
        native_runtime.apply_optimizer_threads(
            estimator.getOptimizerOptions())
        rval = native_runtime.run_incremental_batch(
            estimator, self.problem, True)
        est_stds = np.sqrt(estimator.getSigma2Theta().diagonal())
        
        #split and store the variance
        self.std_trafo_ic = np.array(est_stds[0:6])
        self.std_times = np.array(est_stds[6:])
    
    def saveImuSetParametersYaml(self, resultFile):
        imuSetConfig = kc.ImuSetParameters(resultFile, True)
        for imu in self.ImuList:
            imuConfig = imu.getImuConfig()
            imuSetConfig.addImuParameters(imu_parameters=imuConfig)
        imuSetConfig.writeYaml(resultFile)

    def saveCamChainParametersYaml(self, resultFile):    
        chain = self.CameraChain.chainConfig
        nCams = len(self.CameraChain.camList)
    
        # Calibration results
        for camNr in range(0,nCams):
            #cam-cam baselines           
            if camNr > 0:
                T_cB_cA, baseline = self.CameraChain.getResultBaseline(camNr-1, camNr)
                chain.setExtrinsicsLastCamToHere(camNr, T_cB_cA)

            #imu-cam trafos
            T_ci = self.CameraChain.getResultTrafoImuToCam(camNr)
            chain.setExtrinsicsImuToCam(camNr, T_ci)

            cam = self.CameraChain.camList[camNr]
            if (not self.noTimeCalibration or
                    getattr(cam, 'hasTimeshiftInitialization', False)):
                #imu to cam timeshift
                timeshift = float(self.CameraChain.getResultTimeShift(camNr))
                chain.setTimeshiftCamImu(camNr, timeshift)
             
        try:
            chain.writeYaml(resultFile)
        except:
            raise RuntimeError("ERROR: Could not write parameters to file: {0}\n".format(resultFile))
