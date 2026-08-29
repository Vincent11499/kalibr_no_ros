from __future__ import print_function #handle print in 2.x python
import kalibr_native_optimizer as native_runtime
# kalibr-native-source-overlay
import sm
import aslam_cv as acv
import aslam_cameras_april as acv_april
import aslam_splines as asp
import aslam_backend as aopt
import bsplines
import kalibr_common as kc
import kalibr_errorterms as ket
from . import IccCalibrator as ic
from .IccCalibrator import *

import cv2
import sys
import math
import numbers
import re
import numpy as np
import pylab as pl
import scipy.optimize
import yaml


CAMERA_IMU_CALIBRATION_INITIALIZATION_SCHEMA_VERSION = 1
CAMERA_IMU_CALIBRATION_INITIALIZATION_KIND = (
    'camera_imu_calibration_initialization')
_CAMERA_IMU_INITIALIZATION_STRATEGIES = ('refine', 'direct')
_CAMERA_IMU_INITIALIZATION_TOP_LEVEL_KEYS = {
    'schema_version', 'kind', 'strategy', 'camera_imu', 'imus'
}
_CAMERA_IMU_INITIALIZATION_FIELDS = {
    'T_cam0_imu', 'timeshift_cam_imu_s', 'gravity_direction_target'
}
_IMU_INITIALIZATION_FIELDS = {
    'gyroscope_bias_rad_s', 'accelerometer_bias_m_s2',
    'M_accel', 'M_gyro', 'C_gyro_i', 'A_gyro_accel',
    'ry_i_m', 'rz_i_m', 'T_imu_from_reference',
    'time_offset_to_reference_s'
}
_INDEXED_CAMERA = re.compile(r'^cam(?:0|[1-9][0-9]*)$')
_INDEXED_IMU = re.compile(r'^imu(?:0|[1-9][0-9]*)$')


def _cameraImuInitializationError(filename, message):
    raise RuntimeError(
        'Invalid camera-IMU calibration initialization file "{0}": {1}'.format(
            filename, message))


def _cameraImuInitializationMapping(filename, value, name):
    if not isinstance(value, dict):
        _cameraImuInitializationError(
            filename, '{} must be a mapping'.format(name))
    return value


def _cameraImuInitializationCheckKeys(filename, value, allowed, name):
    unknown = sorted(set(value) - set(allowed))
    if unknown:
        _cameraImuInitializationError(
            filename, '{} contains unsupported field(s): {}'.format(
                name, ', '.join(str(item) for item in unknown)))


def _cameraImuInitializationNumber(filename, value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _cameraImuInitializationError(
            filename, '{} must be a finite number'.format(name))
    value = float(value)
    if not np.isfinite(value):
        _cameraImuInitializationError(
            filename, '{} must be a finite number'.format(name))
    return value


def _cameraImuInitializationArray(filename, value, shape, name):
    try:
        rawArray = np.asarray(value, dtype=object)
    except (TypeError, ValueError):
        _cameraImuInitializationError(
            filename, '{} must have shape {}'.format(name, shape))
    if rawArray.shape != shape:
        _cameraImuInitializationError(
            filename,
            '{} must contain finite values with shape {}'.format(name, shape))
    if any(isinstance(item, bool) or not isinstance(item, numbers.Real)
           for item in rawArray.flat):
        _cameraImuInitializationError(
            filename, '{} must contain only numbers'.format(name))
    array = np.asarray(rawArray, dtype=float)
    if not np.isfinite(array).all():
        _cameraImuInitializationError(
            filename,
            '{} must contain finite values with shape {}'.format(name, shape))
    return array


def _cameraImuInitializationRotation(filename, value, name):
    rotation = _cameraImuInitializationArray(
        filename, value, (3, 3), name)
    if (not np.allclose(np.dot(rotation.T, rotation), np.eye(3), atol=1e-6,
                        rtol=0.0) or
            not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6,
                           rtol=0.0)):
        _cameraImuInitializationError(
            filename, '{} must be a proper rotation matrix'.format(name))
    return rotation


def _cameraImuInitializationTransform(filename, value, name):
    transform = _cameraImuInitializationArray(
        filename, value, (4, 4), name)
    _cameraImuInitializationRotation(
        filename, transform[:3, :3], name + ' rotation')
    if not np.allclose(transform[3, :], np.array([0., 0., 0., 1.]),
                       atol=1e-9, rtol=0.0):
        _cameraImuInitializationError(
            filename,
            '{} must have homogeneous bottom row [0, 0, 0, 1]'.format(name))
    return transform


def loadCameraImuCalibrationInitialization(filename, num_cameras=None,
                                           imu_models=None):
    """Load and defensively validate a legacy-facing initialization YAML."""
    try:
        with open(filename, 'r') as stream:
            document = yaml.safe_load(stream)
    except (IOError, OSError) as error:
        raise RuntimeError(
            'Could not read camera-IMU calibration initialization file "{0}": '
            '{1}'.format(filename, error))
    except yaml.YAMLError as error:
        raise RuntimeError(
            'Could not parse camera-IMU calibration initialization file "{0}": '
            '{1}'.format(filename, error))

    document = _cameraImuInitializationMapping(
        filename, document, 'initialization document root')
    _cameraImuInitializationCheckKeys(
        filename, document, _CAMERA_IMU_INITIALIZATION_TOP_LEVEL_KEYS,
        'initialization document')
    if (type(document.get('schema_version')) is not int or
            document.get('schema_version') !=
            CAMERA_IMU_CALIBRATION_INITIALIZATION_SCHEMA_VERSION):
        _cameraImuInitializationError(
            filename, 'schema_version must be integer {}'.format(
                CAMERA_IMU_CALIBRATION_INITIALIZATION_SCHEMA_VERSION))
    if document.get('kind') != CAMERA_IMU_CALIBRATION_INITIALIZATION_KIND:
        _cameraImuInitializationError(
            filename, 'kind must be "{}"'.format(
                CAMERA_IMU_CALIBRATION_INITIALIZATION_KIND))
    if document.get('strategy') not in _CAMERA_IMU_INITIALIZATION_STRATEGIES:
        _cameraImuInitializationError(
            filename, 'strategy must be either "refine" or "direct"')

    if 'camera_imu' in document:
        cameraImu = _cameraImuInitializationMapping(
            filename, document['camera_imu'], 'camera_imu')
        _cameraImuInitializationCheckKeys(
            filename, cameraImu, _CAMERA_IMU_INITIALIZATION_FIELDS,
            'camera_imu')
        if 'T_cam0_imu' in cameraImu:
            _cameraImuInitializationTransform(
                filename, cameraImu['T_cam0_imu'],
                'camera_imu.T_cam0_imu')
        if 'timeshift_cam_imu_s' in cameraImu:
            timeshifts = _cameraImuInitializationMapping(
                filename, cameraImu['timeshift_cam_imu_s'],
                'camera_imu.timeshift_cam_imu_s')
            for cameraName, timeshift in timeshifts.items():
                if (not isinstance(cameraName, str) or
                        not _INDEXED_CAMERA.match(cameraName)):
                    _cameraImuInitializationError(
                        filename,
                        'camera_imu.timeshift_cam_imu_s keys must be camN')
                cameraNr = int(cameraName[3:])
                if num_cameras is not None and cameraNr >= num_cameras:
                    _cameraImuInitializationError(
                        filename, 'camera time shift references unavailable '
                        '{}'.format(cameraName))
                _cameraImuInitializationNumber(
                    filename, timeshift,
                    'camera_imu.timeshift_cam_imu_s.{}'.format(cameraName))
        if 'gravity_direction_target' in cameraImu:
            gravity = _cameraImuInitializationArray(
                filename, cameraImu['gravity_direction_target'], (3,),
                'camera_imu.gravity_direction_target')
            if np.linalg.norm(gravity) <= 1e-12:
                _cameraImuInitializationError(
                    filename,
                    'camera_imu.gravity_direction_target must be nonzero')

    if 'imus' in document:
        imus = _cameraImuInitializationMapping(
            filename, document['imus'], 'imus')
        for imuName, imu in imus.items():
            if not isinstance(imuName, str) or not _INDEXED_IMU.match(imuName):
                _cameraImuInitializationError(
                    filename, 'imus keys must be imuN')
            imuNr = int(imuName[3:])
            if imu_models is not None and imuNr >= len(imu_models):
                _cameraImuInitializationError(
                    filename, 'IMU initialization references unavailable '
                    '{}'.format(imuName))
            imu = _cameraImuInitializationMapping(
                filename, imu, 'imus.{}'.format(imuName))
            _cameraImuInitializationCheckKeys(
                filename, imu, _IMU_INITIALIZATION_FIELDS,
                'imus.{}'.format(imuName))
            if imuNr == 0:
                for field in ('T_imu_from_reference',
                              'time_offset_to_reference_s'):
                    if field in imu:
                        _cameraImuInitializationError(
                            filename, 'imus.imu0 cannot contain {}'.format(
                                field))
            if imu_models is not None and imuNr < len(imu_models):
                model = imu_models[imuNr]
                intrinsicFields = {
                    'M_accel', 'M_gyro', 'C_gyro_i', 'A_gyro_accel'
                }
                sizeEffectFields = {'ry_i_m', 'rz_i_m'}
                if (intrinsicFields.intersection(imu) and model not in
                        ('scale-misalignment',
                         'scale-misalignment-size-effect')):
                    _cameraImuInitializationError(
                        filename, 'intrinsic fields are not valid for {} model '
                        '{}'.format(imuName, model))
                if (sizeEffectFields.intersection(imu) and
                        model != 'scale-misalignment-size-effect'):
                    _cameraImuInitializationError(
                        filename, 'lever-arm fields are not valid for {} model '
                        '{}'.format(imuName, model))
            for field in ('gyroscope_bias_rad_s',
                          'accelerometer_bias_m_s2', 'ry_i_m', 'rz_i_m'):
                if field in imu:
                    _cameraImuInitializationArray(
                        filename, imu[field], (3,),
                        'imus.{}.{}'.format(imuName, field))
            for field in ('M_accel', 'M_gyro', 'A_gyro_accel'):
                if field in imu:
                    matrix = _cameraImuInitializationArray(
                        filename, imu[field], (3, 3),
                        'imus.{}.{}'.format(imuName, field))
                    if field in ('M_accel', 'M_gyro'):
                        if np.any(np.abs(np.triu(matrix, 1)) > 1e-12):
                            _cameraImuInitializationError(
                                filename,
                                'imus.{}.{} must be lower triangular'.format(
                                    imuName, field))
                        if np.any(np.diag(matrix) <= 0.0):
                            _cameraImuInitializationError(
                                filename,
                                'imus.{}.{} diagonal entries must be '
                                'positive'.format(imuName, field))
            if 'C_gyro_i' in imu:
                _cameraImuInitializationRotation(
                    filename, imu['C_gyro_i'],
                    'imus.{}.C_gyro_i'.format(imuName))
            if 'T_imu_from_reference' in imu:
                _cameraImuInitializationTransform(
                    filename, imu['T_imu_from_reference'],
                    'imus.{}.T_imu_from_reference'.format(imuName))
            if 'time_offset_to_reference_s' in imu:
                _cameraImuInitializationNumber(
                    filename, imu['time_offset_to_reference_s'],
                    'imus.{}.time_offset_to_reference_s'.format(imuName))

    return document


def initCameraBagDataset(bagfile, topic, from_to, freq, perform_synchronization):
    print("Initializing camera rosbag dataset reader:")
    print("\tDataset:          {0}".format(bagfile))
    print("\tTopic:            {0}".format(topic))
    reader = native_runtime.timed_call(
        "image_bag_index", "io", {"topic": topic},
        kc.BagImageDatasetReader, bagfile, topic,
        bag_from_to=from_to, bag_freq=freq,
        perform_synchronization=perform_synchronization)
    print("\tNumber of images: {0}".format(len(reader.index)))
    return reader

def initImuBagDataset(bagfile, topic, from_to=None, perform_synchronization=False):
    print("Initializing imu rosbag dataset reader:")
    print("\tDataset:          {0}".format(bagfile))
    print("\tTopic:            {0}".format(topic))
    reader = native_runtime.timed_call(
        "imu_bag_read", "io", {"topic": topic},
        kc.BagImuDatasetReader, bagfile, topic,
        bag_from_to=from_to,
        perform_synchronization=perform_synchronization)
    print("\tNumber of messages: {0}".format(len(reader.index)))
    return reader


#mono camera
class IccCamera():
    def __init__(self, camConfig, targetConfig, dataset, reprojectionSigma=1.0, showCorners=True, \
                 showReproj=True, showOneStep=False, windowHalfSizePx=None,
                 maxDisplacementPx=None):
        
        #store the configuration
        self.dataset = dataset
        self.camConfig = camConfig
        self.targetConfig = targetConfig
        
        # Corner uncertainty
        self.cornerUncertainty = reprojectionSigma

        #set the extrinsic prior to default
        self.T_extrinsic = sm.Transformation()
        self.initializationStrategy = None
        self.hasTransformInitialization = False
        self.hasTimeshiftInitialization = False
        self.hasGravityInitialization = False
         
        #initialize timeshift prior to zero
        self.timeshiftCamToImuPrior = 0.0
        
        #initialize the camera data
        self.camera = kc.AslamCamera.fromParameters( camConfig )
        
        #extract corners
        self.setupCalibrationTarget(
            targetConfig, showExtraction=showCorners, showReproj=showReproj,
            imageStepping=showOneStep,
            windowHalfSizePx=windowHalfSizePx,
            maxDisplacementPx=maxDisplacementPx)
        multithreading = not (showCorners or showReproj or showOneStep)
        self.targetObservations = kc.extractCornersFromDataset(self.dataset, self.detector, multithreading=multithreading)
        
        #an estimate of the gravity in the world coordinate frame  
        self.gravity_w = np.array([9.80655, 0., 0.])

    def setTransformInitialization(self, transform, strategy):
        self.T_extrinsic = sm.Transformation(np.asarray(transform, dtype=float))
        self.initializationStrategy = strategy
        self.hasTransformInitialization = True

    def setTimeshiftInitialization(self, timeshift, strategy):
        self.timeshiftCamToImuPrior = float(timeshift)
        self.initializationStrategy = strategy
        self.hasTimeshiftInitialization = True

    def setGravityInitialization(self, direction, strategy):
        direction = np.asarray(direction, dtype=float)
        self.gravity_w = direction / np.linalg.norm(direction) * 9.80655
        self.initializationStrategy = strategy
        self.hasGravityInitialization = True
        
    def setupCalibrationTarget(self, targetConfig, showExtraction=False,
                               showReproj=False, imageStepping=False,
                               windowHalfSizePx=None,
                               maxDisplacementPx=None):
        
        #load the calibration target configuration
        targetParams = targetConfig.getTargetParams()
        targetType = targetConfig.getTargetType()
    
        if targetType == 'checkerboard':
            options = acv.CheckerboardOptions() 
            options.filterQuads = True
            options.normalizeImage = True
            options.useAdaptiveThreshold = True        
            options.performFastCheck = False
            options.windowWidth = 5
            options.showExtractionVideo = showExtraction
            grid = acv.GridCalibrationTargetCheckerboard(targetParams['targetRows'], 
                                                            targetParams['targetCols'], 
                                                            targetParams['rowSpacingMeters'], 
                                                            targetParams['colSpacingMeters'],
                                                            options)
        elif targetType == 'circlegrid':
            options = acv.CirclegridOptions()
            options.showExtractionVideo = showExtraction
            options.useAsymmetricCirclegrid = targetParams['asymmetricGrid']
            grid = acv.GridCalibrationTargetCirclegrid(targetParams['targetRows'],
                                                          targetParams['targetCols'], 
                                                          targetParams['spacingMeters'], 
                                                          options)
        elif targetType == 'aprilgrid':
            options = acv_april.AprilgridOptions() 
            if windowHalfSizePx is not None:
                options.subpixWindowHalfSize = windowHalfSizePx
            if maxDisplacementPx is not None:
                options.maxSubpixDisplacement2 = maxDisplacementPx * maxDisplacementPx
            options.showExtractionVideo = showExtraction
            options.minTagsForValidObs = int( np.max( [targetParams['tagRows'], targetParams['tagCols']] ) + 1 )
            
            grid = acv_april.GridCalibrationTargetAprilgrid(targetParams['tagRows'],
                                                            targetParams['tagCols'], 
                                                            targetParams['tagSize'], 
                                                            targetParams['tagSpacing'], 
                                                            targetParams['tagStartId'],
                                                            options)
        else:
            raise RuntimeError( "Unknown calibration target." )
                          
        options = acv.GridDetectorOptions() 
        options.imageStepping = imageStepping
        options.plotCornerReprojection = showReproj
        options.filterCornerOutliers = True
        #options.filterCornerSigmaThreshold = 2.0
        #options.filterCornerMinReprojError = 0.2
        self.detector = acv.GridDetector(self.camera.geometry, grid, options)        

    def findOrientationPriorCameraToImu(self, imu):
        print("")
        print("Estimating imu-camera rotation prior")
        
        # build the problem
        problem = aopt.OptimizationProblem()

        # Add the rotation as design variable. This variable maps camera angular
        # rates into IMU coordinates, hence a supplied T_cam0_imu contributes
        # the transpose of its rotation block.
        if self.hasTransformInitialization:
            q_i_c_prior = sm.r2quat(
                self.T_extrinsic.T()[0:3, 0:3].transpose())
        else:
            q_i_c_prior = self.T_extrinsic.q()
        q_i_c_Dv = aopt.RotationQuaternionDv(q_i_c_prior)
        rotationActive = not (
            self.initializationStrategy == 'direct' and
            self.hasTransformInitialization)
        q_i_c_Dv.setActive(rotationActive)
        problem.addDesignVariable(q_i_c_Dv)

        # Add the gyro bias as design variable
        gyroBiasDv = aopt.EuclideanPointDv(imu.GyroBiasPrior)
        biasActive = not (
            getattr(imu, 'initializationStrategy', None) == 'direct' and
            getattr(imu, 'hasGyroBiasInitialization', False))
        gyroBiasDv.setActive(biasActive)
        problem.addDesignVariable(gyroBiasDv)
        
        # The preliminary problem estimates an absolute camera-to-IMU
        # rotation.  A supplied T_cam0_imu is already the starting value of
        # q_i_c_Dv above, so the visual angular-velocity spline must remain in
        # the camera frame here.  Applying the seed to both the spline and the
        # rotation DV would rotate the prediction twice.
        if self.hasTransformInitialization:
            poseSpline = self.initPoseSplineFromCamera(
                timeOffsetPadding=0.0,
                T_c_b_override=np.eye(4))
        else:
            poseSpline = self.initPoseSplineFromCamera(
                timeOffsetPadding=0.0)
        
        for im in imu.imuData:
            tk = im.stamp.toSec()
            if tk > poseSpline.t_min() and tk < poseSpline.t_max():        
                #DV expressions
                R_i_c = q_i_c_Dv.toExpression()
                bias = gyroBiasDv.toExpression()   
                
                #get the vision predicted omega and measured omega (IMU)
                omega_predicted = R_i_c * aopt.EuclideanExpression( np.matrix( poseSpline.angularVelocityBodyFrame( tk ) ).transpose() )
                omega_measured = im.omega
                
                #error term
                gerr = ket.GyroscopeError(omega_measured, im.omegaInvR, omega_predicted, bias)
                problem.addErrorTerm(gerr)
        
        if problem.numErrorTerms() == 0:
            sm.logFatal("Failed to obtain orientation prior. "\
                        "Please make sure that your sensors are synchronized correctly.")
            sys.exit(-1)

        
        # If direct initialization supplies both coupled quantities there is
        # nothing for this preliminary LM to solve. Otherwise supplied values
        # are either fixed (direct) or active starting points (refine).
        if rotationActive or biasActive:
            options = aopt.Optimizer2Options()
            options.verbose = False
            options.linearSolver = aopt.BlockCholeskyLinearSystemSolver() #does not have multi-threading support
            options.nThreads = 2
            options.convergenceDeltaX = 1e-4
            options.convergenceDeltaJ = 1
            options.maxIterations = 50

            native_runtime.apply_optimizer_threads(options)
            optimizer = aopt.Optimizer2(options)
            optimizer.setProblem(problem)

            try:
                native_runtime.run_optimizer(optimizer)
            except:
                sm.logFatal("Failed to obtain orientation prior!")
                sys.exit(-1)

        #overwrite the external rotation prior (keep the external translation prior)
        R_i_c = q_i_c_Dv.toRotationMatrix().transpose()
        self.T_extrinsic = sm.Transformation( sm.rt2Transform( R_i_c, self.T_extrinsic.t() ) )

        #estimate gravity only when no explicit target-frame direction exists
        if not self.hasGravityInitialization:
            a_w = []
            for im in imu.imuData:
                tk = im.stamp.toSec()
                if tk > poseSpline.t_min() and tk < poseSpline.t_max():
                    a_w.append(np.dot(poseSpline.orientation(tk), np.dot(R_i_c, - im.alpha)))
            mean_a_w = np.mean(np.asarray(a_w).T, axis=1)
            self.gravity_w = mean_a_w / np.linalg.norm(mean_a_w) * 9.80655
        print("Gravity was intialized to", self.gravity_w, "[m/s^2]") 

        #set the gyro bias prior (if we have more than 1 cameras use recursive average)
        b_gyro = gyroBiasDv.toEuclidean()
        imu.GyroBiasPriorCount += 1
        imu.GyroBiasPrior = (imu.GyroBiasPriorCount-1.0)/imu.GyroBiasPriorCount * imu.GyroBiasPrior + 1.0/imu.GyroBiasPriorCount*b_gyro

        #print result
        print("  Orientation prior camera-imu found as: (T_i_c)")
        print(R_i_c)
        print("  Gyro bias prior found as: (b_gyro)")
        print(b_gyro)
    
    #return an etimate of gravity in the world coordinate frame as perceived by this camera
    def getEstimatedGravity(self):
        return self.gravity_w
        
    #estimates the timeshift between the camearas and the imu using a crosscorrelation approach
    #
    #approach: angular rates are constant on a fixed body independent of location
    #          using only the norm of the gyro outputs and assuming that the biases are small
    #          we can estimate the timeshift between the cameras and the imu by calculating
    #          the angular rates of the cameras by fitting a spline and evaluating the derivatives
    #          then computing the cross correlating between the "predicted" angular rates (camera)
    #          and imu, the maximum corresponds to the timeshift...
    #          in a next step we can use the time shift to estimate the rotation between camera and imu
    def findTimeshiftCameraImuPrior(self, imu, verbose=False):
        print("Estimating time shift camera to imu:")

        if (self.hasTimeshiftInitialization and
                self.initializationStrategy == 'direct'):
            print("  Using direct camera-to-IMU time shift initialization:")
            print(self.timeshiftCamToImuPrior)
            return

        initialTimeshift = self.timeshiftCamToImuPrior
        
        #fit a spline to the camera observations
        poseSpline = self.initPoseSplineFromCamera( timeOffsetPadding=0.0 )
        
        #predict time shift prior 
        t=[]
        omega_measured_norm = []
        omega_predicted_norm = []
        
        for im in imu.imuData:
            tk = im.stamp.toSec()
            if tk > poseSpline.t_min() and tk < poseSpline.t_max():
                
                #get imu measurements and spline from camera
                omega_measured = im.omega
                omega_predicted = aopt.EuclideanExpression( np.matrix( poseSpline.angularVelocityBodyFrame( tk ) ).transpose() )

                #calc norm
                t = np.hstack( (t, tk) )
                omega_measured_norm = np.hstack( (omega_measured_norm, np.linalg.norm( omega_measured ) ))
                omega_predicted_norm = np.hstack( (omega_predicted_norm, np.linalg.norm( omega_predicted.toEuclidean() )) )
        
        if len(omega_predicted_norm) == 0 or len(omega_measured_norm) == 0:
            sm.logFatal("The time ranges of the camera and IMU do not overlap. "\
                        "Please make sure that your sensors are synchronized correctly.")
            sys.exit(-1)
        
        #get the time shift
        corr = np.correlate(omega_predicted_norm, omega_measured_norm, "full")
        discrete_shift = corr.argmax() - (np.size(omega_measured_norm) - 1)
        
        #get cont. time shift
        times = [im.stamp.toSec() for im in imu.imuData]
        dT = np.mean(np.diff( times ))
        shift = -discrete_shift*dT
        
        #Create plots
        if verbose:
            pl.plot(t, omega_measured_norm, label="measured_raw")
            pl.plot(t, omega_predicted_norm, label="predicted")
            pl.plot(t-shift, omega_measured_norm, label="measured_corrected")
            pl.legend()
            pl.title("Time shift prior camera-imu estimation")
            pl.figure()
            pl.plot(corr)
            pl.title("Cross-correlation ||omega_predicted||, ||omega_measured||")
            pl.show()
            sm.logDebug("discrete time shift: {0}".format(discrete_shift))
            sm.logDebug("cont. time shift: {0}".format(shift))
            sm.logDebug("dT: {0}".format(dT))
        
        #store the timeshift (t_imu = t_cam + timeshiftCamToImuPrior)
        if (self.hasTimeshiftInitialization and
                self.initializationStrategy == 'refine'):
            self.timeshiftCamToImuPrior = initialTimeshift + shift
        else:
            self.timeshiftCamToImuPrior = shift
        
        print("  Time shift camera to imu (t_imu = t_cam + shift):")
        print(self.timeshiftCamToImuPrior)
        
    #initialize a pose spline using camera poses (pose spline = T_wb)
    def initPoseSplineFromCamera(self, splineOrder=6, poseKnotsPerSecond=100,
                                 timeOffsetPadding=0.02,
                                 T_c_b_override=None):
        T_c_b = (self.T_extrinsic.T() if T_c_b_override is None else
                 np.asarray(T_c_b_override, dtype=float))
        pose = bsplines.BSplinePose(splineOrder, sm.RotationVector() )
                
        # Get the checkerboard times.
        times = np.array([obs.time().toSec()+self.timeshiftCamToImuPrior for obs in self.targetObservations ])                 
        curve = np.matrix([ pose.transformationToCurveValue( np.dot(obs.T_t_c().T(), T_c_b) ) for obs in self.targetObservations]).T
        
        if np.isnan(curve).any():
            raise RuntimeError("Nans in curve values")
            sys.exit(0)
        
        # Add 2 seconds on either end to allow the spline to slide during optimization
        times = np.hstack((times[0] - (timeOffsetPadding * 2.0), times, times[-1] + (timeOffsetPadding * 2.0)))
        curve = np.hstack((curve[:,0], curve, curve[:,-1]))
        
        # Make sure the rotation vector doesn't flip
        for i in range(1,curve.shape[1]):
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
            curve[3:6,i] = best_r;
            
        seconds = times[-1] - times[0]
        knots = int(round(seconds * poseKnotsPerSecond))
        
        print("")
        print("Initializing a pose spline with %d knots (%f knots per second over %f seconds)" % ( knots, poseKnotsPerSecond, seconds))
        pose.initPoseSplineSparse(times, curve, knots, 1e-4)
        return pose
    
    def addDesignVariables(self, problem, noExtrinsics=True, noTimeCalibration=True, baselinedv_group_id=HELPER_GROUP_ID):
        # Add the calibration design variables.
        active = not noExtrinsics
        self.T_c_b_Dv = aopt.TransformationDv(self.T_extrinsic, rotationActive=active, translationActive=active)
        for i in range(0, self.T_c_b_Dv.numDesignVariables()):
            problem.addDesignVariable(self.T_c_b_Dv.getDesignVariable(i), baselinedv_group_id)
        
        # Add the time delay design variable.
        self.cameraTimeToImuTimeDv = aopt.Scalar(0.0)
        self.cameraTimeToImuTimeDv.setActive( not noTimeCalibration )
        problem.addDesignVariable(self.cameraTimeToImuTimeDv, CALIBRATION_GROUP_ID)
        
    def addCameraErrorTerms(self, problem, poseSplineDv, T_cN_b, blakeZissermanDf=0.0, timeOffsetPadding=0.0):
        print("")
        print("Adding camera error terms ({0})".format(self.dataset.topic))
        
        #progress bar
        iProgress = sm.Progress2( len(self.targetObservations) )
        iProgress.sample()

        allReprojectionErrors = list()
        error_t = self.camera.reprojectionErrorType
        
        for obs in self.targetObservations:
            # Build a transformation expression for the time.
            frameTime = self.cameraTimeToImuTimeDv.toExpression() + obs.time().toSec() + self.timeshiftCamToImuPrior
            frameTimeScalar = frameTime.toScalar()
            
            #as we are applying an initial time shift outside the optimization so 
            #we need to make sure that we dont add data outside the spline definition
            if frameTimeScalar <= poseSplineDv.spline().t_min() or frameTimeScalar >= poseSplineDv.spline().t_max():
                continue
            
            T_w_b = poseSplineDv.transformationAtTime(frameTime, timeOffsetPadding, timeOffsetPadding)
            T_b_w = T_w_b.inverse()

            #calibration target coords to camera N coords
            #T_b_w: from world to imu coords
            #T_cN_b: from imu to camera N coords
            T_c_w = T_cN_b  * T_b_w
            
            #get the image and target points corresponding to the frame
            imageCornerPoints =  np.array( obs.getCornersImageFrame() ).T
            targetCornerPoints = np.array( obs.getCornersTargetFrame() ).T
            
            #setup an aslam frame (handles the distortion)
            frame = self.camera.frameType()
            frame.setGeometry(self.camera.geometry)
            
            #corner uncertainty
            R = np.eye(2) * self.cornerUncertainty * self.cornerUncertainty
            invR = np.linalg.inv(R)
            
            for pidx in range(0,imageCornerPoints.shape[1]):
                #add all image points
                k = self.camera.keypointType()
                k.setMeasurement( imageCornerPoints[:,pidx] )
                k.setInverseMeasurementCovariance(invR)
                frame.addKeypoint(k)
            
            reprojectionErrors=list()
            for pidx in range(0,imageCornerPoints.shape[1]):
                #add all target points
                targetPoint = np.insert( targetCornerPoints.transpose()[pidx], 3, 1)
                p = T_c_w *  aopt.HomogeneousExpression( targetPoint )
             
                #build and append the error term
                rerr = error_t(frame, pidx, p)
                
                #add blake-zisserman m-estimator
                if blakeZissermanDf>0.0:
                    mest = aopt.BlakeZissermanMEstimator( blakeZissermanDf )
                    rerr.setMEstimatorPolicy(mest)
                
                problem.addErrorTerm(rerr)  
                reprojectionErrors.append(rerr)
            
            allReprojectionErrors.append(reprojectionErrors)
                        
            #update progress bar
            iProgress.sample()
            
        print("\r  Added {0} camera error terms                      ".format( len(self.targetObservations) ))           
        self.allReprojectionErrors = allReprojectionErrors

#pair of cameras with overlapping field of view (perfectly synced cams required!!)
#
#     Sensor "chain"                    R_C1C0 source: *fixed as input from stereo calib
#                                                      *optimized using stereo error terms
#         R_C1C0(R,t)   C1   R_C2C1(R,t)    C2         Cn
# C0  o------------------o------------------o    ...    o 
#     |
#     | R_C0I (R,t)
#     |
#     o (IMU)
#
#imu is need to initialize an orientation prior between imu and camera chain
class IccCameraChain():
    def __init__(self, chainConfig, targetConfig, parsed):

        #create all camera in the chain
        self.camList = []
        self.hasInitialization = False
        for camNr in range(0, chainConfig.numCameras()):
            camConfig = chainConfig.getCameraParameters(camNr)
            dataset = initCameraBagDataset(parsed.bagfile[0], camConfig.getRosTopic(), \
                                           parsed.bag_from_to, parsed.bag_freq, parsed.perform_synchronization)
            
            #create the camera
            self.camList.append( IccCamera( camConfig, 
                                            targetConfig, 
                                            dataset, 
                                            #Ultimately, this should come from the camera yaml.
                                            reprojectionSigma=parsed.reprojection_sigma, 
                                            showCorners=parsed.showextraction,
                                            showReproj=parsed.showextraction, 
                                            showOneStep=parsed.extractionstepping,
                                            windowHalfSizePx=parsed.window_half_size_px,
                                            maxDisplacementPx=parsed.max_displacement_px) )
                
        self.chainConfig = chainConfig
        
        #find and store time between first and last image over all cameras
        self.findCameraTimespan()
        
        #use stereo calibration guess if no baselines are provided
        self.initializeBaselines()

    def applyInitialization(self, initialization, strategy):
        if initialization is None:
            return

        self.hasInitialization = True

        if 'T_cam0_imu' in initialization:
            self.camList[0].setTransformInitialization(
                initialization['T_cam0_imu'], strategy)

        if 'gravity_direction_target' in initialization:
            self.camList[0].setGravityInitialization(
                initialization['gravity_direction_target'], strategy)

        for cameraName, timeshift in initialization.get(
                'timeshift_cam_imu_s', {}).items():
            cameraNr = int(cameraName[3:])
            if cameraNr >= len(self.camList):
                raise ValueError(
                    "Initialization config references unavailable {}".format(
                        cameraName))
            self.camList[cameraNr].setTimeshiftInitialization(
                timeshift, strategy)

    def initializeBaselines(self):
        #estimate baseline prior if no external guess is provided           
        for camNr in range(1, len(self.camList)):
            self.camList[camNr].T_extrinsic = self.chainConfig.getExtrinsicsLastCamToHere(camNr)

            print("Baseline between cam{0} and cam{1} set to:".format(camNr-1,camNr))
            print("T= ", self.camList[camNr].T_extrinsic.T())
            print("Baseline: ", np.linalg.norm(self.camList[camNr].T_extrinsic.t()), " [m]")
   
    #initialize a pose spline for the chain
    def initializePoseSplineFromCameraChain(self, splineOrder=6, poseKnotsPerSecond=100, timeOffsetPadding=0.02):
        #use the main camera for the spline to initialize the poses
        return self.camList[0].initPoseSplineFromCamera(splineOrder, poseKnotsPerSecond, timeOffsetPadding)

    #find the timestamp for the first and last image considering all cameras in the chain
    def findCameraTimespan(self):
        tStart = acv.Time( 0.0 )
        tEnd = acv.Time( 0.0 )
        
        for cam in self.camList:
            if len(cam.targetObservations)>0:
                tStartCam = cam.targetObservations[0].time()
                tEndCam   = cam.targetObservations[-1].time()
                
                if tStart.toSec() > tStartCam.toSec():
                    tStart = tStartCam
                    
                if tEndCam.toSec() > tEnd.toSec():
                    tEnd = tEndCam
        
        self.timeStart = tStart
        self.timeStart = tEnd
         
    #find/set orientation prior between first camera in chain and main IMU (imu0) 
    def findOrientationPriorCameraChainToImu(self, imu):
        self.camList[0].findOrientationPriorCameraToImu( imu )

    #get an initial estimate of gravity in the world coordinate frame 
    def getEstimatedGravity(self):
        return self.camList[0].getEstimatedGravity()

    #return the baseline transformation from camA to camB
    def getResultBaseline(self, fromCamANr, toCamBNr):
        #transformation from cam a to b is always stored in the higer ID cam
        idx = np.max([fromCamANr, toCamBNr])
        
        #get the transform from camNrmin to camNrmax
        T_cB_cA = sm.Transformation( self.camList[idx].T_c_b_Dv.T() )
        
        #get the transformation direction right
        if fromCamANr > toCamBNr:
            T_cB_cA = T_cB_cA.inverse()
        
        #calculate the metric baseline
        baseline = np.linalg.norm( T_cB_cA.t() )
    
        return T_cB_cA, baseline
    
    def getResultTrafoImuToCam(self, camNr):
        #trafo from imu to cam0 in the chain
        T_c0_i = sm.Transformation( self.camList[0].T_c_b_Dv.T() )
        
        #add all the baselines along the chain up to our camera N
        T_cN_imu = T_c0_i
        
        #now add all baselines starting from the second camera up to the desired one
        for cam in self.camList[1:camNr+1]:
            T_cNplus1_cN = sm.Transformation( cam.T_c_b_Dv.T() )
            T_cN_imu = T_cNplus1_cN*T_cN_imu

        return T_cN_imu
    
    def getResultTimeShift(self, camNr):
        return self.camList[camNr].cameraTimeToImuTimeDv.toScalar() + self.camList[camNr].timeshiftCamToImuPrior
    
    def addDesignVariables(self, problem, noTimeCalibration = True, noChainExtrinsics = True):
        #add the design variables (T(R,t) & time)  for all induvidual cameras
        for camNr, cam in enumerate( self.camList ):
            #the first "baseline" dv is between the imu and cam0
            if camNr == 0:
                noExtrinsics = False
                baselinedv_group_id = CALIBRATION_GROUP_ID
            else:
                noExtrinsics = noChainExtrinsics
                baselinedv_group_id = HELPER_GROUP_ID
            cam.addDesignVariables(problem, noExtrinsics, noTimeCalibration, baselinedv_group_id=baselinedv_group_id)
    
    #add the reprojection error terms for all cameras in the chain
    def addCameraChainErrorTerms(self, problem, poseSplineDv, blakeZissermanDf=-1, timeOffsetPadding=0.0):
        
        #add the induviduak error terms for all cameras
        for camNr, cam in enumerate(self.camList):
            #add error terms for the first chain element
            if camNr == 0:
                #initialize the chain with first camerea ( imu to cam0)
                T_chain = cam.T_c_b_Dv.toExpression()
            else:
                T_chain = cam.T_c_b_Dv.toExpression() * T_chain
            
            #from imu coords to camerea N coords (as DVs)
            T_cN_b = T_chain
            
            #add the error terms
            cam.addCameraErrorTerms( problem, poseSplineDv, T_cN_b, blakeZissermanDf, timeOffsetPadding )

#IMU
class IccImu(object):
    
    class ImuParameters(kc.ImuParameters):
        def __init__(self, imuConfig, imuNr):
            kc.ImuParameters.__init__(self, '', True)
            self.data = imuConfig.data
            self.data["model"] = "calibrated"
            self.imuNr = imuNr

        def setImuPose(self, T_i_b):
            self.data["T_i_b"] = T_i_b.tolist()

        def setTimeOffset(self, time_offset):
            self.data["time_offset"] = time_offset

        def formatIndented(self, indent, np_array):
            return indent + str(np.array_str(np_array)).replace('\n',"\n"+indent)

        def printDetails(self, dest=sys.stdout):
            print("  Model: {0}".format(self.data["model"]), file=dest)
            kc.ImuParameters.printDetails(self, dest)
            print("  T_ib (imu0 to imu{0})".format(self.imuNr), file=dest)
            print(self.formatIndented("    ", np.array(self.data["T_i_b"])), file=dest)
            print("  time offset with respect to IMU0: {0} [s]".format(self.data["time_offset"]), file=dest)

    def getImuConfig(self):
        self.updateImuConfig()
        return self.imuConfig

    def updateImuConfig(self):
        self.imuConfig.setImuPose(self.getTransformationFromBodyToImu().T())
        self.imuConfig.setTimeOffset(self.timeOffset)

    def __init__(self, imuConfig, parsed, isReferenceImu=True, estimateTimedelay=True, imuNr=0):

        #determine whether IMU coincides with body frame (for multi-IMU setups)
        self.isReferenceImu = isReferenceImu
        self.estimateTimedelay = estimateTimedelay

        #store input
        self.imuConfig = self.ImuParameters(imuConfig, imuNr)

        #load dataset
        self.dataset = initImuBagDataset(parsed.bagfile[0], imuConfig.getRosTopic(), \
                                         parsed.bag_from_to, parsed.perform_synchronization)
        
        #statistics
        self.accelUncertaintyDiscrete, self.accelRandomWalk, self.accelUncertainty = self.imuConfig.getAccelerometerStatistics()
        self.gyroUncertaintyDiscrete, self.gyroRandomWalk, self.gyroUncertainty = self.imuConfig.getGyroStatistics()
        
        #init GyroBiasPrior (+ count for recursive averaging if we have more than 1 measurement = >1 cameras)
        self.GyroBiasPrior = np.array([0,0,0])
        self.GyroBiasPriorCount = 0
        
        #load the imu dataset
        native_runtime.timed_call(
            "imu_measurement_build", "problem_build",
            {"topic": self.dataset.topic}, self.loadImuData)

        #initial estimates for multi IMU calibration
        self.q_i_b_prior = np.array([0., 0., 0., 1.])
        self.r_b_prior = np.array([0., 0., 0.])
        self.timeOffset = 0.0
        self.AccelBiasPrior = np.array([0., 0., 0.])
        self.M_accel_prior = np.eye(3)
        self.M_gyro_prior = np.eye(3)
        self.C_gyro_i_prior = np.eye(3)
        self.A_gyro_accel_prior = np.zeros((3, 3))
        self.ry_i_prior = np.array([0., 0., 0.])
        self.rz_i_prior = np.array([0., 0., 0.])
        self.initializationStrategy = None
        self.initializationFields = set()
        self.hasGyroBiasInitialization = False
        self.hasTransformInitialization = False
        self.hasTimeInitialization = False

    def applyInitialization(self, initialization, strategy):
        if initialization is None:
            return

        imuName = 'imu{}'.format(self.imuConfig.imuNr)
        if self.isReferenceImu:
            for field in ('T_imu_from_reference',
                          'time_offset_to_reference_s'):
                if field in initialization:
                    raise ValueError(
                        "imus.{}.{} is only valid for non-reference IMUs".format(
                            imuName, field))

        intrinsicFields = {
            'M_accel', 'M_gyro', 'C_gyro_i', 'A_gyro_accel'
        }
        leverArmFields = {'ry_i_m', 'rz_i_m'}
        model = self.imuConfig.data['model']
        if intrinsicFields.intersection(initialization) and model not in (
                'scale-misalignment', 'scale-misalignment-size-effect'):
            raise ValueError(
                "Intrinsic initialization for {} requires a scale-misalignment "
                "IMU model".format(imuName))
        if leverArmFields.intersection(initialization) and model != (
                'scale-misalignment-size-effect'):
            raise ValueError(
                "Lever-arm initialization for {} requires the "
                "scale-misalignment-size-effect IMU model".format(imuName))

        self.initializationStrategy = strategy
        self.initializationFields = set(initialization)

        if 'gyroscope_bias_rad_s' in initialization:
            self.GyroBiasPrior = np.asarray(
                initialization['gyroscope_bias_rad_s'], dtype=float)
            self.hasGyroBiasInitialization = True
        if 'accelerometer_bias_m_s2' in initialization:
            self.AccelBiasPrior = np.asarray(
                initialization['accelerometer_bias_m_s2'], dtype=float)
        if 'T_imu_from_reference' in initialization:
            transform = np.asarray(
                initialization['T_imu_from_reference'], dtype=float)
            rotation = transform[0:3, 0:3]
            translation = transform[0:3, 3]
            self.q_i_b_prior = sm.r2quat(rotation)
            # r_b is the reference-frame lever arm used by the residuals;
            # T_i_b translation is -C_i_b * r_b.
            self.r_b_prior = -np.dot(rotation.transpose(), translation)
            self.hasTransformInitialization = True
        if 'time_offset_to_reference_s' in initialization:
            if strategy == 'refine' and not self.estimateTimedelay:
                raise ValueError(
                    "refine initialization of {} time offset requires "
                    "--imu-delay-by-correlation".format(imuName))
            self.timeOffset = float(
                initialization['time_offset_to_reference_s'])
            self.hasTimeInitialization = True
        if 'M_accel' in initialization:
            self.M_accel_prior = np.asarray(
                initialization['M_accel'], dtype=float)
        if 'M_gyro' in initialization:
            self.M_gyro_prior = np.asarray(
                initialization['M_gyro'], dtype=float)
        if 'C_gyro_i' in initialization:
            self.C_gyro_i_prior = np.asarray(
                initialization['C_gyro_i'], dtype=float)
        if 'A_gyro_accel' in initialization:
            self.A_gyro_accel_prior = np.asarray(
                initialization['A_gyro_accel'], dtype=float)
        if 'ry_i_m' in initialization:
            self.ry_i_prior = np.asarray(initialization['ry_i_m'], dtype=float)
        if 'rz_i_m' in initialization:
            self.rz_i_prior = np.asarray(initialization['rz_i_m'], dtype=float)
    class ImuMeasurement(object):
        def __init__(self, stamp, omega, alpha, Rgyro, Raccel):
            self.omega = omega
            self.alpha = alpha
            self.omegaR = Rgyro
            self.omegaInvR = np.linalg.inv(Rgyro)
            self.alphaR = Raccel
            self.alphaInvR = np.linalg.inv(Raccel)
            self.stamp = stamp
        
    def loadImuData(self):
        print("Reading IMU data ({0})".format(self.dataset.topic))
            
        # prepare progess bar
        iProgress = sm.Progress2( self.dataset.numMessages() )
        iProgress.sample()
        
        Rgyro = np.eye(3) * self.gyroUncertaintyDiscrete * self.gyroUncertaintyDiscrete
        Raccel = np.eye(3) * self.accelUncertaintyDiscrete * self.accelUncertaintyDiscrete
        
        # Now read the imu measurements.
        imu = []
        for timestamp, omega, alpha in self.dataset:
            timestamp = acv.Time( timestamp.toSec() ) 
            imu.append( self.ImuMeasurement(timestamp, omega, alpha, Rgyro, Raccel) )
            iProgress.sample()
        
        self.imuData = imu
        
        if len(self.imuData)>1:
            print("\r  Read %d imu readings over %.1f seconds                   " \
                    % (len(imu), imu[-1].stamp.toSec() - imu[0].stamp.toSec()))
        else:
            sm.logFatal("Could not find any IMU messages. Please check the dataset.")
            sys.exit(-1)
            
            
    def addDesignVariables(self, problem):
        #create design variables
        self.gyroBiasDv = asp.EuclideanBSplineDesignVariable( self.gyroBias )
        self.accelBiasDv = asp.EuclideanBSplineDesignVariable( self.accelBias )
        
        addSplineDesignVariables(problem, self.gyroBiasDv, setActive=True,
                                    group_id=HELPER_GROUP_ID)
        addSplineDesignVariables(problem, self.accelBiasDv, setActive=True,
                                    group_id=HELPER_GROUP_ID)

        self.q_i_b_Dv = aopt.RotationQuaternionDv(self.q_i_b_prior)
        problem.addDesignVariable(self.q_i_b_Dv, HELPER_GROUP_ID)
        self.q_i_b_Dv.setActive(False)
        self.r_b_Dv = aopt.EuclideanPointDv(self.r_b_prior)
        problem.addDesignVariable(self.r_b_Dv, HELPER_GROUP_ID)
        self.r_b_Dv.setActive(False)

        if not self.isReferenceImu:
            self.q_i_b_Dv.setActive(True)
            self.r_b_Dv.setActive(True)

    def addAccelerometerErrorTerms(self, problem, poseSplineDv, g_w, mSigma=0.0, \
                                   accelNoiseScale=1.0):
        print("")
        print("Adding accelerometer error terms ({0})".format(self.dataset.topic))
        
        #progress bar
        iProgress = sm.Progress2( len(self.imuData) )
        iProgress.sample()
        
        # AccelerometerError(measurement,  invR,  C_b_w,  acceleration_w,  bias,  g_w)
        weight = 1.0/accelNoiseScale
        accelErrors = []
        num_skipped = 0
        
        if mSigma > 0.0:
            mest = aopt.HuberMEstimator(mSigma)
        else:
            mest = aopt.NoMEstimator()
            
        for im in self.imuData:
            tk = im.stamp.toSec() + self.timeOffset
            if tk > poseSplineDv.spline().t_min() and tk < poseSplineDv.spline().t_max():
                C_b_w = poseSplineDv.orientation(tk).inverse()
                a_w = poseSplineDv.linearAcceleration(tk)
                b_i = self.accelBiasDv.toEuclideanExpression(tk,0)
                w_b = poseSplineDv.angularVelocityBodyFrame(tk)
                w_dot_b = poseSplineDv.angularAccelerationBodyFrame(tk)
                C_i_b = self.q_i_b_Dv.toExpression()
                r_b = self.r_b_Dv.toExpression()
                a = C_i_b * (C_b_w * (a_w - g_w) + \
                             w_dot_b.cross(r_b) + w_b.cross(w_b.cross(r_b)))
                aerr = ket.EuclideanError(im.alpha, im.alphaInvR * weight, a + b_i)
                aerr.setMEstimatorPolicy(mest)
                accelErrors.append(aerr)
                problem.addErrorTerm(aerr)
            else:
                num_skipped = num_skipped + 1

            #update progress bar
            iProgress.sample()

        print("\r  Added {0} of {1} accelerometer error terms (skipped {2} out-of-bounds measurements)".format( len(self.imuData)-num_skipped, len(self.imuData), num_skipped ))
        self.accelErrors = accelErrors

    def addGyroscopeErrorTerms(self, problem, poseSplineDv, mSigma=0.0, gyroNoiseScale=1.0, \
                               g_w=None):
        print("")
        print("Adding gyroscope error terms ({0})".format(self.dataset.topic))
        
        #progress bar
        iProgress = sm.Progress2( len(self.imuData) )
        iProgress.sample()

        num_skipped = 0
        gyroErrors = []
        weight = 1.0/gyroNoiseScale
        if mSigma > 0.0:
            mest = aopt.HuberMEstimator(mSigma)
        else:
            mest = aopt.NoMEstimator()
            
        for im in self.imuData:
            tk = im.stamp.toSec() + self.timeOffset
            if tk > poseSplineDv.spline().t_min() and tk < poseSplineDv.spline().t_max():
                # GyroscopeError(measurement, invR, angularVelocity, bias)
                w_b = poseSplineDv.angularVelocityBodyFrame(tk)
                b_i = self.gyroBiasDv.toEuclideanExpression(tk,0)
                C_i_b = self.q_i_b_Dv.toExpression()
                w = C_i_b * w_b
                gerr = ket.EuclideanError(im.omega, im.omegaInvR * weight, w + b_i)
                gerr.setMEstimatorPolicy(mest)
                gyroErrors.append(gerr)
                problem.addErrorTerm(gerr)
            else:
                num_skipped = num_skipped + 1

            #update progress bar
            iProgress.sample()

        print("\r  Added {0} of {1} gyroscope error terms (skipped {2} out-of-bounds measurements)".format( len(self.imuData)-num_skipped, len(self.imuData), num_skipped ))           
        self.gyroErrors = gyroErrors

    def initBiasSplines(self, poseSpline, splineOrder, biasKnotsPerSecond):
        start = poseSpline.t_min();
        end = poseSpline.t_max();
        seconds = end - start;
        knots = int(round(seconds * biasKnotsPerSecond))
        
        print("")
        print("Initializing the bias splines with %d knots" % (knots))
        
        #initialize the bias splines
        self.gyroBias = bsplines.BSpline(splineOrder)
        self.gyroBias.initConstantSpline(start,end,knots, self.GyroBiasPrior )
        
        self.accelBias = bsplines.BSpline(splineOrder)
        self.accelBias.initConstantSpline(start,end,knots, self.AccelBiasPrior)
        
    def addBiasMotionTerms(self, problem):
        Wgyro = np.eye(3) / (self.gyroRandomWalk * self.gyroRandomWalk)
        Waccel =  np.eye(3) / (self.accelRandomWalk * self.accelRandomWalk)
        gyroBiasMotionErr = asp.BSplineEuclideanMotionError(self.gyroBiasDv, Wgyro, 1)
        problem.addErrorTerm(gyroBiasMotionErr)
        accelBiasMotionErr = asp.BSplineEuclideanMotionError(self.accelBiasDv, Waccel, 1)
        problem.addErrorTerm(accelBiasMotionErr)
        
    def getTransformationFromBodyToImu(self):
        if self.isReferenceImu:
            return sm.Transformation()
        return sm.Transformation(sm.r2quat(self.q_i_b_Dv.toRotationMatrix()) , \
                                 - np.dot(self.q_i_b_Dv.toRotationMatrix(), \
                                          self.r_b_Dv.toEuclidean()))

    def _referenceAngularVelocitySplineBounds(self, referenceImu):
        # The spline models the reference IMU.  Preserve the legacy domain
        # when no time seed exists, but use the actual reference clock domain
        # for seeded alignment so a valid large offset can create overlap.
        sourceImu = referenceImu if self.hasTimeInitialization else self
        return (sourceImu.imuData[0].stamp.toSec(),
                sourceImu.imuData[-1].stamp.toSec())

    def findOrientationPrior(self, referenceImu):
        print("")
        print("Estimating imu-imu rotation initial guess.")
        
        # build the problem
        problem = aopt.OptimizationProblem()

        # Add the relative rotation as design variable.
        q_i_b_Dv = aopt.RotationQuaternionDv(self.q_i_b_prior)
        rotationActive = not (
            self.initializationStrategy == 'direct' and
            self.hasTransformInitialization)
        q_i_b_Dv.setActive(rotationActive)
        problem.addDesignVariable(q_i_b_Dv)

        # Add spline representing rotational velocity of in body frame
        startTime, endTime = self._referenceAngularVelocitySplineBounds(
            referenceImu)
        knotsPerSecond = 50
        knots = int( round( (endTime - startTime) * knotsPerSecond) )

        angularVelocity = bsplines.BSpline(3)
        angularVelocity.initConstantSpline(startTime, endTime, knots, np.array([0., 0., 0.]) )
        angularVelocityDv = asp.EuclideanBSplineDesignVariable(angularVelocity)

        for i in range(0,angularVelocityDv.numDesignVariables()):
            dv = angularVelocityDv.designVariable(i)
            dv.setActive(True)
            problem.addDesignVariable(dv)

        # Add constant reference gyro bias as design variable
        referenceGyroBiasPrior = (
            referenceImu.GyroBiasPrior
            if getattr(referenceImu, 'hasGyroBiasInitialization', False)
            else np.zeros(3))
        referenceGyroBiasDv = aopt.EuclideanPointDv(
            referenceGyroBiasPrior)
        referenceBiasActive = not (
            getattr(referenceImu, 'initializationStrategy', None) == 'direct'
            and getattr(
                referenceImu, 'hasGyroBiasInitialization', False))
        referenceGyroBiasDv.setActive(referenceBiasActive)
        problem.addDesignVariable(referenceGyroBiasDv)

        for im in referenceImu.imuData:
            tk = im.stamp.toSec()
            if tk > angularVelocity.t_min() and tk < angularVelocity.t_max():        
                #DV expressions
                bias = referenceGyroBiasDv.toExpression()   
                
                omega_predicted = angularVelocityDv.toEuclideanExpression(tk, 0)
                omega_measured = im.omega
                
                #error term
                gerr = ket.GyroscopeError(im.omega, im.omegaInvR, omega_predicted, bias)
                problem.addErrorTerm(gerr)
            
        #define the optimization 
        options = aopt.Optimizer2Options()
        options.verbose = False
        options.linearSolver = aopt.BlockCholeskyLinearSystemSolver() #does not have multi-threading support
        options.nThreads = 2
        options.convergenceDeltaX = 1e-4
        options.convergenceDeltaJ = 1
        options.maxIterations = 50

        #run the optimization
        native_runtime.apply_optimizer_threads(options)
        optimizer = aopt.Optimizer2(options)
        optimizer.setProblem(problem)
        
        try:
            native_runtime.run_optimizer(optimizer)
        except:
            sm.logFatal("Failed to obtain initial guess for the relative orientation!")
            sys.exit(-1)

        if getattr(referenceImu, 'hasGyroBiasInitialization', False):
            referenceImu.GyroBiasPrior = referenceGyroBiasDv.toEuclidean()

        initialTimeOffset = self.timeOffset
        referenceAbsoluteOmega = lambda dt = np.array([0.]): \
                np.asarray([np.linalg.norm(angularVelocityDv.toEuclidean(im.stamp.toSec() + initialTimeOffset + dt[0], 0)) \
                            for im in self.imuData \
                            if (im.stamp.toSec() + initialTimeOffset + dt[0] > angularVelocity.t_min() \
                                and im.stamp.toSec() + initialTimeOffset + dt[0] < angularVelocity.t_max())])
        absoluteOmega = lambda dt = np.array([0.]): \
                np.asarray([np.linalg.norm(im.omega) for im in self.imuData \
                            if (im.stamp.toSec() + initialTimeOffset + dt[0] > angularVelocity.t_min() \
                                and im.stamp.toSec() + initialTimeOffset + dt[0] < angularVelocity.t_max())])

        if len(referenceAbsoluteOmega()) == 0 or len(absoluteOmega()) == 0:
            sm.logFatal("The time ranges of the IMUs published as topics {0} and {1} do not overlap. "\
                        "Please make sure that the sensors are synchronized correctly." \
                        .format(referenceImu.imuConfig.getRosTopic(), self.imuConfig.getRosTopic()))
            sys.exit(-1)
         
        directTime = (
            self.hasTimeInitialization and
            self.initializationStrategy == 'direct')
        if not directTime:
            #get the (residual, for refine initialization) time shift
            corr = np.correlate(
                referenceAbsoluteOmega(), absoluteOmega(), "full")
            discrete_shift = corr.argmax() - (np.size(absoluteOmega()) - 1)
            times = [im.stamp.toSec() for im in self.imuData]
            dT = np.mean(np.diff( times ))
            shift = discrete_shift*dT

            if self.estimateTimedelay and not self.isReferenceImu:
                objectiveFunction = lambda dt: np.linalg.norm(
                    referenceAbsoluteOmega(dt) - absoluteOmega(dt))**2
                refined_shift = scipy.optimize.fmin(
                    objectiveFunction, np.array([shift]), maxiter=100)[0]
                if (self.hasTimeInitialization and
                        self.initializationStrategy == 'refine'):
                    self.timeOffset = initialTimeOffset + float(refined_shift)
                else:
                    self.timeOffset = float(refined_shift)

        print("Temporal correction with respect to reference IMU ")
        print(self.timeOffset, "[s]", ("" if (
            self.estimateTimedelay or self.hasTimeInitialization) else \
                                       " (this offset is not accounted for in the calibration)"))

        # Add constant gyro bias as design variable
        gyroBiasDv = aopt.EuclideanPointDv(self.GyroBiasPrior)
        biasActive = not (
            self.initializationStrategy == 'direct' and
            self.hasGyroBiasInitialization)
        gyroBiasDv.setActive(biasActive)
        problem.addDesignVariable(gyroBiasDv)

        for im in self.imuData:
            tk = im.stamp.toSec() + self.timeOffset
            if tk > angularVelocity.t_min() and tk < angularVelocity.t_max():        
                #DV expressions
                C_i_b = q_i_b_Dv.toExpression()
                bias = gyroBiasDv.toExpression()   
                
                omega_predicted = C_i_b * angularVelocityDv.toEuclideanExpression(tk, 0)
                omega_measured = im.omega
                
                #error term
                gerr = ket.GyroscopeError(im.omega, im.omegaInvR, omega_predicted, bias)
                problem.addErrorTerm(gerr)

        #get the prior unless direct initialization supplied both coupled values
        if rotationActive or biasActive:
            try:
                native_runtime.run_optimizer(optimizer)
            except:
                sm.logFatal("Failed to obtain initial guess for the relative orientation!")
                sys.exit(-1)

        print("Estimated imu to reference imu Rotation: ")
        print(q_i_b_Dv.toRotationMatrix())

        self.q_i_b_prior = sm.r2quat(q_i_b_Dv.toRotationMatrix())
        if self.hasTransformInitialization or self.hasGyroBiasInitialization:
            self.GyroBiasPrior = gyroBiasDv.toEuclidean()
        
        
class IccScaledMisalignedImu(IccImu):

    class ImuParameters(IccImu.ImuParameters):
        def __init__(self, imuConfig, imuNr):
            IccImu.ImuParameters.__init__(self, imuConfig, imuNr)
            self.data = imuConfig.data
            self.data["model"] = "scale-misalignment"

        def printDetails(self, dest=sys.stdout):
            IccImu.ImuParameters.printDetails(self, dest)
            print("  Gyroscope: ", file=dest)
            print("    M:", file=dest)
            print(self.formatIndented("      ", np.array(self.data["gyroscopes"]["M"])), file=dest)
            print("    A [(rad/s)/(m/s^2)]:", file=dest)
            print(self.formatIndented("      ", np.array(self.data["gyroscopes"]["A"])), file=dest)
            print("    C_gyro_i:", file=dest)
            print(self.formatIndented("      ", np.array(self.data["gyroscopes"]["C_gyro_i"])), file=dest)
            print("  Accelerometer: ", file=dest)
            print("    M:", file=dest)
            print(self.formatIndented("      ", np.array(self.data["accelerometers"]["M"])), file=dest)

        def setIntrisicsMatrices(self, M_accel, C_gyro_i, M_gyro, Ma_gyro):
            self.data["accelerometers"] = dict()
            self.data["accelerometers"]["M"] = M_accel.tolist()
            self.data["gyroscopes"] = dict()
            self.data["gyroscopes"]["M"] = M_gyro.tolist()
            self.data["gyroscopes"]["A"] = Ma_gyro.tolist()
            self.data["gyroscopes"]["C_gyro_i"] = C_gyro_i.tolist()

    def updateImuConfig(self):
        IccImu.updateImuConfig(self)
        self.imuConfig.setIntrisicsMatrices(self.M_accel_Dv.toMatrix3x3(), \
                                            self.q_gyro_i_Dv.toRotationMatrix(), \
                                            self.M_gyro_Dv.toMatrix3x3(), \
                                            self.M_accel_gyro_Dv.toMatrix3x3())
        
    def addDesignVariables(self, problem):
        IccImu.addDesignVariables(self, problem)

        self.q_gyro_i_Dv = aopt.RotationQuaternionDv(
            sm.r2quat(self.C_gyro_i_prior))
        problem.addDesignVariable(self.q_gyro_i_Dv, HELPER_GROUP_ID)
        self.q_gyro_i_Dv.setActive(True)

        self.M_accel_Dv = aopt.MatrixBasicDv(self.M_accel_prior, np.array([[1, 0, 0],[1, 1, 0],[1, 1, 1]], \
                                                                 dtype=int))
        problem.addDesignVariable(self.M_accel_Dv, HELPER_GROUP_ID)
        self.M_accel_Dv.setActive(True)
        
        self.M_gyro_Dv = aopt.MatrixBasicDv(self.M_gyro_prior, np.array([[1, 0, 0],[1, 1, 0],[1, 1, 1]], \
                                                                dtype=int))
        problem.addDesignVariable(self.M_gyro_Dv, HELPER_GROUP_ID)
        self.M_gyro_Dv.setActive(True)
        
        self.M_accel_gyro_Dv = aopt.MatrixBasicDv(
            self.A_gyro_accel_prior, np.ones((3,3),dtype=int))
        problem.addDesignVariable(self.M_accel_gyro_Dv, HELPER_GROUP_ID)
        self.M_accel_gyro_Dv.setActive(True)

    def addAccelerometerErrorTerms(self, problem, poseSplineDv, g_w, mSigma=0.0, \
                                   accelNoiseScale=1.0):
        print("")
        print("Adding accelerometer error terms ({0})".format(self.dataset.topic))
        
        #progress bar
        iProgress = sm.Progress2( len(self.imuData) )
        iProgress.sample()
        
        # AccelerometerError(measurement,  invR,  C_b_w,  acceleration_w,  bias,  g_w)
        weight = 1.0/accelNoiseScale
        accelErrors = []
        num_skipped = 0
        
        if mSigma > 0.0:
            mest = aopt.HuberMEstimator(mSigma)
        else:
            mest = aopt.NoMEstimator()
            
        for im in self.imuData:
            tk = im.stamp.toSec() + self.timeOffset
            if tk > poseSplineDv.spline().t_min() and tk < poseSplineDv.spline().t_max():
                C_b_w = poseSplineDv.orientation(tk).inverse()
                a_w = poseSplineDv.linearAcceleration(tk)
                b_i = self.accelBiasDv.toEuclideanExpression(tk,0)
                M = self.M_accel_Dv.toExpression()
                w_b = poseSplineDv.angularVelocityBodyFrame(tk)
                w_dot_b = poseSplineDv.angularAccelerationBodyFrame(tk)
                C_i_b = self.q_i_b_Dv.toExpression()
                r_b = self.r_b_Dv.toExpression()
                a = M * (C_i_b * (C_b_w * (a_w - g_w) + \
                                  w_dot_b.cross(r_b) + w_b.cross(w_b.cross(r_b))))

                aerr = ket.EuclideanError(im.alpha, im.alphaInvR * weight, a + b_i)
                aerr.setMEstimatorPolicy(mest)
                accelErrors.append(aerr)
                problem.addErrorTerm(aerr)
            else:
                num_skipped = num_skipped + 1

            #update progress bar
            iProgress.sample()

        print("\r  Added {0} of {1} accelerometer error terms (skipped {2} out-of-bounds measurements)".format( len(self.imuData)-num_skipped, len(self.imuData), num_skipped ))
        self.accelErrors = accelErrors

    def addGyroscopeErrorTerms(self, problem, poseSplineDv, mSigma=0.0, gyroNoiseScale=1.0, g_w=None):
        print("")
        print("Adding gyroscope error terms ({0})".format(self.dataset.topic))
        
        #progress bar
        iProgress = sm.Progress2( len(self.imuData) )
        iProgress.sample()

        num_skipped = 0
        gyroErrors = []
        weight = 1.0/gyroNoiseScale
        if mSigma > 0.0:
            mest = aopt.HuberMEstimator(mSigma)
        else:
            mest = aopt.NoMEstimator()
            
        for im in self.imuData:
            tk = im.stamp.toSec() + self.timeOffset
            if tk > poseSplineDv.spline().t_min() and tk < poseSplineDv.spline().t_max():
                # GyroscopeError(measurement, invR, angularVelocity, bias)
                w_b = poseSplineDv.angularVelocityBodyFrame(tk)
                w_dot_b = poseSplineDv.angularAccelerationBodyFrame(tk)
                b_i = self.gyroBiasDv.toEuclideanExpression(tk,0)
                C_b_w = poseSplineDv.orientation(tk).inverse()
                a_w = poseSplineDv.linearAcceleration(tk)
                r_b = self.r_b_Dv.toExpression()
                a_b = C_b_w * (a_w - g_w) + w_dot_b.cross(r_b) + w_b.cross(w_b.cross(r_b))

                C_i_b = self.q_i_b_Dv.toExpression()
                C_gyro_i = self.q_gyro_i_Dv.toExpression()
                C_gyro_b = C_gyro_i * C_i_b
                M = self.M_gyro_Dv.toExpression()
                Ma = self.M_accel_gyro_Dv.toExpression()

                w = M * (C_gyro_b * w_b) + Ma * (C_gyro_b * a_b)

                gerr = ket.EuclideanError(im.omega, im.omegaInvR * weight, w + b_i)
                gerr.setMEstimatorPolicy(mest)
                gyroErrors.append(gerr)
                problem.addErrorTerm(gerr)
            else:
                num_skipped = num_skipped + 1

            #update progress bar
            iProgress.sample()

        print("\r  Added {0} of {1} gyroscope error terms (skipped {2} out-of-bounds measurements)".format( len(self.imuData)-num_skipped, len(self.imuData), num_skipped ))           
        self.gyroErrors = gyroErrors

class IccScaledMisalignedSizeEffectImu(IccScaledMisalignedImu):

    class ImuParameters(IccScaledMisalignedImu.ImuParameters):
        def __init__(self, imuConfig, imuNr):
            IccScaledMisalignedImu.ImuParameters.__init__(self, imuConfig, imuNr)
            self.data = imuConfig.data
            self.data["model"] = "scale-misalignment-size-effect"

        def printDetails(self, dest=sys.stdout):
            IccScaledMisalignedImu.ImuParameters.printDetails(self, dest)
            print("    rx_i [m]:", file=dest)
            print(self.formatIndented("      ", \
                                               np.array(self.data["accelerometers"]["rx_i"])), file=dest)
            print("    ry_i [m]:", file=dest)
            print(self.formatIndented("      ", \
                                               np.array(self.data["accelerometers"]["ry_i"])), file=dest)
            print("    rz_i [m]:", file=dest)
            print(self.formatIndented("      ", \
                                               np.array(self.data["accelerometers"]["rz_i"])), file=dest)

        def setAccelerometerLeverArms(self, rx_i, ry_i, rz_i):
            self.data["accelerometers"]["rx_i"] = rx_i.tolist()
            self.data["accelerometers"]["ry_i"] = ry_i.tolist()
            self.data["accelerometers"]["rz_i"] = rz_i.tolist()

    def updateImuConfig(self):
        IccScaledMisalignedImu.updateImuConfig(self)
        self.imuConfig.setAccelerometerLeverArms(self.rx_i_Dv.toEuclidean(), \
                                                 self.ry_i_Dv.toEuclidean(), \
                                                 self.rz_i_Dv.toEuclidean())

    def addDesignVariables(self, problem):
        IccScaledMisalignedImu.addDesignVariables(self, problem)

        self.rx_i_Dv = aopt.EuclideanPointDv(np.array([0., 0., 0.]))
        problem.addDesignVariable(self.rx_i_Dv, HELPER_GROUP_ID)
        self.rx_i_Dv.setActive(False)
        
        self.ry_i_Dv = aopt.EuclideanPointDv(self.ry_i_prior)
        problem.addDesignVariable(self.ry_i_Dv, HELPER_GROUP_ID)
        self.ry_i_Dv.setActive(True)

        self.rz_i_Dv = aopt.EuclideanPointDv(self.rz_i_prior)
        problem.addDesignVariable(self.rz_i_Dv, HELPER_GROUP_ID)
        self.rz_i_Dv.setActive(True)

        self.Ix_Dv = aopt.MatrixBasicDv(np.diag([1.,0.,0.]), np.zeros((3,3),dtype=int))
        problem.addDesignVariable(self.Ix_Dv, HELPER_GROUP_ID)
        self.Ix_Dv.setActive(False)
        self.Iy_Dv = aopt.MatrixBasicDv(np.diag([0.,1.,0.]), np.zeros((3,3),dtype=int))
        problem.addDesignVariable(self.Iy_Dv, HELPER_GROUP_ID)
        self.Iy_Dv.setActive(False)
        self.Iz_Dv = aopt.MatrixBasicDv(np.diag([0.,0.,1.]), np.zeros((3,3),dtype=int))
        problem.addDesignVariable(self.Iz_Dv, HELPER_GROUP_ID)
        self.Iz_Dv.setActive(False)

    def addAccelerometerErrorTerms(self, problem, poseSplineDv, g_w, mSigma=0.0, \
                                   accelNoiseScale=1.0):
        print("")
        print("Adding accelerometer error terms ({0})".format(self.dataset.topic))
        
        #progress bar
        iProgress = sm.Progress2( len(self.imuData) )
        iProgress.sample()
        
        # AccelerometerError(measurement,  invR,  C_b_w,  acceleration_w,  bias,  g_w)
        weight = 1.0/accelNoiseScale
        accelErrors = []
        num_skipped = 0
        
        if mSigma > 0.0:
            mest = aopt.HuberMEstimator(mSigma)
        else:
            mest = aopt.NoMEstimator()
            
        for im in self.imuData:
            tk = im.stamp.toSec() + self.timeOffset
            if tk > poseSplineDv.spline().t_min() and tk < poseSplineDv.spline().t_max():
                C_b_w = poseSplineDv.orientation(tk).inverse()
                a_w = poseSplineDv.linearAcceleration(tk)
                b_i = self.accelBiasDv.toEuclideanExpression(tk,0)
                M = self.M_accel_Dv.toExpression()
                w_b = poseSplineDv.angularVelocityBodyFrame(tk)
                w_dot_b = poseSplineDv.angularAccelerationBodyFrame(tk)
                C_i_b = self.q_i_b_Dv.toExpression()
                rx_b = self.r_b_Dv.toExpression() + C_i_b.inverse() * self.rx_i_Dv.toExpression()
                ry_b = self.r_b_Dv.toExpression() + C_i_b.inverse() * self.ry_i_Dv.toExpression()
                rz_b = self.r_b_Dv.toExpression() + C_i_b.inverse() * self.rz_i_Dv.toExpression()
                Ix = self.Ix_Dv.toExpression()
                Iy = self.Iy_Dv.toExpression()
                Iz = self.Iz_Dv.toExpression()
                
                a = M * (C_i_b * (C_b_w * (a_w - g_w)) + \
                         Ix * (C_i_b * (w_dot_b.cross(rx_b) + w_b.cross(w_b.cross(rx_b)))) + \
                         Iy * (C_i_b * (w_dot_b.cross(ry_b) + w_b.cross(w_b.cross(ry_b)))) + \
                         Iz * (C_i_b * (w_dot_b.cross(rz_b) + w_b.cross(w_b.cross(rz_b)))) )

                aerr = ket.EuclideanError(im.alpha, im.alphaInvR * weight, a + b_i)
                aerr.setMEstimatorPolicy(mest)
                accelErrors.append(aerr)
                problem.addErrorTerm(aerr)
            else:
                num_skipped = num_skipped + 1

            #update progress bar
            iProgress.sample()

        print("\r  Added {0} of {1} accelerometer error terms (skipped {2} out-of-bounds measurements)".format( len(self.imuData)-num_skipped, len(self.imuData), num_skipped ))
        self.accelErrors = accelErrors
