from __future__ import print_function #handle print in 2.x python
import kalibr_native_optimizer as native_runtime
# kalibr-native-source-overlay
import sm
from sm import PlotCollection
from kalibr_common import ConfigReader as cr
import aslam_cv as acv
import aslam_cameras_april as acv_april
import aslam_cv_backend as acvb
import aslam_backend as aopt
import incremental_calibration as ic
import kalibr_camera_calibration as kcc

from matplotlib.backends.backend_pdf import PdfPages
import mpl_toolkits.mplot3d.axes3d as p3
import cv2
import numpy as np
import pylab as pl
import math
import gc
import sys
import yaml
import numbers

np.set_printoptions(suppress=True, precision=8)

#DV group IDs
CALIBRATION_GROUP_ID = 0
TRANSFORMATION_GROUP_ID = 1
LANDMARK_GROUP_ID = 2

class OptimizationDiverged(Exception):
    pass


CAMERA_CALIBRATION_INITIALIZATION_KIND = (
    'camera_calibration_initialization'
)
CAMERA_CALIBRATION_INITIALIZATION_SCHEMA_VERSION = 1


def _initialization_error(filename, message):
    raise RuntimeError(
        'Invalid camera calibration initialization file "{0}": {1}'.format(
            filename, message))


def _initialization_vector(filename, camera_name, field_name, value):
    if not isinstance(value, (list, tuple)):
        _initialization_error(
            filename, '{0}.{1} must be a list'.format(
                camera_name, field_name))

    if any(isinstance(item, bool) or not isinstance(item, numbers.Real)
           for item in value):
        _initialization_error(
            filename, '{0}.{1} must contain only numbers'.format(
                camera_name, field_name))
    vector = np.asarray(value, dtype=float)

    if vector.ndim != 1:
        _initialization_error(
            filename, '{0}.{1} must be a one-dimensional list'.format(
                camera_name, field_name))
    if not np.all(np.isfinite(vector)):
        _initialization_error(
            filename, '{0}.{1} must contain only finite numbers'.format(
                camera_name, field_name))
    return tuple(float(item) for item in vector)


def _initialization_transform(filename, camera_name, value):
    try:
        raw_transform = np.asarray(value, dtype=object)
    except (TypeError, ValueError):
        _initialization_error(
            filename, '{0}.T_cam_from_previous must contain only numbers'.format(
                camera_name))

    if raw_transform.shape != (4, 4):
        _initialization_error(
            filename,
            '{0}.T_cam_from_previous must be a full 4x4 matrix'.format(
                camera_name))
    if any(isinstance(item, bool) or not isinstance(item, numbers.Real)
           for item in raw_transform.flat):
        _initialization_error(
            filename, '{0}.T_cam_from_previous must contain only numbers'.format(
                camera_name))
    transform = np.asarray(raw_transform, dtype=float)
    if not np.all(np.isfinite(transform)):
        _initialization_error(
            filename,
            '{0}.T_cam_from_previous must contain only finite numbers'.format(
                camera_name))
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0],
                       rtol=0.0, atol=1e-9):
        _initialization_error(
            filename,
            '{0}.T_cam_from_previous must have homogeneous last row '
            '[0, 0, 0, 1]'.format(camera_name))

    rotation = transform[0:3, 0:3]
    if (not np.allclose(np.dot(rotation.T, rotation), np.eye(3),
                        rtol=0.0, atol=1e-6) or
            not np.isclose(np.linalg.det(rotation), 1.0,
                           rtol=0.0, atol=1e-6)):
        _initialization_error(
            filename,
            '{0}.T_cam_from_previous rotation must be orthonormal with '
            'determinant +1'.format(camera_name))

    # Tuples keep the parsed seed independent from matrices mutated by native
    # optimizers.  A fresh sm.Transformation is made for every attempt below.
    return tuple(tuple(float(item) for item in row) for row in transform)


def loadCameraCalibrationInitialization(filename, num_cameras,
                                        camera_models=None):
    """Load and defensively validate the legacy camera seed document.

    Parameter-vector lengths depend on the selected camera model and are
    checked when the seed is applied to a CameraGeometry.
    """
    try:
        with open(filename, 'r') as stream:
            document = yaml.safe_load(stream)
    except (IOError, OSError) as error:
        raise RuntimeError(
            'Could not read camera calibration initialization file "{0}": '
            '{1}'.format(filename, error))
    except yaml.YAMLError as error:
        raise RuntimeError(
            'Could not parse camera calibration initialization file "{0}": '
            '{1}'.format(filename, error))

    if not isinstance(document, dict):
        _initialization_error(filename, 'the document root must be a mapping')

    allowed_top_level = set([
        'schema_version', 'kind', 'strategy', 'cameras'])
    unknown = set(document.keys()) - allowed_top_level
    if unknown:
        _initialization_error(
            filename, 'unknown top-level field(s): {0}'.format(
                ', '.join(sorted(str(item) for item in unknown))))
    missing = allowed_top_level - set(document.keys())
    if missing:
        _initialization_error(
            filename, 'missing top-level field(s): {0}'.format(
                ', '.join(sorted(missing))))

    schema_version = document['schema_version']
    if (type(schema_version) is not int or
            schema_version != CAMERA_CALIBRATION_INITIALIZATION_SCHEMA_VERSION):
        _initialization_error(
            filename, 'schema_version must be integer {0}'.format(
                CAMERA_CALIBRATION_INITIALIZATION_SCHEMA_VERSION))
    if document['kind'] != CAMERA_CALIBRATION_INITIALIZATION_KIND:
        _initialization_error(
            filename, 'kind must be "{0}"'.format(
                CAMERA_CALIBRATION_INITIALIZATION_KIND))

    strategy = document['strategy']
    if strategy not in ('refine', 'direct'):
        _initialization_error(
            filename, 'strategy must be either "refine" or "direct"')

    cameras = document['cameras']
    if not isinstance(cameras, dict):
        _initialization_error(filename, 'cameras must be a mapping')

    expected_names = set(
        'cam{0}'.format(index) for index in range(num_cameras))
    unknown_cameras = set(cameras.keys()) - expected_names
    if unknown_cameras:
        _initialization_error(
            filename, 'camera key(s) do not match this calibration: {0}'.format(
                ', '.join(sorted(str(item) for item in unknown_cameras))))

    normalized_cameras = dict()
    allowed_camera_fields = set([
        'intrinsics', 'distortion_coeffs', 'T_cam_from_previous'])
    for camera_name, block in cameras.items():
        if not isinstance(block, dict):
            _initialization_error(
                filename, '{0} must be a mapping'.format(camera_name))
        unknown_fields = set(block.keys()) - allowed_camera_fields
        if unknown_fields:
            _initialization_error(
                filename, '{0} has unknown field(s): {1}'.format(
                    camera_name,
                    ', '.join(sorted(str(item) for item in unknown_fields))))

        normalized = dict()
        for field_name in ('intrinsics', 'distortion_coeffs'):
            if field_name in block:
                normalized[field_name] = _initialization_vector(
                    filename, camera_name, field_name, block[field_name])
        if 'T_cam_from_previous' in block:
            if camera_name == 'cam0':
                _initialization_error(
                    filename, 'cam0 cannot define T_cam_from_previous')
            normalized['T_cam_from_previous'] = _initialization_transform(
                filename, camera_name, block['T_cam_from_previous'])
        normalized_cameras[camera_name] = normalized

    if strategy == 'direct':
        incomplete = list()
        for camera_index in range(num_cameras):
            camera_name = 'cam{0}'.format(camera_index)
            block = normalized_cameras.get(camera_name, {})
            for field_name in ('intrinsics', 'distortion_coeffs'):
                if field_name not in block:
                    incomplete.append('{0}.{1}'.format(
                        camera_name, field_name))
            if (camera_index > 0 and
                    'T_cam_from_previous' not in block):
                incomplete.append('{0}.T_cam_from_previous'.format(
                    camera_name))
        if incomplete:
            _initialization_error(
                filename, 'direct strategy requires complete seeds; missing: '
                '{0}'.format(', '.join(incomplete)))

    if camera_models is not None:
        if len(camera_models) != num_cameras:
            _initialization_error(
                filename, 'expected {0} camera models, got {1}'.format(
                    num_cameras, len(camera_models)))
        for camera_index, camera_model in enumerate(camera_models):
            # Preserve the legacy unknown-model error at its original site.
            if camera_model is not None:
                validateCameraInitializationSeed(
                    camera_model, 'cam{0}'.format(camera_index),
                    normalized_cameras.get(
                        'cam{0}'.format(camera_index), {}))

    return {
        'schema_version': CAMERA_CALIBRATION_INITIALIZATION_SCHEMA_VERSION,
        'kind': CAMERA_CALIBRATION_INITIALIZATION_KIND,
        'strategy': strategy,
        'cameras': normalized_cameras,
    }


def cameraInitializationBaselineGuesses(initialization, num_cameras,
                                        require_all=False):
    """Return fresh chain-baseline transformations from a parsed seed."""
    baselines = list()
    for camera_index in range(1, num_cameras):
        camera_name = 'cam{0}'.format(camera_index)
        block = initialization['cameras'].get(camera_name, {})
        matrix = block.get('T_cam_from_previous')
        if matrix is None:
            if require_all:
                raise RuntimeError(
                    'Initialization is missing {0}.T_cam_from_previous'.format(
                        camera_name))
            baselines.append(None)
        else:
            baselines.append(sm.Transformation(np.asarray(matrix, dtype=float)))
    return baselines


def _cameraInitializationResolution(observations, camera_name):
    if not observations:
        raise RuntimeError(
            'Cannot initialize {0}: no observations are available to obtain '
            'the image resolution'.format(camera_name))
    try:
        resolution = (
            int(observations[0].imCols()),
            int(observations[0].imRows()),
        )
    except Exception as error:
        raise RuntimeError(
            'Cannot initialize {0}: unable to obtain image resolution from '
            'observations: {1}'.format(camera_name, error))
    if resolution[0] <= 0 or resolution[1] <= 0:
        raise RuntimeError(
            'Cannot initialize {0}: observation image resolution must be '
            'positive, got {1}x{2}'.format(
                camera_name, resolution[0], resolution[1]))
    return resolution


def validateCameraInitializationSeed(camera_model, camera_name, seed):
    """Validate optional vectors against one selected native camera model."""
    geometry = camera_model.geometry()
    fields = (
        ('intrinsics', int(geometry.minimalDimensionsProjection())),
        ('distortion_coeffs', int(geometry.minimalDimensionsDistortion())),
    )
    for field_name, expected_size in fields:
        if field_name not in seed:
            continue
        parameters = np.asarray(seed[field_name], dtype=float)
        if parameters.ndim != 1 or parameters.size != expected_size:
            raise RuntimeError(
                '{0}.{1} has {2} values, but the selected camera model '
                'requires {3}'.format(
                    camera_name, field_name, parameters.size,
                    expected_size))


def cameraGeometryFromInitialization(camera_model, camera_name, seed,
                                     observations):
    """Construct seed geometry with observation-derived image metadata."""
    if 'intrinsics' not in seed:
        raise RuntimeError(
            'Cannot construct seeded geometry for {0} without intrinsics'.format(
                camera_name))

    resolution_u, resolution_v = _cameraInitializationResolution(
        observations, camera_name)
    validateCameraInitializationSeed(camera_model, camera_name, seed)
    default_geometry = camera_model.geometry()
    expected_distortion = int(default_geometry.minimalDimensionsDistortion())
    intrinsics = np.asarray(seed['intrinsics'], dtype=float)

    distortion_parameters = np.asarray(
        seed.get('distortion_coeffs', [0.0] * expected_distortion),
        dtype=float)

    try:
        distortion = camera_model.distortionType()
        if expected_distortion:
            distortion.setParameters(distortion_parameters)
        projection_arguments = list(intrinsics) + [
            resolution_u, resolution_v, distortion]
        projection = camera_model.projectionType(*projection_arguments)
        return camera_model.geometry(projection)
    except Exception as error:
        raise RuntimeError(
            'Could not construct seeded camera geometry for {0}: {1}'.format(
                camera_name, error))

class CameraGeometry(object):
    def __init__(self, cameraModel, targetConfig, dataset, geometry=None,
                 verbose=False, windowHalfSizePx=None,
                 maxDisplacementPx=None):
        self.dataset = dataset
        
        self.model = cameraModel
        if geometry is None:
            self.geometry = cameraModel.geometry()
        else:
            self.geometry = geometry
        
        if not type(self.geometry) == cameraModel.geometry:
            raise RuntimeError("The type of geometry passed in \"%s\" does not match the model type \"%s\"" % (type(geometry),type(cameraModel.geometry)))
        
        #create the design variables
        self.dv = cameraModel.designVariable(self.geometry)
        self.setDvActiveStatus(True, True, False)
        self.isGeometryInitialized = False

        #create target detector
        self.ctarget = TargetDetector(
            targetConfig, self.geometry, showCorners=verbose,
            windowHalfSizePx=windowHalfSizePx,
            maxDisplacementPx=maxDisplacementPx)

    def setDvActiveStatus(self, projectionActive, distortionActive, shutterActice):
        self.dv.projectionDesignVariable().setActive(projectionActive)
        self.dv.distortionDesignVariable().setActive(distortionActive)
        self.dv.shutterDesignVariable().setActive(shutterActice)

    def initGeometryFromObservations(self, observations):
        #obtain focal length guess
        success = self.geometry.initializeIntrinsics(observations)
        if not success:
            sm.logError("initialization of focal length for cam with topic {0} failed  ".format(self.dataset.topic))
        
        #in case of an omni model, first optimize over intrinsics only
        #(--> catch most of the distortion with the projection model)
        if self.model == acvb.DistortedOmni:
            success = kcc.calibrateIntrinsics(self, observations, distortionActive=False)
            if not success:
                sm.logError("initialization of intrinsics for cam with topic {0} failed  ".format(self.dataset.topic))
        
        #optimize for intrinsics & distortion    
        success = kcc.calibrateIntrinsics(self, observations)
        if not success:
            sm.logError("initialization of intrinsics for cam with topic {0} failed  ".format(self.dataset.topic))
        
        self.isGeometryInitialized = success        
        return success

    def _applyInitializationSeed(self, camera_name, seed):
        projection = self.geometry.projection()
        parameter_fields = (
            ('intrinsics', projection,
             int(self.geometry.minimalDimensionsProjection())),
            ('distortion_coeffs', projection.distortion(),
             int(self.geometry.minimalDimensionsDistortion())),
        )
        for field_name, native_object, expected_size in parameter_fields:
            if field_name not in seed:
                continue
            parameters = np.asarray(seed[field_name], dtype=float)
            if parameters.ndim != 1 or parameters.size != expected_size:
                raise RuntimeError(
                    '{0}.{1} has {2} values, but the selected camera model '
                    'requires {3}'.format(
                        camera_name, field_name, parameters.size,
                        expected_size))
            # Native no-distortion policies expose a zero-dimensional block,
            # but do not all provide a useful setParameters(empty) binding.
            if expected_size:
                native_object.setParameters(parameters)

    def _restoreUnseededInitializationDefaults(self, seed):
        """Keep omitted refine fields stable across optimizer restarts."""
        projection = self.geometry.projection()
        native_fields = {
            'intrinsics': projection,
            'distortion_coeffs': projection.distortion(),
        }
        if not hasattr(self, '_initializationSeedDefaults'):
            self._initializationSeedDefaults = {
                field_name: tuple(
                    np.asarray(native_object.getParameters()).flatten())
                for field_name, native_object in native_fields.items()
            }
            return

        for field_name, native_object in native_fields.items():
            if field_name in seed:
                continue
            parameters = np.asarray(
                self._initializationSeedDefaults[field_name], dtype=float)
            if parameters.size:
                native_object.setParameters(parameters)

    def _bootstrapObservationMetadata(self, observations, camera_name):
        """Populate image resolution without accepting analytic parameters.

        The native projection bindings expose ``ru``/``rv`` but no stable
        resolution setter.  ``initializeIntrinsics`` writes that metadata
        before attempting its analytic estimate, so use it only as a metadata
        bootstrap and let the caller unconditionally restore the seed.
        """
        expected_ru, expected_rv = _cameraInitializationResolution(
            observations, camera_name)

        projection = self.geometry.projection()
        if (int(projection.ru()) == expected_ru and
                int(projection.rv()) == expected_rv):
            return

        bootstrap_error = None
        try:
            # The success value describes the analytic focal-length estimate;
            # direct/seeded initialization deliberately does not depend on it.
            self.geometry.initializeIntrinsics(observations)
        except Exception as error:
            bootstrap_error = error

        projection = self.geometry.projection()
        if (int(projection.ru()) != expected_ru or
                int(projection.rv()) != expected_rv):
            detail = (': {0}'.format(bootstrap_error)
                      if bootstrap_error is not None else '')
            raise RuntimeError(
                'Cannot initialize {0}: native camera model could not obtain '
                'image resolution {1}x{2} from observations{3}'.format(
                    camera_name, expected_ru, expected_rv, detail))

    def initGeometryFromObservationsWithSeed(self, observations, camera_name,
                                             seed):
        """Apply a partial seed, then run the legacy intrinsic refinement."""
        if 'intrinsics' not in seed:
            success = self.geometry.initializeIntrinsics(observations)
            if not success:
                sm.logError(
                    "initialization of focal length for cam with topic {0} "
                    "failed  ".format(self.dataset.topic))
        else:
            self._bootstrapObservationMetadata(observations, camera_name)

        # Explicit intrinsics bypass the analytic estimate.  Distortion is
        # overlaid after any analytic projection estimate, so neither supplied
        # block is silently overwritten before the original LM stages.
        self._restoreUnseededInitializationDefaults(seed)
        self._applyInitializationSeed(camera_name, seed)

        if self.model == acvb.DistortedOmni:
            success = kcc.calibrateIntrinsics(
                self, observations, distortionActive=False)
            if not success:
                sm.logError(
                    "initialization of intrinsics for cam with topic {0} "
                    "failed  ".format(self.dataset.topic))

        success = kcc.calibrateIntrinsics(self, observations)
        if not success:
            sm.logError(
                "initialization of intrinsics for cam with topic {0} "
                "failed  ".format(self.dataset.topic))

        self.isGeometryInitialized = success
        return success

    def initGeometryFromSeed(self, observations, camera_name, seed):
        """Install a complete direct-mode seed without running intrinsic LM."""
        missing = [field_name for field_name in
                   ('intrinsics', 'distortion_coeffs')
                   if field_name not in seed]
        if missing:
            raise RuntimeError(
                'Direct initialization for {0} is missing: {1}'.format(
                    camera_name, ', '.join(missing)))
        self._bootstrapObservationMetadata(observations, camera_name)
        self._restoreUnseededInitializationDefaults(seed)
        self._applyInitializationSeed(camera_name, seed)
        self.isGeometryInitialized = True
        return True

class TargetDetector(object):
    def __init__(self, targetConfig, cameraGeometry, showCorners=False,
                 showReproj=False, showOneStep=False,
                 windowHalfSizePx=None, maxDisplacementPx=None):
        self.targetConfig = targetConfig
        
        #initialize the calibration target
        targetParams = targetConfig.getTargetParams()
        targetType = targetConfig.getTargetType()

        if targetType == 'checkerboard':
            options = acv.CheckerboardOptions()
            options.filterQuads = True
            options.normalizeImage = True
            options.useAdaptiveThreshold = True        
            options.performFastCheck = False
            options.windowWidth = 5            
            options.showExtractionVideo = showCorners
            
            self.grid = acv.GridCalibrationTargetCheckerboard(targetParams['targetRows'], 
                                                              targetParams['targetCols'], 
                                                              targetParams['rowSpacingMeters'], 
                                                              targetParams['colSpacingMeters'], 
                                                              options)
        elif targetType == 'circlegrid':
            options = acv.CirclegridOptions()
            options.showExtractionVideo = showCorners
            options.useAsymmetricCirclegrid = targetParams['asymmetricGrid']
            
            self.grid = acv.GridCalibrationTargetCirclegrid(targetParams['targetRows'],
                                                           targetParams['targetCols'], 
                                                           targetParams['spacingMeters'], 
                                                           options)
         
        elif targetType == 'aprilgrid':
            options = acv_april.AprilgridOptions()
            if windowHalfSizePx is not None:
                options.subpixWindowHalfSize = windowHalfSizePx
            if maxDisplacementPx is not None:
                options.maxSubpixDisplacement2 = maxDisplacementPx * maxDisplacementPx
            #enforce more than one row --> pnp solution can be bad if all points are almost on a line...
            options.minTagsForValidObs = int( np.max( [targetParams['tagRows'], targetParams['tagCols']] ) + 1 )
            options.showExtractionVideo = showCorners
            
            self.grid = acv_april.GridCalibrationTargetAprilgrid(targetParams['tagRows'], 
                                                                 targetParams['tagCols'], 
                                                                 targetParams['tagSize'], 
                                                                 targetParams['tagSpacing'], 
                                                                 targetParams['tagStartId'],
                                                                 options)
        else:
            RuntimeError('Unknown calibration target type!')

        options = acv.GridDetectorOptions() 
        options.imageStepping = showOneStep
        options.plotCornerReprojection = showReproj
        options.filterCornerOutliers = False
        
        self.detector = acv.GridDetector(cameraGeometry, self.grid, options)

class CalibrationTarget(object):
    def __init__(self, target, estimateLandmarks=False):
        self.target = target
        # Create design variables and expressions for all target points.
        P_t_dv = []
        P_t_ex = []
        for i in range(0,self.target.size()):
            p_t_dv = aopt.HomogeneousPointDv(sm.toHomogeneous(self.target.point(i)));
            p_t_dv.setActive(estimateLandmarks)
            p_t_ex = p_t_dv.toExpression()
            P_t_dv.append(p_t_dv)
            P_t_ex.append(p_t_ex)
        self.P_t_dv = P_t_dv
        self.P_t_ex = P_t_ex
    def getPoint(self,i):
        return P_t_ex[i]

class CalibrationTargetOptimizationProblem(ic.CalibrationOptimizationProblem):        
    @classmethod
    def fromTargetViewObservations(cls, cameras, target, baselines, timestamp, T_tc_guess, rig_observations, useBlakeZissermanMest=True):
        rval = CalibrationTargetOptimizationProblem()        

        #store the arguements in case we want to rebuild a modified problem
        rval.cameras = cameras
        rval.target = target
        rval.baselines = baselines
        rval.timestamp = timestamp
        rval.T_tc_guess = T_tc_guess
        rval.rig_observations = rig_observations
        
        # 1. Create a design variable for this pose
        T_target_camera = T_tc_guess
        
        rval.dv_T_target_camera = aopt.TransformationDv(T_target_camera)
        for i in range(0, rval.dv_T_target_camera.numDesignVariables()):
            rval.addDesignVariable( rval.dv_T_target_camera.getDesignVariable(i), TRANSFORMATION_GROUP_ID)
        
        #2. add all baselines DVs
        for baseline_dv in baselines:
            for i in range(0, baseline_dv.numDesignVariables()):
                rval.addDesignVariable(baseline_dv.getDesignVariable(i), CALIBRATION_GROUP_ID)
        
        #3. add landmark DVs
        for p in target.P_t_dv:
            rval.addDesignVariable(p,LANDMARK_GROUP_ID)
        
        #4. add camera DVs
        for camera in cameras:
            if not camera.isGeometryInitialized:
                raise RuntimeError('The camera geometry is not initialized. Please initialize with initGeometry() or initGeometryFromDataset()')
            camera.setDvActiveStatus(True, True, False)
            rval.addDesignVariable(camera.dv.distortionDesignVariable(), CALIBRATION_GROUP_ID)
            rval.addDesignVariable(camera.dv.projectionDesignVariable(), CALIBRATION_GROUP_ID)
            rval.addDesignVariable(camera.dv.shutterDesignVariable(), CALIBRATION_GROUP_ID)
        
        #4.add all observations for this view
        cams_in_view = set()
        rval.rerrs=dict()
        rerr_cnt=0
        for cam_id, obs in rig_observations:
            camera = cameras[cam_id]
            cams_in_view.add(cam_id)
            
            #add reprojection errors
            #build baseline chain (target->cam0->baselines->camN)                
            T_cam0_target = rval.dv_T_target_camera.expression.inverse()
            T_camN_calib = T_cam0_target
            for idx in range(0, cam_id):
                T_camN_calib =  baselines[idx].toExpression() * T_camN_calib
            
            # \todo pass in the detector uncertainty somehow.
            cornerUncertainty = 1.0
            R = np.eye(2) * cornerUncertainty * cornerUncertainty
            invR = np.linalg.inv(R)
            
            rval.rerrs[cam_id] = list()
            for i in range(0,len(target.P_t_ex)):
                p_target = target.P_t_ex[i]
                valid, y = obs.imagePoint(i)
                if valid:
                    rerr_cnt+=1
                    # Create an error term.
                    rerr = camera.model.reprojectionError(y, invR, T_camN_calib * p_target, camera.dv)
                    rerr.idx = i
                    
                    #add blake-zisserman mest
                    if useBlakeZissermanMest:
                        mest = aopt.BlakeZissermanMEstimator( 2.0 )
                        rerr.setMEstimatorPolicy(mest)
                    rval.addErrorTerm(rerr)
                    rval.rerrs[cam_id].append(rerr)
                else:
                    rval.rerrs[cam_id].append(None)

        sm.logDebug("Adding a view with {0} cameras and {1} error terms".format(len(cams_in_view), rerr_cnt))
        return rval

def removeCornersFromBatch(batch, camId_cornerIdList_tuples, useBlakeZissermanMest=True):
    #translate (camid,obs) tuple to dict
    obsdict=dict()
    for cidx, obs in batch.rig_observations:
        obsdict[cidx]=obs
       
    #disable the corners
    hasCornerRemoved=False
    for cidx, removelist in camId_cornerIdList_tuples:
        for corner_id in removelist: 
            obsdict[cidx].removeImagePoint(corner_id)
            hasCornerRemoved=True
    assert hasCornerRemoved, "need to remove at least one corner..."
    
    #rebuild problem
    new_problem = CalibrationTargetOptimizationProblem.fromTargetViewObservations(batch.cameras, 
                                                                                  batch.target, 
                                                                                  batch.baselines, 
                                                                                  batch.timestamp, 
                                                                                  batch.T_tc_guess, 
                                                                                  batch.rig_observations,
                                                                                  useBlakeZissermanMest=useBlakeZissermanMest)

    return new_problem
        
class CameraCalibration(object):
    def __init__(self, cameras, baseline_guesses, estimateLandmarks=False, verbose=False, useBlakeZissermanMest=True):
        self.cameras = cameras
        self.useBlakeZissermanMest = useBlakeZissermanMest
        #create the incremental estimator
        self.estimator = ic.IncrementalEstimator(CALIBRATION_GROUP_ID)
        self.linearSolverOptions = self.estimator.getLinearSolverOptions()
        self.optimizerOptions = self.estimator.getOptimizerOptions()
        self.target = CalibrationTarget(cameras[0].ctarget.detector.target(), estimateLandmarks=estimateLandmarks)
        self.initializeBaselineDVs(baseline_guesses)
        #storage for the used views
        self.views = list()
        
    def initializeBaselineDVs(self, baseline_guesses):
        self.baselines = list()
        for baseline_idx in range(0, len(self.cameras)-1): 
            self.baselines.append( aopt.TransformationDv(baseline_guesses[baseline_idx]) )
            
    def getBaseline(self, i):
        return self.baselines[i]
    
    def addTargetView(self, timestamp, rig_observations, T_tc_guess, force=False):
        #create the problem for this batch and try to add it 
        batch_problem = CalibrationTargetOptimizationProblem.fromTargetViewObservations(self.cameras, self.target, self.baselines, timestamp, T_tc_guess, rig_observations, useBlakeZissermanMest=self.useBlakeZissermanMest)
        native_runtime.apply_optimizer_threads(self.optimizerOptions)
        self.estimator_return_value = native_runtime.run_incremental_batch(
            self.estimator, batch_problem, force)
        
        if self.estimator_return_value.numIterations >= self.optimizerOptions.maxIterations:
            sm.logError("Did not converge in maxIterations... restarting...")
            raise OptimizationDiverged
        
        success = self.estimator_return_value.batchAccepted
        if success:
            sm.logDebug("The estimator accepted this batch")
            self.views.append(batch_problem)
        else:
            sm.logDebug("The estimator did not accept this batch")
        return success
