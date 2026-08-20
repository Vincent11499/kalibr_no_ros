"""Generate the two native calibration CLI overlays.

The transformer is intentionally separate from the top-level build logic.  It
operates on the already staged (ROS-free/model-extended) command so other
extensions can run before it without duplicating either upstream entry point.
"""

from __future__ import print_function

import argparse
import os
import sys


SUPPORTED_COMMANDS = (
    "kalibr_calibrate_cameras",
    "kalibr_calibrate_imu_camera",
)
_MARKER = "# kalibr-native-runtime-overlay"


def _replace_once(source, old, new, description):
    occurrences = source.count(old)
    if occurrences != 1:
        raise RuntimeError(
            "expected one {0} anchor, found {1}".format(
                description, occurrences))
    return source.replace(old, new, 1)


def _replace_expected(source, old, new, expected, description):
    occurrences = source.count(old)
    if occurrences != expected:
        raise RuntimeError(
            "expected {0} {1} anchors, found {2}".format(
                expected, description, occurrences))
    return source.replace(old, new)


def transform_cli_source(source, command):
    """Return a CLI source string with native runtime integration."""
    if command not in SUPPORTED_COMMANDS:
        raise ValueError("unsupported calibration command: {0}".format(command))
    if _MARKER in source:
        return source

    source = _replace_once(
        source,
        "import argparse\n",
        "import argparse\nimport kalibr_native_optimizer as native_runtime\n"
        + _MARKER + "\n",
        "argparse import",
    )
    source = _replace_once(
        source,
        "    #print help if no argument is specified\n",
        "    nativeOptions = parser.add_argument_group("
        "'Native parallelism and profiling')\n"
        "    native_runtime.add_parallelism_arguments(nativeOptions)\n\n"
        "    #print help if no argument is specified\n",
        "argument parser",
    )
    # The Python 2-era entry points catch bare ``except`` around argparse,
    # which turns argparse's successful ``--help`` SystemExit(0) into exit 2.
    # Restrict the compatibility catch to real exceptions; calibration
    # parsing and validation otherwise remain unchanged.
    source = _replace_once(
        source,
        "    try:\n        parsed = parser.parse_args()\n"
        "    except:\n        sys.exit(2)\n",
        "    try:\n        parsed = parser.parse_args()\n"
        "    except Exception:\n        sys.exit(2)\n",
        "argparse SystemExit handling",
    )
    source = _replace_once(
        source,
        "    return parsed\n",
        "    native_runtime.configure(parsed, command={0!r})\n"
        "    return parsed\n".format(command),
        "parsed argument return",
    )
    if command == "kalibr_calibrate_cameras":
        source = _replace_once(
            source,
            "    reader = kc.BagImageDatasetReader(bagfile, topic, bag_from_to=from_to, bag_freq=freq)\n",
            "    reader = native_runtime.timed_call(\n"
            "        \"image_bag_index\", \"io\", {\"topic\": topic},\n"
            "        kc.BagImageDatasetReader, bagfile, topic,\n"
            "        bag_from_to=from_to, bag_freq=freq)\n",
            "camera image bag index",
        )
        source = _replace_expected(
            source,
            "            if not cam.initGeometryFromObservations(observations):\n",
            "            if not native_runtime.timed_call(\n"
            "                    \"camera_intrinsics_initialization\",\n"
            "                    \"initialization\", {\"inclusive\": True},\n"
            "                    cam.initGeometryFromObservations, observations):\n",
            2,
            "camera intrinsic initialization",
        )
        source = _replace_once(
            source,
            "                baseline_guesses = graph.getInitialGuesses(cameraList)\n",
            "                baseline_guesses = native_runtime.timed_call(\n"
            "                    \"camera_baseline_initialization\",\n"
            "                    \"initialization\", {\"inclusive\": True},\n"
            "                    graph.getInitialGuesses, cameraList)\n",
            "camera baseline initialization",
        )
        source = _replace_once(
            source,
            "            calibrator = kcc.CameraCalibration(cameraList, baseline_guesses, verbose=parsed.verbose, useBlakeZissermanMest=parsed.doBlakeZisserman)\n",
            "            calibrator = native_runtime.timed_call(\n"
            "                \"camera_full_batch_initialization\",\n"
            "                \"initialization\", {\"inclusive\": True},\n"
            "                kcc.CameraCalibration, cameraList,\n"
            "                baseline_guesses, verbose=parsed.verbose,\n"
            "                useBlakeZissermanMest=parsed.doBlakeZisserman)\n",
            "camera full batch initialization",
        )
        # Outlier filtering removes a batch and submits its corrected
        # replacement directly through the estimator, bypassing
        # CameraCalibration.addTargetView().  Keep this optimization visible
        # in the timing report and re-apply an explicit thread override at the
        # actual callsite without wrapping the Boost.Python estimator object.
        source = _replace_once(
            source,
            "                            rval = calibrator.estimator.addBatch( new_batch, False )\n",
            "                            native_runtime.apply_optimizer_threads(\n"
            "                                calibrator.estimator.getOptimizerOptions())\n"
            "                            rval = native_runtime.run_incremental_batch(\n"
            "                                calibrator.estimator, new_batch, False)\n",
            "corrected camera batch optimization",
        )
        source = _replace_once(
            source,
            "            kcc.saveChainParametersYaml(calibrator, resultFile, graph)\n",
            "            native_runtime.timed_call(\n"
            "                \"result_serialization\", \"output\",\n"
            "                {\"format\": \"camchain_yaml\"},\n"
            "                kcc.saveChainParametersYaml, calibrator,\n"
            "                resultFile, graph)\n",
            "camera YAML output",
        )
        source = _replace_once(
            source,
            "            kcc.saveResultTxt(calibrator, filename=resultFileTxt)\n",
            "            native_runtime.timed_call(\n"
            "                \"result_serialization\", \"output\",\n"
            "                {\"format\": \"camera_text\"},\n"
            "                kcc.saveResultTxt, calibrator,\n"
            "                filename=resultFileTxt)\n",
            "camera text output",
        )
        source = _replace_once(
            source,
            "            kcc.generateReport(calibrator, reportFile, showOnScreen=not parsed.dontShowReport, graph=G, removedOutlierCorners=removedOutlierCorners);\n",
            "            native_runtime.timed_call(\n"
            "                \"report_generation\", \"output\",\n"
            "                {\"format\": \"pdf\"}, kcc.generateReport,\n"
            "                calibrator, reportFile,\n"
            "                showOnScreen=not parsed.dontShowReport, graph=G,\n"
            "                removedOutlierCorners=removedOutlierCorners)\n",
            "camera report generation",
        )
    else:
        source = _replace_once(
            source,
            "    iCal.buildProblem(splineOrder=6, \n",
            "    with native_runtime.stage(\n"
            "            \"problem_build_total\", category=\"problem_build\",\n"
            "            metadata={\"inclusive\": True}):\n"
            "        iCal.buildProblem(splineOrder=6, \n",
            "IMU problem build",
        )
        source = _replace_once(
            source,
            "    print(\"Before Optimization\")\n"
            "    print(\"===================\")\n"
            "    util.printErrorStatistics(iCal)\n",
            "    print(\"Before Optimization\")\n"
            "    print(\"===================\")\n"
            "    native_runtime.timed_call(\n"
            "        \"residual_statistics\", \"reporting\",\n"
            "        {\"position\": \"before_optimization\"},\n"
            "        util.printErrorStatistics, iCal)\n",
            "pre-optimization residual statistics",
        )
        source = _replace_once(
            source,
            "    print(\"After Optimization (Results)\")\n"
            "    print(\"==================\")\n"
            "    util.printErrorStatistics(iCal)\n"
            "    util.printResults(iCal, withCov=parsed.recover_cov)\n",
            "    print(\"After Optimization (Results)\")\n"
            "    print(\"==================\")\n"
            "    native_runtime.timed_call(\n"
            "        \"residual_statistics\", \"reporting\",\n"
            "        {\"position\": \"after_optimization\"},\n"
            "        util.printErrorStatistics, iCal)\n"
            "    native_runtime.timed_call(\n"
            "        \"result_formatting\", \"reporting\", {},\n"
            "        util.printResults, iCal, withCov=parsed.recover_cov)\n",
            "post-optimization result statistics",
        )
        source = _replace_once(
            source,
            "    iCal.saveCamChainParametersYaml(yamlFilename)\n",
            "    native_runtime.timed_call(\n"
            "        \"result_serialization\", \"output\",\n"
            "        {\"format\": \"camchain_yaml\"},\n"
            "        iCal.saveCamChainParametersYaml, yamlFilename)\n",
            "IMU camchain YAML output",
        )
        source = _replace_once(
            source,
            "    iCal.saveImuSetParametersYaml(yamlFilename)\n",
            "    native_runtime.timed_call(\n"
            "        \"result_serialization\", \"output\",\n"
            "        {\"format\": \"imu_yaml\"},\n"
            "        iCal.saveImuSetParametersYaml, yamlFilename)\n",
            "IMU YAML output",
        )
        source = _replace_once(
            source,
            "    util.saveResultTxt(iCal, filename=resultFileTxt)\n",
            "    native_runtime.timed_call(\n"
            "        \"result_serialization\", \"output\",\n"
            "        {\"format\": \"imu_camera_text\"}, util.saveResultTxt,\n"
            "        iCal, filename=resultFileTxt)\n",
            "IMU text output",
        )
        source = _replace_once(
            source,
            "    util.generateReport(iCal, filename=reportFile, showOnScreen=not parsed.dontShowReport)\n",
            "    native_runtime.timed_call(\n"
            "        \"report_generation\", \"output\",\n"
            "        {\"format\": \"pdf\"}, util.generateReport, iCal,\n"
            "        filename=reportFile,\n"
            "        showOnScreen=not parsed.dontShowReport)\n",
            "IMU report generation",
        )
    source = _replace_once(
        source,
        'if __name__ == "__main__":\n    main()\n',
        'if __name__ == "__main__":\n'
        '    try:\n'
        '        main()\n'
        '    except BaseException:\n'
        '        try:\n'
        '            native_runtime.finish("error")\n'
        '        except Exception:\n'
        '            pass\n'
        '        raise\n'
        '    else:\n'
        '        native_runtime.finish("ok")\n',
        "main invocation",
    )
    return source


def transform_file(path, command, output=None):
    output = output or path
    with open(path, "r") as source_file:
        transformed = transform_cli_source(source_file.read(), command)
    temporary = "{0}.tmp.{1}".format(output, os.getpid())
    with open(temporary, "w") as output_file:
        output_file.write(transformed)
    os.replace(temporary, output)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Add native parallelism/timing controls to a Kalibr CLI")
    parser.add_argument("--command", choices=SUPPORTED_COMMANDS, required=True)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output")
    arguments = parser.parse_args(argv)
    transform_file(arguments.input, arguments.command, arguments.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
