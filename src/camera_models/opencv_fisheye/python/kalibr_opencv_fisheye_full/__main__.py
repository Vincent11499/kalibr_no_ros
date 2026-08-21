import argparse

from .yaml_io import (
    DIRECT_TRANSFORM,
    INVERSE_TRANSFORM,
    export_kalibr_camchain,
    import_opencv_mono,
    import_opencv_stereo,
)


def _resolution(values):
    return tuple(values) if values is not None else None


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Convert full OpenCV fisheye YAML, including alpha/skew"
    )
    commands = parser.add_subparsers(dest="command", required=True)

    mono = commands.add_parser("import-mono")
    mono.add_argument("--input", required=True)
    mono.add_argument("--output", required=True)
    mono.add_argument("--topic")
    mono.add_argument("--resolution", nargs=2, type=int)

    stereo = commands.add_parser("import-stereo")
    stereo.add_argument("--stereo", required=True)
    stereo.add_argument("--left")
    stereo.add_argument("--right")
    stereo.add_argument("--output", required=True)
    stereo.add_argument("--topics", nargs=2)
    stereo.add_argument("--left-resolution", nargs=2, type=int)
    stereo.add_argument("--right-resolution", nargs=2, type=int)
    stereo.add_argument(
        "--transform-direction",
        choices=(DIRECT_TRANSFORM, INVERSE_TRANSFORM),
        default=None,
        help="override transform_direction stored in the input file",
    )
    translation = stereo.add_mutually_exclusive_group()
    translation.add_argument(
        "--translation-scale",
        type=float,
        help="multiply stored T by this value to obtain Kalibr meters",
    )
    translation.add_argument(
        "--translation-unit",
        choices=("m", "cm", "mm", "um"),
        help="unit of stored T; converted to Kalibr meters",
    )

    export = commands.add_parser("export")
    export.add_argument("--input", required=True)
    export.add_argument("--output-prefix", required=True)
    return parser


def main(argv=None):
    arguments = build_argument_parser().parse_args(argv)
    if arguments.command == "import-mono":
        outputs = [
            import_opencv_mono(
                arguments.input,
                arguments.output,
                resolution=_resolution(arguments.resolution),
                topic=arguments.topic,
            )
        ]
    elif arguments.command == "import-stereo":
        outputs = [
            import_opencv_stereo(
                arguments.stereo,
                arguments.output,
                left_path=arguments.left,
                right_path=arguments.right,
                resolutions=(
                    _resolution(arguments.left_resolution),
                    _resolution(arguments.right_resolution),
                ),
                topics=(
                    tuple(arguments.topics)
                    if arguments.topics is not None
                    else (None, None)
                ),
                transform_direction=arguments.transform_direction,
                translation_scale=arguments.translation_scale,
                translation_unit=arguments.translation_unit,
            )
        ]
    else:
        outputs = export_kalibr_camchain(
            arguments.input, arguments.output_prefix
        )
    for output in outputs:
        print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
