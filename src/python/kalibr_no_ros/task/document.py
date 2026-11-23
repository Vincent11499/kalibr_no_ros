"""Stable YAML documents, versions, errors, and task-relative paths."""

from pathlib import Path
import math
import tempfile

import yaml

from ..version import (
    CONFIG_SCHEMA_VERSION,
    RESULT_SCHEMA_VERSION,
    SUPPORTED_CONFIG_SCHEMA_VERSIONS,
    SUPPORTED_RESULT_SCHEMA_VERSIONS,
    supports_schema_version,
)

TASK_SCHEMA_VERSION = CONFIG_SCHEMA_VERSION
CALIBRATION_RESULT_VERSION = RESULT_SCHEMA_VERSION

class TaskError(ValueError):
    pass


class _StableDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        # Indent block sequences beneath their mapping key.  PyYAML's default
        # indentless style is valid YAML, but is harder to scan by eye.
        return super().increase_indent(flow, False)


class _FlowSequence(list):
    """Sequence that is rendered in YAML flow style."""


def _represent_float(dumper, value):
    text = format(value, ".17g")
    # Keep integral-valued floats visibly typed as floats without PyYAML's
    # explicit ``!!float`` annotation (for example, write 0.0 instead of 0).
    if math.isfinite(value) and not any(marker in text for marker in ".eE"):
        text += ".0"
    return dumper.represent_scalar("tag:yaml.org,2002:float", text)


def _represent_flow_sequence(dumper, value):
    return dumper.represent_sequence(
        "tag:yaml.org,2002:seq", value, flow_style=True)


_StableDumper.add_representer(float, _represent_float)
_StableDumper.add_representer(_FlowSequence, _represent_flow_sequence)


def _is_numeric_matrix(value):
    if not isinstance(value, list) or not value:
        return False
    if not all(isinstance(row, list) and row for row in value):
        return False
    column_count = len(value[0])
    return all(
        len(row) == column_count
        and all(type(element) in (int, float) for element in row)
        for row in value
    )


def _format_yaml_collections(value):
    """Keep mappings block-oriented while rendering compact numeric data inline."""
    if _is_numeric_matrix(value):
        return [_FlowSequence(row) for row in value]
    if isinstance(value, dict):
        return {
            key: _format_yaml_collections(element)
            for key, element in value.items()
        }
    if isinstance(value, list):
        if value and all(
                not isinstance(element, (dict, list)) for element in value):
            return _FlowSequence(value)
        return [_format_yaml_collections(element) for element in value]
    return value


def load_yaml(path):
    path = Path(path).resolve()
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = yaml.safe_load(stream)
    except yaml.YAMLError as error:
        raise TaskError("invalid YAML in {}: {}".format(path, error)) from error
    if not isinstance(value, dict):
        raise TaskError("YAML root must be a mapping: {}".format(path))
    return value


def dump_yaml(value, path):
    path = Path(path)
    text = yaml.dump(_format_yaml_collections(value), Dumper=_StableDumper,
                     allow_unicode=True, default_flow_style=False,
                     sort_keys=False, width=2147483647)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix="." + path.name + ".", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(text)
        temporary.replace(path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def resolve_task_path(task, value):
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = Path(task["_config_dir"]) / path
    return path.resolve()


def require_document_version(document, label, kind=None, *, result=False):
    supported = (SUPPORTED_RESULT_SCHEMA_VERSIONS
                 if result else SUPPORTED_CONFIG_SCHEMA_VERSIONS)
    if not supports_schema_version(document.get("schema_version"), supported):
        raise TaskError(
            "{} schema_version must be one of {}".format(
                label, ", ".join(sorted(supported))))
    if kind is not None and document.get("kind", kind) != kind:
        raise TaskError("{} kind must be {}".format(label, kind))


def canonical_document(document, *, result=False):
    """Normalize a supported schema alias for semantic comparisons."""
    from copy import deepcopy
    value = deepcopy(document)
    require_document_version(value, "document", result=result)
    value["schema_version"] = (
        RESULT_SCHEMA_VERSION if result else CONFIG_SCHEMA_VERSION)
    return value
