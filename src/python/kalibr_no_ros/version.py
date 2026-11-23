"""Software, document, and calibration-behavior versions."""

VERSION = "1.1.0"

# Document formats evolve independently from the software release.  The first
# v1.1.0 package briefly wrote 1.1.0 without changing any document shape, so it
# remains a read-only compatibility alias for the canonical 1.0.0 schemas.
CONFIG_SCHEMA_VERSION = "1.0.0"
DATASET_SCHEMA_VERSION = "1.0.0"
RESULT_SCHEMA_VERSION = "1.0.0"
STATE_SCHEMA_VERSION = "1.0.0"
CALIB_PARAMS_SCHEMA_VERSION = "1.0.0"

_V1_SCHEMA_ALIASES = frozenset(("1.0.0", "1.1.0"))
SUPPORTED_CONFIG_SCHEMA_VERSIONS = _V1_SCHEMA_ALIASES
SUPPORTED_DATASET_SCHEMA_VERSIONS = _V1_SCHEMA_ALIASES
SUPPORTED_RESULT_SCHEMA_VERSIONS = _V1_SCHEMA_ALIASES
SUPPORTED_STATE_SCHEMA_VERSIONS = _V1_SCHEMA_ALIASES
SUPPORTED_CALIB_PARAMS_SCHEMA_VERSIONS = _V1_SCHEMA_ALIASES

# Reuse compatibility changes only when calibration math, defaults, data
# selection, or coordinate semantics change.
CALIBRATION_BEHAVIOR_VERSION = "1.0.0"
_SOFTWARE_BEHAVIOR_VERSIONS = {
    "1.0.0": "1.0.0",
    "1.1.0": "1.0.0",
}


def supports_schema_version(value, supported):
    return isinstance(value, str) and value in supported


def calibration_behavior_version(software):
    """Return a known calibration behavior for a provenance mapping."""
    if not isinstance(software, dict):
        return None
    explicit = software.get("calibration_behavior_version")
    if isinstance(explicit, str):
        return explicit
    return _SOFTWARE_BEHAVIOR_VERSIONS.get(software.get("version"))


# Deprecated compatibility alias for third-party imports.  Internal code must
# select the schema family explicitly.
SCHEMA_VERSION = CONFIG_SCHEMA_VERSION
