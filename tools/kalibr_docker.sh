#!/usr/bin/env bash
set -euo pipefail

readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly default_base_image="sha256:2edbbc5dc405e9612ba3584ce95480277e3eb374407b5505fe26f17df77c7dbc"
readonly base_tag="kalibr-ubuntu-base:22.04-local"
readonly runtime_image="${KALIBR_DOCKER_IMAGE:-kalibr-cedarobot-runtime:ubuntu22.04}"
readonly ubuntu_mirror="${KALIBR_UBUNTU_MIRROR:-https://mirrors.aliyun.com/ubuntu}"

usage() {
  cat <<'EOF'
Usage:
  tools/kalibr_docker.sh build [BASE_IMAGE]
  tools/kalibr_docker.sh check INSTALL_DIR
  tools/kalibr_docker.sh machine INSTALL_DIR USER_CONFIG MACHINE_BATCH_DIR

Commands:
  build    Build the fixed Ubuntu 22.04 runtime dependency image.
  check    Verify Python imports, native dependencies, and both CLI entries.
  machine  Run one machine workflow with INSTALL_DIR mounted read-only and the
           complete batch directory mounted read-write.

Environment:
  KALIBR_DOCKER_IMAGE  Runtime image tag (default:
                       kalibr-cedarobot-runtime:ubuntu22.04).
  KALIBR_UBUNTU_MIRROR Ubuntu package mirror used for main/updates/backports
                       (default: Aliyun; security remains official).
EOF
}

fail() {
  printf 'kalibr_docker: error: %s\n' "$*" >&2
  exit 2
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || fail "required command not found: $1"
}

absolute_directory() {
  local path="$1"
  [[ -d "$path" ]] || fail "directory not found: $path"
  realpath "$path"
}

absolute_file() {
  local path="$1"
  [[ -f "$path" ]] || fail "file not found: $path"
  realpath "$path"
}

require_runtime_image() {
  docker image inspect "$runtime_image" >/dev/null 2>&1 \
    || fail "runtime image not found; run: tools/kalibr_docker.sh build"
}

build_image() {
  local source_image="${1:-$default_base_image}"
  docker image inspect "$source_image" >/dev/null 2>&1 \
    || fail "base image not found: $source_image"
  docker tag "$source_image" "$base_tag"
  docker build --pull=false --network=host \
    --build-arg "BASE_IMAGE=$base_tag" \
    --build-arg "UBUNTU_MIRROR=$ubuntu_mirror" \
    --file "$script_dir/docker/Dockerfile.runtime" \
    --tag "$runtime_image" \
    "$script_dir/docker"
}

check_install() {
  local install_dir
  install_dir="$(absolute_directory "$1")"
  [[ -x "$install_dir/bin/kalibr-noros" ]] \
    || fail "kalibr-noros is missing or not executable in $install_dir"
  [[ -x "$install_dir/bin/kalibr-cedarobot" ]] \
    || fail "kalibr-cedarobot is missing or not executable in $install_dir"
  require_runtime_image

  docker run --rm \
    --mount "type=bind,src=$install_dir,dst=/opt/kalibr,readonly" \
    --entrypoint /bin/bash \
    "$runtime_image" -lc '
      set -euo pipefail
      missing=0
      while IFS= read -r -d "" path; do
        if file -Lb "$path" | grep -q "^ELF"; then
          unresolved="$(LD_LIBRARY_PATH=/opt/kalibr/lib ldd "$path" 2>/dev/null \
            | grep "not found" || true)"
          if [[ -n "$unresolved" ]]; then
            printf "[missing] %s\n%s\n" "$path" "$unresolved" >&2
            missing=1
          fi
        fi
      done < <(find /opt/kalibr -type f -print0)
      [[ "$missing" -eq 0 ]]
      python3 -c "import cv2, matplotlib, numpy, scipy, yaml; print(\"NumPy\", numpy.__version__, \"OpenCV\", cv2.__version__)"
      /opt/kalibr/bin/kalibr-noros --help >/dev/null
      /opt/kalibr/bin/kalibr-cedarobot --help >/dev/null
      echo "Kalibr Docker runtime check passed."
    '
}

run_machine() {
  local install_dir config_path machine_batch machine_name batch_name
  local container_batch uid gid
  install_dir="$(absolute_directory "$1")"
  config_path="$(absolute_file "$2")"
  machine_batch="$(absolute_directory "$3")"
  [[ -x "$install_dir/bin/kalibr-cedarobot" ]] \
    || fail "kalibr-cedarobot is missing or not executable in $install_dir"
  require_runtime_image

  machine_name="$(basename "$(dirname "$machine_batch")")"
  batch_name="$(basename "$machine_batch")"
  [[ "$machine_name" =~ ^[A-Za-z][A-Za-z0-9]*-[0-9]+$ ]] \
    || fail "machine directory must match <machine_version>-<machine_num>: $machine_name"
  [[ "$batch_name" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$ ]] \
    || fail "invalid batch directory name: $batch_name"
  container_batch="/data/$machine_name/$batch_name"
  uid="$(id -u)"
  gid="$(id -g)"

  local -a terminal=()
  if [[ -t 0 && -t 1 ]]; then
    terminal=(-it)
  fi

  docker run --rm "${terminal[@]}" \
    --user "$uid:$gid" \
    --shm-size 2g \
    --env HOME=/tmp \
    --env MPLCONFIGDIR=/tmp/matplotlib \
    --env "KALIBR_MACHINE_DIR=$container_batch" \
    --mount "type=bind,src=$install_dir,dst=/opt/kalibr,readonly" \
    --mount "type=bind,src=$config_path,dst=/config/source-user.yaml,readonly" \
    --mount "type=bind,src=$machine_batch,dst=$container_batch" \
    --workdir "$container_batch" \
    --entrypoint /bin/bash \
    "$runtime_image" -lc '
      set -euo pipefail
      python3 -c '"'"'
import os
from pathlib import Path
import yaml

source = Path("/config/source-user.yaml")
target = Path("/tmp/kalibr-user.yaml")
value = yaml.safe_load(source.read_text(encoding="utf-8"))
if not isinstance(value, dict):
    raise SystemExit("user config must contain a YAML mapping")
value["machine_dir"] = os.environ["KALIBR_MACHINE_DIR"]
target.write_text(
    yaml.safe_dump(value, allow_unicode=True, sort_keys=False, width=120),
    encoding="utf-8",
)
'"'"'
      exec /opt/kalibr/bin/kalibr-cedarobot machine \
        --config /tmp/kalibr-user.yaml
    '
}

main() {
  require_command docker
  require_command realpath
  case "${1:---help}" in
    --help|-h)
      usage
      ;;
    build)
      [[ "$#" -le 2 ]] || fail "build accepts at most one BASE_IMAGE"
      build_image "${2:-}"
      ;;
    check)
      [[ "$#" -eq 2 ]] || fail "check requires INSTALL_DIR"
      check_install "$2"
      ;;
    machine)
      [[ "$#" -eq 4 ]] \
        || fail "machine requires INSTALL_DIR USER_CONFIG MACHINE_BATCH_DIR"
      run_machine "$2" "$3" "$4"
      ;;
    *)
      fail "unknown command: $1"
      ;;
  esac
}

main "$@"
