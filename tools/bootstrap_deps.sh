#!/usr/bin/env bash
set -euo pipefail

readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly project_root="$(cd "${script_dir}/.." && pwd)"
readonly deps_root="${project_root}/.deps"

install_python_runtime() {
  local destination="${deps_root}/python"
  local staging
  mkdir -p "${deps_root}"
  staging="$(mktemp -d "${deps_root}/python.tmp.XXXXXX")"
  trap 'rm -rf "${staging}"' RETURN

  # NumPy must match the host OpenCV/Boost.Python ABI. Do not install a second
  # NumPy wheel into the private directory as a transitive bag dependency.
  python3 -c 'import numpy, cv2, scipy, yaml, matplotlib'
  python3 -m pip install \
    --disable-pip-version-check \
    --no-cache-dir \
    --no-deps \
    --target "${staging}" \
    'rosbags==0.9.23'
  # Resolve these packages normally: older ruamel.yaml versions used on
  # Python 3.8 require ruamel.yaml.clib on CPython. Only rosbags resolution
  # is disabled above, to keep its NumPy dependency supplied by the host.
  python3 -m pip install \
    --disable-pip-version-check \
    --no-cache-dir \
    --target "${staging}" \
    'lz4>=4.3.2,<5' \
    'ruamel.yaml>=0.18.6,<0.20' \
    'zstandard>=0.21,<0.26'
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="${staging}" python3 -c \
    'import rosbags.rosbag1, rosbags.rosbag2, lz4.frame, ruamel.yaml, zstandard; assert ruamel.yaml.YAML(typ="safe").load("ok: true") == {"ok": True}'

  rm -rf "${destination}"
  mv "${staging}" "${destination}"
  trap - RETURN
  printf 'Python runtime: %s\n' "${destination}"
}

install_ubuntu_2204_runtime() {
  local destination="${deps_root}/sysroot"
  local staging package_root candidate package archive
  local -a packages=(
    libamd2 libcamd2 libccolamd2 libcholmod3 libcolamd2
    libmetis-dev libmetis5 libspqr2 libsuitesparse-dev libsuitesparseconfig5
    python3-igraph python3-texttable
  )

  mkdir -p "${deps_root}"
  package_root="$(mktemp -d "${deps_root}/packages.tmp.XXXXXX")"
  staging="$(mktemp -d "${deps_root}/sysroot.tmp.XXXXXX")"
  trap 'rm -rf "${staging}" "${package_root}"' RETURN
  for package in "${packages[@]}"; do
    candidate="$(apt-cache policy "${package}" | awk '/Candidate:/ { print $2 }')"
    if [[ -z "${candidate}" || "${candidate}" == "(none)" ]]; then
      printf 'No Ubuntu 22.04 candidate is available for %s.\n' \
        "${package}" >&2
      return 1
    fi
    (cd "${package_root}" && apt-get download "${package}=${candidate}")
  done

  for archive in "${package_root}"/*.deb; do
    dpkg-deb --extract "${archive}" "${staging}"
  done
  # The development package supplies headers for all SuiteSparse components.
  # Keep only the shared-library closure used by CHOLMOD/SPQR and Kalibr.
  python3 - "${staging}" <<'PY'
from pathlib import Path
import shutil
import sys

root = Path(sys.argv[1])
keep = {"amd", "camd", "ccolamd", "cholmod", "colamd", "metis", "spqr",
        "suitesparseconfig"}
for libdir in (root / "usr/lib").glob("*-linux-gnu"):
    for path in libdir.iterdir():
        if path.name.startswith("lib"):
            if ".so" in path.name:
                retain = path.name[3:].split(".so")[0] in keep
            else:
                retain = False
            if not retain:
                path.unlink()
    for name in keep:
        if not (libdir / ("lib" + name + ".so")).exists():
            raise RuntimeError("missing SuiteSparse dependency: " + name)
headers = root / "usr/include"
for path in headers.glob("suitesparse/umfpack*"):
    path.unlink()
for name in ("Mongoose.hpp", "suitesparse/btf.h", "suitesparse/cs.h",
             "suitesparse/klu.h", "suitesparse/ldl.h", "suitesparse/RBio.h",
             "suitesparse/SLIP_LU.h"):
    path = headers / name
    if path.exists():
        path.unlink()
for package in (root / "usr/share/doc").iterdir():
    for path in package.iterdir():
        if path.name != "copyright":
            if path.is_dir():
                shutil.rmtree(path)
            else:
                path.unlink()
shutil.rmtree(root / "usr/share/lintian", ignore_errors=True)
PY
  rm -rf "${destination}"
  mv "${staging}" "${destination}"
  rm -rf "${package_root}"
  trap - RETURN
  printf 'Ubuntu 22.04 runtime: %s\n' "${destination}"
}

install_python_runtime

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
fi

case "${ID:-unknown}:${VERSION_ID:-unknown}" in
  ubuntu:22.04)
    install_ubuntu_2204_runtime
    ;;
  ubuntu:20.04)
    printf 'Ubuntu 20.04 uses the host C++ dependencies.\n'
    ;;
  *)
    printf 'No private C++ dependency bootstrap for %s %s; using host libraries.\n' \
      "${ID:-unknown}" "${VERSION_ID:-unknown}"
    ;;
esac

printf 'Dependency bootstrap complete. No host package was installed.\n'
