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

  python3 -m pip install \
    --disable-pip-version-check \
    --target "${staging}" \
    --upgrade \
    'rosbags>=0.9.20,<0.10'

  rm -rf "${destination}"
  mv "${staging}" "${destination}"
  trap - RETURN
  printf 'Python runtime: %s\n' "${destination}"
}

install_ubuntu_2204_runtime() {
  local package_root="${deps_root}/packages/ubuntu-22.04"
  local destination="${deps_root}/sysroot"
  local staging candidate package archive
  local -a archives
  local -a packages=(
    libamd2 libbtf1 libcamd2 libccolamd2 libcholmod3 libcolamd2
    libcxsparse3 libklu1 libldl2 libmetis-dev libmetis5 libmongoose2
    python3-igraph python3-texttable librbio2 libsliplu1 libspqr2
    libsuitesparse-dev libsuitesparseconfig5 libumfpack5
  )

  mkdir -p "${package_root}" "${deps_root}"
  for package in "${packages[@]}"; do
    candidate="$(apt-cache policy "${package}" | awk '/Candidate:/ { print $2 }')"
    if [[ -z "${candidate}" || "${candidate}" == "(none)" ]]; then
      printf 'No Ubuntu 22.04 candidate is available for %s.\n' \
        "${package}" >&2
      return 1
    fi
    archives=("${package_root}/${package}_"*.deb)
    if [[ ! -e "${archives[0]}" ]]; then
      (cd "${package_root}" && apt-get download "${package}=${candidate}")
    fi
  done

  staging="$(mktemp -d "${deps_root}/sysroot.tmp.XXXXXX")"
  trap 'rm -rf "${staging}"' RETURN
  for archive in "${package_root}"/*.deb; do
    dpkg-deb --extract "${archive}" "${staging}"
  done
  rm -rf "${destination}"
  mv "${staging}" "${destination}"
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
