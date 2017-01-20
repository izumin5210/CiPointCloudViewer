#!/usr/bin/env bash
#
# Setup project

set -eu

version="0.1.0"

usage() {
  cat <<EOF
$(basename ${0}) - Setup this repository

Usage:
    $(basename ${0})

Options:
    --version, -v     print $(basename ${0}) version
    --help, -h        print this
EOF
}

version() {
  echo "$(basename ${0}) version ${version}"
}

setup() {
  git submodule update --init --update

  mkdir framework/build
  cd framework/build
  cmake ..
  make -j$(getconf _NPROCESSORS_ONLN)
}

while [ $# -gt 0 ];
do
  case ${1} in
    --debug|-d)
      set -x
    ;;

    --version|-v)
      version
      exit 0
    ;;

    --help|-h)
      usage
      exit 0
    ;;

    *)
      setup
      exit 0
    ;;
  esac
  shift
done
