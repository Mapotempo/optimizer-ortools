#!/bin/sh

case $ORTOOLS_VERSION in
  'v7.1')
    export ORTOOLS_URL="https://github.com/google/or-tools/releases/download/v7.1/or-tools_debian-9_v7.1.6720.tar.gz"
    ;;
  'v7.5')
    export ORTOOLS_URL="https://github.com/google/or-tools/releases/download/v7.5/or-tools_debian-10_v7.5.7466.tar.gz"
    ;;
  'v7.6')
    export ORTOOLS_URL="https://github.com/google/or-tools/releases/download/v7.6/or-tools_debian-10_v7.6.7691.tar.gz"
    ;;
  *)
    echo "Unknown OR-Tools version"
esac

