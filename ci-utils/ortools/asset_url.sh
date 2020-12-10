#!/bin/sh

case $ORTOOLS_VERSION in
  'v7.1')
    export ORTOOLS_URL="https://github.com/google/or-tools/releases/download/v7.1/or-tools_debian-9_v7.1.6720.tar.gz"
    ;;
  'v7.4')
    export ORTOOLS_URL="https://github.com/google/or-tools/releases/download/v7.4/or-tools_debian-9_v7.4.7247.tar.gz"
    ;;
  'v7.5')
    export ORTOOLS_URL="https://github.com/google/or-tools/releases/download/v7.5/or-tools_debian-10_v7.5.7466.tar.gz"
    ;;
  'v7.8')
    export ORTOOLS_URL="https://github.com/google/or-tools/releases/download/v7.8/or-tools_debian-10_v7.8.7959.tar.gz"
    ;;
  *)
    echo "Unknown OR-Tools version"
esac

