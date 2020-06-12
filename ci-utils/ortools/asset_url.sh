#!/bin/sh
echo $ORTOOLS_VERSION
if [ $ORTOOLS_VERSION = 'v7.1' ]; then
  export ORTOOLS_URL="https://github.com/google/or-tools/releases/download/v7.1/or-tools_debian-9_v7.1.6720.tar.gz";
elif [ $ORTOOLS_VERSION = 'v7.5' ]; then
  export ORTOOLS_URL="https://github.com/google/or-tools/releases/download/v7.5/or-tools_debian-10_v7.5.7466.tar.gz";
elif [ $ORTOOLS_VERSION = 'v7.6' ]; then
  export ORTOOLS_URL="https://github.com/google/or-tools/releases/download/v7.6/or-tools_debian-10_v7.6.7691.tar.gz";
else
  echo "Unknown OR-Tools version"
fi
