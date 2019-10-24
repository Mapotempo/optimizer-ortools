optimizer-ortools
=================

Compute an optimized solution to the Vehicle Routing Problem with Time Windows and various constraints using OR-Tools.
This wrapper is designed to be called through [Optimizer-API](https://github.com/Mapotempo/optimizer-api) and has been tested on Ubuntu 17.10, 18.04. Linux Mint 18, Debian 8.

Installation
============
## Requirements

Require OR-Tools for the C++ part. Fetch source code at [https://github.com/google/or-tools](https://github.com/google/or-tools).

The current implementation has been tested with the version 7.0 of OR-Tools

    git clone git@github.com:google/or-tools.git
    git fetch
    git checkout tags/v7.1 -b v7.1

    sudo apt-get install git bison flex python-setuptools python-dev autoconf \
    libtool zlib1g-dev texinfo help2man gawk g++ curl texlive cmake subversion

    make third_party

    make cc

More details on [Google Optimization Tools Documentation](https://developers.google.com/optimization/introduction/installing)


## Optimizer

Compile the C++ optimizer

    make tsp_simple


Test
====

LD_LIBRARY_PATH=../or-tools/dependencies/install/lib/:../or-tools/lib/ ../optimizer-ortools/tsp_simple  -time_limit_in_ms 239994 -init_duration 11926 -time_out_multiplier 2 -intermediate_solutions -instance_file 'data/49Missions_7Vehicles_VRP2TW' -solution_file '/tmp/optimize-or-tools-output20180612-5826-8ji7pc'

Dev
===

After dev the code shall be formatted according to the style file of the project with the following command:

clang-format -i *.cc *.h
