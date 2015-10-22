mapotempo-optimizer
===================

Compute an optimized solution to the Vehicle Routing Problem with Time Windows using OR-Tools through an HTTP API.

Installation
============

## Optimizer

Require OR-Tools for the C++ part. Fetch source code at [https://github.com/google/or-tools](https://github.com/google/or-tools)

Compile the C++ optimizer

    cd optimizer
    make tsp_simple

## Ruby server

Install ruby from system package.
Install ruby bundle gem by :

    export GEM_HOME=~/.gem/ruby/2.2.0
    gem install bundler

Now add gem bin directory to path with :

    export PATH=$PATH:~/.gem/ruby/2.2.0/bin

And finally install gem project dependencies with :

    cd server
    bundle install

Configuration
=============

Review the content of server default values in server/config.ru.

Running
=======

Start standalone app server with

    cd server
    ruby config.ru

Now API is on [http://localhost:4567](http://localhost:4567)
