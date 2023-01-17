ARG RUBY_VERSION
ARG ARCHITECTURE

FROM ${ARCHITECTURE}ruby:${RUBY_VERSION}

ARG ORTOOLS_VERSION=${ORTOOLS_VERSION}

LABEL maintainer="Mapotempo <tech@mapotempo.com>"

RUN apt-get update > /dev/null && \
  apt install -y git build-essential cmake swig lsb-release python3-pip autoconf libtool zlib1g-dev > /dev/null && \
  git clone -b ${ORTOOLS_VERSION} https://github.com/google/or-tools /srv/or-tools

# tmp until ruby image have the good cmake version
RUN curl 'https://github.com/Kitware/CMake/releases/download/v3.15.7/cmake-3.15.7.tar.gz' -s -L -R -o cmake.tar.gz && \
  tar -zxvf cmake.tar.gz && \
  cd cmake-3.15.7 && \
  ./bootstrap && \
  make && \
  make install && \
  cd .. && \
  rm -R cmake-3.15.7 cmake.tar.gz

WORKDIR /srv/or-tools
RUN /usr/local/bin/cmake -S . -B build -DBUILD_DEPS=ON -DBUILD_PYTHON=ON && \
  /usr/local/bin/cmake --build build --config Release --target install -v

ADD . /srv/optimizer-ortools

WORKDIR /srv/optimizer-ortools
RUN make tsp_simple && \
  cd .. && \
  rm -R /srv/or-tools
