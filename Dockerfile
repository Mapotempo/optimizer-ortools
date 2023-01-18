ARG RUBY_VERSION
ARG ARCHITECTURE

FROM ${ARCHITECTURE}ruby:${RUBY_VERSION}

ARG ORTOOLS_VERSION=${ORTOOLS_VERSION}

LABEL maintainer="Mapotempo <tech@mapotempo.com>"

RUN apt-get update > /dev/null && \
  apt install -y git build-essential cmake swig lsb-release python3-pip autoconf libtool zlib1g-dev > /dev/null && \
  git clone -b ${ORTOOLS_VERSION} https://github.com/google/or-tools /srv/or-tools

WORKDIR /srv/or-tools
RUN make third_party && \
  make cc

ADD . /srv/optimizer-ortools
WORKDIR /srv/optimizer-ortools
RUN rm -R /srv/or-tools/docs /srv/or-tools/examples && \
  make tsp_simple
