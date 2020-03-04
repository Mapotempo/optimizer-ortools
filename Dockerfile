ARG ORTOOLS_VERSION=${ORTOOLS_VERSION:-latest}
ARG REGISTRY=${REGISTRY:-registry.mapotempo.com/}

# Install ORTools
FROM ${REGISTRY}mapotempo/ortools:${ORTOOLS_VERSION} as optimizer-ortools
ARG OPTIMIZER_ORTOOLS_VERSION

# Build wrapper
RUN apt-get update > /dev/null && \
  apt-get -y install git wget pkg-config build-essential cmake autoconf libtool zlib1g-dev lsb-release > /dev/null

ADD . /srv/optimizer-ortools

WORKDIR /srv/optimizer-ortools
RUN make tsp_simple

# Final image
FROM debian:latest

LABEL maintainer="Mapotempo <tech@mapotempo.com>"

COPY --from=optimizer-ortools /srv/optimizer-ortools /srv/optimizer-ortools

WORKDIR /srv/optimizer-ortools
