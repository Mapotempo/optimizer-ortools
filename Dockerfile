ARG ORTOOLS_VERSION=${ORTOOLS_VERSION}
ARG REGISTRY=${REGISTRY:-registry.mapotempo.com/}

# Install ORTools
FROM ${REGISTRY}mapotempo/ortools:${ORTOOLS_VERSION} as ortools

# Build wrapper
RUN apt-get update > /dev/null && \
  apt-get -y install git wget pkg-config build-essential cmake autoconf libtool zlib1g-dev lsb-release > /dev/null

ADD . /srv/optimizer-ortools

WORKDIR /srv/optimizer-ortools
RUN make tsp_simple

# Final image
FROM debian:latest

LABEL maintainer="Mapotempo <tech@mapotempo.com>"

COPY --from=ortools /srv/optimizer-ortools /srv/optimizer-ortools
COPY --from=ortools /srv/or-tools srv/or-tools
COPY --from=ortools /usr/lib/x86_64-linux-gnu/ /usr/lib/x86_64-linux-gnu/
