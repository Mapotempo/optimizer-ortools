ARG ORTOOLS_URL=${ORTOOLS_URL}
ARG REGISTRY=${REGISTRY:-registry.mapotempo.com/}

# Final image
# Build wrapper
FROM debian:latest
ARG ORTOOLS_URL

WORKDIR /srv/

RUN apt-get update > /dev/null && \
  apt-get -y install git wget pkg-config build-essential cmake autoconf libtool zlib1g-dev lsb-release > /dev/null

ADD . /srv/or-tools

RUN wget -qO- $ORTOOLS_URL | tar xz --strip-components=1 -C /srv/or-tools

ADD . /srv/optimizer-ortools

WORKDIR /srv/optimizer-ortools
RUN make tsp_simple

LABEL maintainer="Mapotempo <tech@mapotempo.com>"
