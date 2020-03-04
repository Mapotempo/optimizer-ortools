# Building images

```
apt-get -y install git wget pkg-config build-essential cmake autoconf libtool zlib1g-dev lsb-release > /dev/null
export REGISTRY='registry.mapotempo.com/'
```

## Required images
Optimizer requires the two following images that must be manually built.

### Ortools

```
export ORTOOLS_VERSION=v7.0
docker build --build-arg ORTOOLS_VERSION=${ORTOOLS_VERSION} \
  -f ./docker/ortools/Dockerfile -t ${REGISTRY}mapotempo/ortools:${ORTOOLS_VERSION} .
```

## Build
```
export ORTOOLS_VERSION=v7.0
export BRANCH=${BRANCH:-beta}
docker build --build-arg ORTOOLS_VERSION=${ORTOOLS_VERSION} \
  --build-arg BRANCH=${BRANCH} \
  -f ./Dockerfile -t ${REGISTRY}mapotempo-${BRANCH}/optimizer-ortools:latest .
```