#!/usr/bin/env bash

set -e

if [[ $GITHUB_REF == *tags* ]]; then
  echo "> Building image for tag purpose (${GITHUB_REF}) with or-tools ${ORTOOLS_VERSION}"
  TAG=${GITHUB_REF#refs/tags/}
else
  echo "> Building image for dev purpose (${REPOSITORY}) with or-tools ${ORTOOLS_VERSION}"
  TAG=$(echo "${REPOSITORY}" | sed "s/\/.*//")
  REGISTRY=${REGISTRY:-registry.test.com}
fi

IMAGE_NAME=${REGISTRY}/mapotempo-ce/optimizer-ortools:${TAG}

docker build --build-arg RUBY_VERSION="2.5" --build-arg ORTOOLS_VERSION="${ORTOOLS_VERSION}" -f ./Dockerfile -t "${IMAGE_NAME}" .
docker run -d --name optimizer -t "${IMAGE_NAME}"
docker exec -i optimizer bash -c "LD_LIBRARY_PATH=/srv/or-tools/lib/ /srv/optimizer-ortools/tsp_simple -time_limit_in_ms 500 -intermediate_solutions -instance_file '/srv/optimizer-ortools/data/test_ortools_single_route_with_route_order' -solution_file '/tmp/optimize-or-tools-output'"

echo "::set-output name=image_name::${IMAGE_NAME}"
