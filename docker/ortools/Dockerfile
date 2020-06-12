# Install ORTools
FROM debian:latest as ortools

ARG ORTOOLS_LINK=${ORTOOLS_LINK}

RUN wget ${ORTOOLS_LINK} /srv/or-tools

FROM debian:latest
LABEL maintainer="Mapotempo <contact@mapotempo.com>"

COPY --from=ortools /srv/or-tools/Makefile* srv/or-tools/
COPY --from=ortools /srv/or-tools/Version.txt srv/or-tools/Version.txt
COPY --from=ortools /srv/or-tools/makefiles srv/or-tools/makefiles
COPY --from=ortools /srv/or-tools/lib srv/or-tools/lib
COPY --from=ortools /srv/or-tools/ortools srv/or-tools/ortools
COPY --from=ortools /srv/or-tools/dependencies srv/or-tools/dependencies
COPY --from=ortools /usr/lib/x86_64-linux-gnu/ /usr/lib/x86_64-linux-gnu/
