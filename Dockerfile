# 1.0.13 is the latest version containing bundler 2 required for optimizer-api
FROM phusion/passenger-ruby25:1.0.13

ARG ORTOOLS_URL=${ORTOOLS_URL}

LABEL maintainer="Mapotempo <tech@mapotempo.com>"

WORKDIR /srv/

# Trick to install passenger-docker on Ruby 2.5. Othwerwise `apt-get update` fails with a
# certificate error. See following links for explanantion:
# https://issueexplorer.com/issue/phusion/passenger-docker/325
# and
# https://issueexplorer.com/issue/phusion/passenger-docker/322
# Basically, DST Root CA X3 certificates are expired on Setember 2021 and apt-get cannot validate
# with the old certificates and the certification correction is only done for Ruby 2.6+ on the
# passenger-docker repo because Ruby 2.5 is EOL.
RUN mv /etc/apt/sources.list.d /etc/apt/sources.list.d.bak
RUN apt update && apt install -y ca-certificates
RUN mv /etc/apt/sources.list.d.bak /etc/apt/sources.list.d
# The above trick can be removed after Ruby version is increased.

RUN apt-get update > /dev/null && \
  apt-get -y install git wget pkg-config build-essential cmake autoconf libtool zlib1g-dev lsb-release > /dev/null

ADD . /srv/or-tools

RUN wget -qO- $ORTOOLS_URL | tar xz --strip-components=1 -C /srv/or-tools

ADD . /srv/optimizer-ortools

WORKDIR /srv/optimizer-ortools
RUN make tsp_simple
