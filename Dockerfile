####
# A Docker container for running aztarna,
#   a tool for robot footprinting
#
# To build, beware of caching and:
#
#   * If you wish to build current master
#
#        docker build -t aztarna_docker .
#
#   * If you wish to build a specific commit, use the AZTARNA_COMMIT build argument.
#
#        docker build -t aztarna_docker --build-arg AZTARNA_COMMIT=<your commit> .
#
# To run:
#
#     docker run  -it --rm --net=host aztarna_docker <aztarna args>
####

FROM ros:dashing
# ARG AZTARNA_COMMIT=master
# ENV AZTARNA_COMMIT ${AZTARNA_COMMIT}

RUN \
    echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections \
    && apt-get -qq update && apt-get -qqy upgrade \
    && apt-get -qqy install \
      libgmp3-dev gengetopt \
      libpcap-dev flex byacc \
      libjson-c-dev unzip \
      libunistring-dev wget \
      libxml2-dev libxslt1-dev \
      libffi-dev libssl-dev \
      tshark \
    && rm -rf /var/lib/apt/lists/*

# copy the aztarna files the FS and install it
COPY . /root/aztarna
RUN cd /root/aztarna && python3 setup.py install


ENTRYPOINT ["/root/aztarna/ros2_entrypoint.sh"]
