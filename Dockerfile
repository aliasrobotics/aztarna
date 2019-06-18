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

FROM osrf/ros2:nightly
# ARG AZTARNA_COMMIT=master
# ENV AZTARNA_COMMIT ${AZTARNA_COMMIT}

RUN apt-get -qq update && apt-get -qqy upgrade
# install aztarna build dependencies
RUN apt-get -qqy install build-essential cmake libgmp3-dev gengetopt libpcap-dev flex byacc libjson-c-dev pkg-config libunistring-dev wget unzip git
RUN apt-get -qqy install python3.7 python3.7-dev python3-pip
RUN apt-get -qqy install libxml2-dev libxslt1-dev
RUN apt-get -qqy install zlib1g-dev
RUN apt-get -qqy install libffi-dev
RUN apt-get -qqy install libssl-dev
RUN rm -rf /var/lib/apt/lists/*

# copy the aztarna files the FS and install it
COPY . /root/aztarna
# RUN cd /root/aztarna && git checkout ${AZTARNA_COMMIT} && python3.7 setup.py install
RUN cd /root/aztarna && python3 setup.py install
RUN echo "source /opt/ros/dashing/setup.bash" >> /root/.bashrc


ENTRYPOINT ["/root/aztarna/ros2_entrypoint.sh"]
