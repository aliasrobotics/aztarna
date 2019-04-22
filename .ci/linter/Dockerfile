FROM ubuntu:bionic

LABEL maintainer="Lander Usategui <lander at aliasrobotics dot com>"

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
WORKDIR /root/aztarna

RUN \
     echo 'Etc/UTC' > /etc/timezone \
     && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
     && apt-get -qq update && apt install -y \
     python3.7 python3.7-dev python3-pip pylint \
     && rm -rf /var/lib/apt/lists/*
COPY docker-entrypoint.sh /
ENTRYPOINT ["/docker-entrypoint.sh"]

