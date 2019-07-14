FROM ubuntu:bionic

LABEL maintainer="Lander Usategui <lander at aliasrobotics dot com>"

WORKDIR /home/root/aztarna
COPY ./docker-entrypoint.sh /

RUN \
    apt update -qq && apt install -y pep8 \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT /docker-entrypoint.sh
