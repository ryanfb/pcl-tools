FROM ubuntu:jammy

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y cmake build-essential pcl-tools libpcl-dev

ADD . /pcl-tools
WORKDIR /pcl-tools

RUN cmake . && make
