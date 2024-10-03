FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libpcl-dev \
    libvtk9-dev \
    && apt-get clean

WORKDIR /root

RUN git clone https://github.com/furo-org/p2o.git

WORKDIR /root/p2o
RUN mkdir build && cd build && cmake .. && make -j$(nproc)

CMD ["/root/p2o/build/p2o_viewer/p2o_viewer"]

