FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    git cmake g++ make libzmq3-dev libeigen3-dev python3-pip pkg-config \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pyzmq

WORKDIR /app

RUN git clone --recursive https://github.com/montrealrobotics/vicon_data_server.git /app/vicon_data_server

RUN cd /app/vicon_data_server && git submodule update --init --recursive

RUN cd /app/vicon_data_server/external/cppzmq \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make -j$(nproc) \
    && make install

RUN find /usr/local -name "cppzmqConfig.cmake" \
    && mkdir -p /usr/local/lib/cmake/cppzmq \
    && mv $(find /usr/local -name "cppzmqConfig.cmake") /usr/local/lib/cmake/cppzmq/

RUN mkdir -p /app/vicon_data_server/build && cd /app/vicon_data_server/build && cmake .. && make -j$(nproc)

RUN ls -l /app/vicon_data_server/build/ViconDataServer

CMD ["/app/vicon_data_server/build/ViconDataServer", "172.19.0.61", "go1_v3", "remote"]

