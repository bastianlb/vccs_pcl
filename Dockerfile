FROM pointcloudlibrary/env:20.04

RUN apt update && apt install -y git

RUN git clone https://github.com/PointCloudLibrary/pcl.git /home

RUN cd /home/ && mkdir build && cd build && cmake .. \
  && make -j$(nproc) && make -j$(nproc) install
