## start with 18.04 bionic to match 865 image
FROM  ubuntu:18.04

ENV BRANCH=dev

# Hack to fix voxl-feature-tracker run which was looking for libvoxl_common_config.so in /usr/lib64
ENV LD_LIBRARY_PATH="/usr/lib:/usr/lib64"

# Baseline Dependencies
RUN apt update && apt upgrade -y
RUN apt install -y build-essential make cmake sudo curl unzip gcc wget git nano vim xterm libgflags-dev libgoogle-glog-dev net-tools
RUN apt install -y libz-dev libpng-dev

# Create workspace
RUN mkdir -p /home/dependency-workspace
WORKDIR /home/dependency-workspace

# Clone and compile first set of dependencies
RUN git clone -b ${BRANCH} https://gitlab.com/voxl-public/voxl-sdk/core-libs/libmodal-json.git
RUN cd libmodal-json && ./build.sh native && cd build/ && make install && cd ../../

RUN git clone -b ${BRANCH} https://gitlab.com/voxl-public/voxl-sdk/third-party/voxl-mavlink.git
RUN cd voxl-mavlink && \
    git submodule update --init --recursive && \
    ./make_package.sh deb && \
    cp -R pkg/data/usr/include/* /usr/include/ && \ 
    cd ../

RUN git clone -b dev https://gitlab.com/voxl-public/voxl-sdk/core-libs/libmodal-pipe.git
RUN cd libmodal-pipe && ./build.sh native && cd build/ && make install && cd ../../

RUN git clone -b ${BRANCH} https://gitlab.com/voxl-public/voxl-sdk/core-libs/librc-math.git
RUN cd librc-math && ./build.sh native && cd build/ && make install && cd ../../

RUN git clone -b ${BRANCH} https://gitlab.com/voxl-public/voxl-sdk/core-libs/libvoxl-cutils.git
RUN cd libvoxl-cutils && ./build.sh native && cd build/ && make install && cd ../../

#RUN git clone -b ${BRANCH} https://gitlab.com/voxl-public/voxl-sdk/third-party/voxl-opencv.git
RUN git clone -b dev https://gitlab.com/voxl-public/voxl-sdk/third-party/voxl-opencv.git
RUN cd voxl-opencv && git submodule update --init --recursive && ./build.sh native && cd build/ && make install && cd ../../
# Hack to work with existing opencv dependencies, should really be cleaned in actual projects CMake files though
RUN mkdir -p /usr/lib64 && cp /usr/lib/x86_64-linux-gnu/libopencv*.so* /usr/lib64/

RUN git clone -b ${BRANCH} https://gitlab.com/voxl-public/voxl-sdk/utilities/voxl-mpa-tools.git
RUN cd voxl-mpa-tools && ./build.sh native && cd build/ && make install && cd ../../
RUN cp /home/dependency-workspace/voxl-mpa-tools/scripts/* /usr/bin/

#RUN git clone -b ${BRANCH} https://gitlab.com/voxl-public/voxl-sdk/third-party/voxl-jpeg-turbo.git
RUN git clone -b dev https://gitlab.com/voxl-public/voxl-sdk/third-party/voxl-jpeg-turbo.git
RUN cd voxl-jpeg-turbo && \
    git submodule update --init --recursive && \
    ./build.sh native && \ 
    cd build/ && \ 
    make install && cd ../../

RUN git clone -b ${BRANCH} https://gitlab.com/voxl-public/voxl-sdk/utilities/voxl-logger.git
RUN cd voxl-logger && ./build.sh native && cd build/ && make install && cd ../../

RUN git clone -b dev https://gitlab.com/voxl-public/voxl-sdk/services/voxl-cpu-monitor.git
RUN cd voxl-cpu-monitor && ./build.sh native && cd build/ && make install && cd ../../

RUN git clone -b ${BRANCH} https://gitlab.com/voxl-public/voxl-sdk/third-party/voxl-mongoose.git
RUN cd voxl-mongoose && \
    git submodule update --init --recursive && \
    ./build.sh native && \ 
    cd build/ && \
    make install && \
    cd ../../

RUN git clone -b dev https://gitlab.com/voxl-public/voxl-sdk/services/voxl-portal.git
RUN cd voxl-portal && ./build.sh native && cd build/ && make install && cd ../../

RUN git clone -b ${BRANCH} https://gitlab.com/voxl-public/voxl-sdk/third-party/voxl-eigen3.git
RUN cd voxl-eigen3 && ./build.sh native && cd build64/ && make install && cd ../../

RUN git clone -b ${BRANCH} https://gitlab.com/voxl-public/voxl-sdk/core-libs/libmodal-journal.git
RUN cd libmodal-journal && ./build.sh native && cd build/ && make install && cd ../../