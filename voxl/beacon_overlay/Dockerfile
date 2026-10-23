# beacon_overlay: a ROS 2 Humble image + libtflite_gpu_shim.so, for beacon_overlay.py.
#
# Build context is voxl/ (run.sh build does this):
#   docker build -f beacon_overlay/Dockerfile --build-arg BASE=voxl2-humble -t beacon-overlay voxl/
#
# TFLite is built from source here (OpenCL GPU delegate on, XNNPACK on) and linked
# into the shim, so CPU and GPU inference both run this TF_VERSION whatever
# qrb5165-tflite the drone has. TF_VERSION matches the TensorFlow that exports the
# models (2.19.0), so op versions in the .tflite are always known to the runtime.
# The delegate dlopen()s the drone's Adreno libOpenCL.so at run time; run.sh
# passes it and the kgsl/ion device nodes in.
#
# BASE needs python3, numpy, cv2 and rclpy: voxl2-humble on the flight drone,
# seabird on the bench board. The repo, /data/modalai (calibration) and the model
# directory are mounted read-only at run time, so code edits need only a restart.
ARG BASE=voxl2-humble

FROM ${BASE} AS tflite
ARG TF_VERSION=2.19.0
# 8 cores / 7 GB on the VOXL2: more jobs than this runs out of memory on the GPU
# delegate sources.
ARG JOBS=4
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /opt/tensorflow \
    && curl -fsSL "https://github.com/tensorflow/tensorflow/archive/refs/tags/v${TF_VERSION}.tar.gz" \
       | tar xz -C /opt/tensorflow --strip-components=1
# TFLite itself first, in its own layer, against a placeholder shim source: an
# edit to tflite_gpu_shim.cc then only recompiles the shim, not TFLite (~1 h).
# KleidiAI off: XNNPACK fetches it as a GitLab archive whose SHA256 no longer
# matches the one pinned in 2.19 (GitLab regenerates archives), and it only
# accelerates int8/int4 matmuls -- these models are fp16/fp32.
COPY beacon_gpu/CMakeLists.txt /opt/beacon_gpu/
RUN touch /opt/beacon_gpu/tflite_gpu_shim.cc \
    && cmake -S /opt/beacon_gpu -B /opt/beacon_gpu/build -DCMAKE_BUILD_TYPE=Release \
        -DTENSORFLOW_SOURCE_DIR=/opt/tensorflow -DTFLITE_ENABLE_GPU=ON -DTFLITE_ENABLE_XNNPACK=ON \
        -DXNNPACK_ENABLE_KLEIDIAI=OFF \
    && cmake --build /opt/beacon_gpu/build -j "${JOBS}" --target tensorflow-lite
COPY beacon_gpu/tflite_gpu_shim.cc /opt/beacon_gpu/
RUN cmake --build /opt/beacon_gpu/build -j "${JOBS}" --target tflite_gpu_shim \
    && strip --strip-unneeded /opt/beacon_gpu/build/libtflite_gpu_shim.so \
    && nm -D --defined-only /opt/beacon_gpu/build/libtflite_gpu_shim.so | grep ' T bg_'

FROM ${BASE}
ARG TF_VERSION=2.19.0
LABEL tflite.version="${TF_VERSION}"
COPY --from=tflite /opt/beacon_gpu/build/libtflite_gpu_shim.so /opt/beacon_gpu/

# Install tools for building PX4 ROS 2 messages
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    ros-humble-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*

# Download PX4 1.14 message definitions
RUN mkdir -p /opt/px4_ws/src \
    && git clone --depth 1 --branch release/1.14 \
       https://github.com/PX4/px4_msgs.git \
       /opt/px4_ws/src/px4_msgs

# Build the message package
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /opt/px4_ws && \
    colcon build --packages-select px4_msgs"

# Expose PX4 message interfaces to ROS 2 and Python
ENV AMENT_PREFIX_PATH=/opt/px4_ws/install/px4_msgs:${AMENT_PREFIX_PATH}
ENV PYTHONPATH=/opt/px4_ws/install/px4_msgs/lib/python3.10/site-packages:${PYTHONPATH}
ENV LD_LIBRARY_PATH=/opt/px4_ws/install/px4_msgs/lib:${LD_LIBRARY_PATH}

RUN python3 -c "import numpy, cv2; print(numpy.__version__, cv2.__version__)"

WORKDIR /workspace
