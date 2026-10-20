# libtflite_gpu_shim.so: TFLite (built from source, OpenCL GPU delegate on) +
# tflite_gpu_shim.cc in one shared library. Built inside the beacon_overlay image
# (see ../beacon_overlay/Dockerfile), so the TFLite version is pinned there rather
# than taken from the drone's qrb5165-tflite package.
#
#   cmake -S . -B build -DTENSORFLOW_SOURCE_DIR=/path/to/tensorflow -DTFLITE_ENABLE_GPU=ON
#   cmake --build build -j4 --target tflite_gpu_shim
cmake_minimum_required(VERSION 3.16)
project(tflite_gpu_shim C CXX)

set(TENSORFLOW_SOURCE_DIR "" CACHE PATH "TensorFlow source tree (the one containing tensorflow/lite)")
if(NOT EXISTS "${TENSORFLOW_SOURCE_DIR}/tensorflow/lite/CMakeLists.txt")
  message(FATAL_ERROR "TENSORFLOW_SOURCE_DIR must point at a TensorFlow checkout")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Every TFLite static archive ends up inside a shared library.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory("${TENSORFLOW_SOURCE_DIR}/tensorflow/lite"
                 "${CMAKE_CURRENT_BINARY_DIR}/tensorflow-lite" EXCLUDE_FROM_ALL)

add_library(tflite_gpu_shim SHARED tflite_gpu_shim.cc)
# absl log: TF 2.19's compiler/mlir/lite/core/api/flatbuffer_conversions.cc uses
# LOG(), but the tensorflow-lite target doesn't link absl's logging library, so
# a shared library (--no-undefined) fails with undefined LogMessage symbols.
target_link_libraries(tflite_gpu_shim PRIVATE tensorflow-lite absl::log absl::log_internal_message)
# Only bg_* is exported: --exclude-libs keeps TFLite's own symbols private, so
# nothing clashes with another TFLite in the same process. --no-undefined turns a
# missing dependency into a build error, not a load error.
set_target_properties(tflite_gpu_shim PROPERTIES CXX_VISIBILITY_PRESET hidden)
target_link_options(tflite_gpu_shim PRIVATE -Wl,--exclude-libs,ALL -Wl,--no-undefined)
