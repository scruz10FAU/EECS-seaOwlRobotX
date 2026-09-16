## This is the recommended toolchain for building for 865/QRB5165 Ubun2.0 platforms
## in a cross-compile environment like voxl-cross
# set(CMAKE_SYSTEM_NAME Linux)
# set(CMAKE_SYSTEM_PROCESSOR aarch64)

# # ubuntu cross compiler installs straight to /usr/bin/
# set(tools /usr)
# set(CMAKE_C_COMPILER ${tools}/bin/aarch64-linux-gnu-gcc-9)
# set(CMAKE_CXX_COMPILER ${tools}/bin/aarch64-linux-gnu-g++-9)

# # set architecture
# set(CMAKE_C_FLAGS   "-march=armv8.2-a ${CMAKE_C_FLAGS}")
# set(CMAKE_CXX_FLAGS "-march=armv8.2-a ${CMAKE_CXX_FLAGS}")

message(WARN " you're using a deprecated toolchain, please use the new toolchains:")
message(WARN " /opt/cross_toolchain/qrb5165_ubun1_18.04_aarch64.toolchain.cmake")
message(WARN " /opt/cross_toolchain/qrb5165_ubun2_20.04_aarch64.toolchain.cmake")
include(/opt/cross_toolchain/qrb5165_ubun2_20.04_aarch64.toolchain.cmake)
