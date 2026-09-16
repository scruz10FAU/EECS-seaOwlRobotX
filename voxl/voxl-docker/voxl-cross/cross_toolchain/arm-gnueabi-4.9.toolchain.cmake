
# set(CMAKE_SYSTEM_NAME Linux)
# set(CMAKE_SYSTEM_PROCESSOR arm)

# # ubuntu cross compiler installs straight to /usr/bin/
# set(tools /usr)
# set(CMAKE_C_COMPILER ${tools}/bin/arm-linux-gnueabi-gcc-4.9)
# set(CMAKE_CXX_COMPILER ${tools}/bin/arm-linux-gnueabi-g++-4.9)

# ## glibc 2.23 is off in a separate location in voxl-cross to avoid conflicting
# ## with the new gcc7 compiler stuff which is in the default location
# set(LOC /usr/arm-linux-gnueabi-2.23)
# set(CMAKE_C_FLAGS   "-L${LOC} -I${LOC}/include ${CMAKE_C_FLAGS}")
# set(CMAKE_CXX_FLAGS "-L${LOC} -I${LOC}/include ${CMAKE_CXX_FLAGS}")

message(WARN " you're using a deprecated toolchain, please use the new toolchains:")
message(WARN " qrb5165_ubun1_18.04_arm32.toolchain.cmake")
message(WARN " qrb5165_ubun2_20.04_arm32.toolchain.cmake")
include(/opt/cross_toolchain/qrb5165_ubun1_18.04_arm32.toolchain.cmake)
