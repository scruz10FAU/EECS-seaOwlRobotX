## This is the recommended toolchain for building for QCS6490 ubun 24.04
## in a cross-compile environment like voxl-cross
## This targets glibc 2.39 and CXXABI_1.3.15

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER_TARGET aarch64-linux-gnu)
set(CMAKE_CXX_COMPILER_TARGET aarch64-linux-gnu)
set(TARGET_SYSROOT /opt/sysroots/qcs6490)
set(CMAKE_CROSSCOMPILING ON CACHE BOOL "Cross-compiling" FORCE)

# gcc 13 is the latest on noble.
set(tools /usr)
set(CMAKE_C_COMPILER ${tools}/bin/aarch64-linux-gnu-gcc-13 CACHE STRING "" FORCE)
set(CMAKE_CXX_COMPILER ${tools}/bin/aarch64-linux-gnu-g++-13 CACHE STRING "" FORCE)

# tell gcc where our sysroot is.
set(CMAKE_SYSROOT_COMPILE ${TARGET_SYSROOT})
set(CMAKE_SYSROOT_LINK    ${TARGET_SYSROOT})

# look for things in sysroot first, then host sysroot, except progrmas like make/ar
set(CMAKE_FIND_ROOT_PATH ${TARGET_SYSROOT} / )
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)


# the TARGET_SYSROOT option adds some sysroot include dirs at the END of gcc's
# search path but we want to force gcc to use the same headers and libs as the
# sysroot so we specify them here in the same order that the on-target compiler
# users. the `=` symbol is important, it means that directory is relative to
# the sysroot. These directories in this order are determined by running the
# following commands on target and duplicating the search paths.
# gcc -E -x c - -v < /dev/null
# g++ -E -x c++ - -v < /dev/null
# Also add the host /usr/include AFTER these just to find things installed with
# install_build_deps.sh
set(CMAKE_C_FLAGS_INIT "\
-isystem=/usr/lib/gcc/aarch64-linux-gnu/13/include \
-isystem=/usr/local/include \
-isystem=/usr/include/aarch64-linux-gnu \
-isystem=/usr/include \
-idirafter /usr/include")

set(CMAKE_CXX_FLAGS_INIT "\
-isystem=/usr/include/c++/13 \
-isystem=/usr/include/aarch64-linux-gnu/c++/13 \
-isystem=/usr/include/c++/13/backward \
-isystem=/usr/lib/gcc/aarch64-linux-gnu/13/include \
-isystem=/usr/local/include \
-isystem=/usr/include/aarch64-linux-gnu \
-isystem=/usr/include \
-idirafter /usr/include")


# make sure the linker can find libs in the sysroot and host os
# order matters. paths came from inspecting the following output on target
# voxl2:/tmp$ aarch64-linux-gnu-g++ -E -x c++ - -v < /dev/null
# most are duplicates or resolve to the same realpath
# note the -B argument is to point to Scrt1.o crti.o and crtn.o
# crtbeginS.o and crtendS.o are still pulled from host cross compiler
# the qcm6490 directory contains some camera stuff
set(COMMON_LINKER_FLAGS "\
-B${TARGET_SYSROOT}/usr/lib/ \
-L${TARGET_SYSROOT}/usr/lib/gcc/aarch64-linux-gnu/13/ \
-L${TARGET_SYSROOT}/usr/lib/aarch64-linux-gnu/ \
-L${TARGET_SYSROOT}/usr/lib/ \
-L${TARGET_SYSROOT}/lib/aarch64-linux-gnu/ \
-L${TARGET_SYSROOT}/lib/ \
-L${TARGET_SYSROOT}/usr/lib/qcm6490/ \
-L/usr/lib64 \
-L/usr/lib")

# Apply to each kind of linker
set(CMAKE_EXE_LINKER_FLAGS_INIT    ${COMMON_LINKER_FLAGS})
set(CMAKE_SHARED_LINKER_FLAGS_INIT ${COMMON_LINKER_FLAGS})
set(CMAKE_MODULE_LINKER_FLAGS_INIT ${COMMON_LINKER_FLAGS})


# Qualcomm QCS6490 cpu is based on ARM A78 and A55 cores (armv8.2-a)
# gcc8 added warnings for addressing packed struct members which is really
# annoying since mavlink does this and we use mavlink everywhere so supress it.
# TODO try some ceres benchmarking with -march=armv8.2-a+dotprod+crypto -mtune=cortex-a78
set(CMAKE_C_FLAGS_INIT "\
${CMAKE_C_FLAGS_INIT} \
-march=armv8.2-a \
-Wno-address-of-packed-member")

set(CMAKE_CXX_FLAGS_INIT "\
${CMAKE_CXX_FLAGS_INIT} \
-march=armv8.2-a \
-Wno-address-of-packed-member")

# helpers to find opencv libraries in the sysroot
file(GLOB OpenCV_LIBS "${TARGET_SYSROOT}/usr/lib/aarch64-linux-gnu/libopencv*.so")
set(OpenCV_INCLUDE_DIRS "${TARGET_SYSROOT}/usr/include/opencv4")

# QRB5165 used to install 64-bit libraries to lib64 & 32-bit libs to /usr/lib
# Starting with qcs6490 we are done with 32-bit binaries now.
set(LIB_INSTALL_DIR /usr/lib)
set(PLATFORM "QCS6490")
add_definitions(-DPLATFORM_QCS6490)



## print some stats once
if(NOT DEFINED _MY_TOOLCHAIN_INITIALIZED)
set(_MY_TOOLCHAIN_INITIALIZED TRUE)
message(STATUS "---------------------------------------------------------")
message(STATUS "Using voxl-cross 64-bit toolchain for QCS6490 ubun24.04")
message(STATUS "C Compiler  : ${CMAKE_C_COMPILER}")
message(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Sysroot     : ${TARGET_SYSROOT}")
message(STATUS "C flags     : ${CMAKE_C_FLAGS_INIT}")
message(STATUS "CXX flags   : ${CMAKE_CXX_FLAGS_INIT}")
message(STATUS "Link Flags  : ${CMAKE_SHARED_LINKER_FLAGS_INIT}")
endif()

## uncomment these when debugging toolchain setup
# set(CMAKE_FIND_DEBUG_MODE ON)
# set(CMAKE_VERBOSE_MAKEFILE ON)
# set(CMAKE_SHARED_LINKER_FLAGS_INIT "${CMAKE_SHARED_LINKER_FLAGS_INIT} -Wl,--verbose")
# set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT} -Wl,--verbose")
# set(CMAKE_C_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} -H")
# set(CMAKE_CXX_FLAGS_INIT "${CMAKE_CXX_FLAGS_INIT} -H")
# set(CMAKE_FIND_DEBUG_MODE TRUE)
