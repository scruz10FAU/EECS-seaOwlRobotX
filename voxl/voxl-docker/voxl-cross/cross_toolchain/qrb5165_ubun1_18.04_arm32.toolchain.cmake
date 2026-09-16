## This is the recommended toolchain for building 32-bit libs for qrb5165 to
## support the old mv lib from qualcomm. This is only needed for voxl-qvio-server
## and supporting libraries (libmodal_json, librc_math, libmodal_pipe)
##
## This is really quite awkward as the  mv lib from qualcomm has not been updated
## or maintained in a long time and was last built for the APQ8096 platform which
## used glibc 2.23, gcc 4.9, and 32-bit arm architecture.
##
## however, we can still build against this old glibc and run the binaries on
## qrb5165

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(TARGET_SYSROOT /opt/sysroots/qrb5165_1)

# the qrb5165 ubun1 18.04 chipcode ships with gcc7 so we use it here. voxl-cross
# has to add bionic sources to get it. I would like to use gcc 10 from focal but
# gcc10 does not play nice with the weird mix of 32-bit stuff we have in this
# system image.
set(tools /usr)
set(CMAKE_C_COMPILER ${tools}/bin/arm-linux-gnueabi-gcc-7 CACHE STRING "" FORCE)
set(CMAKE_CXX_COMPILER ${tools}/bin/arm-linux-gnueabi-g++-7 CACHE STRING "" FORCE)

# use sysroot as primary search path, then fall back to using host rootfs
## build with qrb5165 ubun1 sysroot
set(CMAKE_SYSROOT_COMPILE ${TARGET_SYSROOT})
set(CMAKE_SYSROOT_LINK    ${TARGET_SYSROOT})

# look for things in sysroot first, then host sysroot, except progrmas like make/ar
set(CMAKE_FIND_ROOT_PATH ${TARGET_SYSROOT} / )
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)


# Qualcomm qrb5165 Kona cpu is based on ARM A77 and A55 cores (armv8.2-a)
# add the host /usr/include AFTER these just to find things installed with
# install_build_deps.sh
# double check the directory order with this command inside voxl-cross
# echo "" | aarch64-linux-gnu-gcc-10 --sysroot /opt/sysroots/qrb5165_1 -xc -E -v -
# disable stack protector and set the softfp ABI to match qualcomm's libmv and
# the weird 32-bit libs we have in this system image.
set(CMAKE_C_FLAGS_INIT "\
-isystem=/usr/arm-linux-gnueabi/include \
-isystem=/usr/include \
-idirafter /usr/include \
-fno-stack-protector \
-march=armv7-a \
-mfloat-abi=softfp \
-mfpu=vfpv4")

set(CMAKE_CXX_FLAGS_INIT "\
-isystem=/usr/arm-linux-gnueabi/include \
-isystem=/usr/include \
-idirafter /usr/include \
-fno-stack-protector \
-march=armv7-a \
-mfloat-abi=softfp \
-mfpu=vfpv4")


# force gcc to use the sysroot lib directories and libgcc bins, it really
# doesn't want to use these so we manually set the files. I tried cleaning this
# list up but since the 32-bit environment on qrb5165 is wonky we need to take
# full control to ensure the right binaries are pulled.
set(CMAKE_EXE_LINKER_FLAGS_INIT "\
-L${TARGET_SYSROOT}/usr/lib32 \
-L${TARGET_SYSROOT}/lib \
-L${TARGET_SYSROOT}/usr/lib \
-L${TARGET_SYSROOT}/usr/arm-linux-gnueabi \
-L${TARGET_SYSROOT}/usr/lib/gcc-cross/arm-linux-gnueabi/7 \
${TARGET_SYSROOT}/usr/lib/gcc-cross/arm-linux-gnueabi/7/libgcc.a \
${TARGET_SYSROOT}/usr/lib/gcc-cross/arm-linux-gnueabi/7/libgcc_eh.a \
${TARGET_SYSROOT}/usr/lib/gcc-cross/arm-linux-gnueabi/7/libssp_nonshared.a \
${TARGET_SYSROOT}/usr/arm-linux-gnueabi/lib/libc_nonshared.a \
-L/usr/lib")

set(CMAKE_SHARED_LINKER_FLAGS_INIT "\
-L${TARGET_SYSROOT}/usr/lib32 \
-L${TARGET_SYSROOT}/lib \
-L${TARGET_SYSROOT}/usr/lib \
-L${TARGET_SYSROOT}/usr/arm-linux-gnueabi \
-L${TARGET_SYSROOT}/usr/lib/gcc-cross/arm-linux-gnueabi/7 \
-L/usr/lib")


# for VOXL, install 64-bit libraries to lib64, 32-bit libs go in /usr/lib
set(LIB_INSTALL_DIR /usr/lib)
set(PLATFORM "QRB5165")
add_definitions(-DPLATFORM_QRB5165)

## print some stats once
if(NOT DEFINED _MY_TOOLCHAIN_INITIALIZED)
set(_MY_TOOLCHAIN_INITIALIZED TRUE)
message(STATUS "---------------------------------------------------------")
message(STATUS "Using voxl-cross 32-bit toolchain for QRB5165")
message(STATUS "C Compiler     : ${CMAKE_C_COMPILER}")
message(STATUS "C++ Compiler   : ${CMAKE_CXX_COMPILER}")
message(STATUS "Sysroot        : ${TARGET_SYSROOT}")
message(STATUS "C flags        : ${CMAKE_C_FLAGS_INIT}")
message(STATUS "CXX flags      : ${CMAKE_CXX_FLAGS_INIT}")
message(STATUS "EXE Link Flags : ${CMAKE_SHARED_LINKER_FLAGS_INIT}")
message(STATUS "SO Link Flags  : ${CMAKE_EXE_LINKER_FLAGS_INIT}")
endif()


## uncomment these when debugging toolchain setup
# set(CMAKE_FIND_DEBUG_MODE ON)
# set(CMAKE_VERBOSE_MAKEFILE ON)
# set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS_INIT} -Wl,--verbose")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS_INIT} -Wl,--verbose")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS_INIT} -H")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_INIT} -H")
