## start with 24.04 to match QCS6490
FROM ubuntu:24.04


## version number passed in by build-cross-docker.sh
ARG VERSION
RUN test -n "$VERSION" || (echo "ERROR: You must pass --build-arg VERSION=<value>" && exit 1)
RUN mkdir -p /etc/modalai/
RUN echo -n "voxl-cross(${VERSION})" > /etc/modalai/image.name


# new primary sources list pointing to archive, plus arm64 sources and bionic
# sources to help us get the old gcc7
COPY apt_sources /etc/apt/

# update base packages in noninteractive mode
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update
RUN apt-get -y install apt-utils sudo
RUN apt-get -y upgrade

# basic dev tools
RUN apt-get -y install build-essential make sudo curl unzip gcc git nano vim xterm file

# qrb5165 ubun1 18.04 chipcode is a bit sparse on 32-bit support, it just has a
# few core libs. We pulled out many many standard lib packages from the bionic packages
# but install these just in case. TODO see if these are still needed.
RUN apt-get -y install libc6-armel-cross libc6-dev-armel-cross binutils-arm-linux-gnueabi

# other cross compilers commented out for reference
# RUN apt-get -y install gcc-4.9-arm-linux-gnueabi g++-4.9-arm-linux-gnueabi
# RUN apt-get -y install gcc-4.9-aarch64-linux-gnu g++-4.9-aarch64-linux-gnu
# RUN apt-get -y install gcc-7-aarch64-linux-gnu g++-7-aarch64-linux-gnu
# RUN apt-get -y install gcc-8-aarch64-linux-gnu g++-8-aarch64-linux-gnu



# the qrb5165 ubun1 18.04 chipcode ships with gcc7 so we use it here. voxl-cross
# has to add bionic sources to get it. I would like to use gcc 10 from focal but
# gcc10 does not play nice with the arm-neon headers in 18.04 and potentially
# other things so to be safe we match the compiler that ships with the chipcode.
# We really just use the 32-bit compiler but pull in the 64-bit one too for
# debugging and conveninece
RUN apt-get -y install gcc-7-aarch64-linux-gnu g++-7-aarch64-linux-gnu
RUN apt-get -y install gcc-7-arm-linux-gnueabi g++-7-arm-linux-gnueabi

# gcc 10 is the latest on 20.04 focal and gcc 10 libs seem to be installed on
# ubun2 but gcc 9 is installed by default so we use that. Include both anyway
RUN apt-get -y install gcc-9-aarch64-linux-gnu g++-9-aarch64-linux-gnu
RUN apt-get -y install gcc-10-aarch64-linux-gnu g++-10-aarch64-linux-gnu

# gcc 13 is the latest on 24.04 noble so use it for qcs6490
RUN apt-get -y install gcc-13-aarch64-linux-gnu g++-13-aarch64-linux-gnu

# Need to install Python ctypesgen module for autogenerating Python MPA interface definitions
RUN apt-get -y install net-tools sshpass iputils-ping
RUN apt-get -y install python3-pip
RUN pip3 install ctypesgen --break-system-packages


# Selected CMake version, you can run `apt list -a cmake` in the docker image to see available versions
# we want cmake 3.31 since cmake 4.0.0 that's in ubuntu 20.04 won't build one of our ancient 3rd party packages
RUN apt-get update && apt-get install -y ca-certificates gpg wget
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | \
	gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ noble main' | \
	tee /etc/apt/sources.list.d/kitware.list >/dev/null

ARG CMAKE_VERSION=3.31.0-0kitware1ubuntu24.04.1
RUN apt-get update && apt-get install -y cmake=${CMAKE_VERSION} cmake-data=${CMAKE_VERSION}

# Setup to allow multiarch for aarch64 apt package installations
RUN dpkg --add-architecture arm64

# update with the new architectures
RUN apt-get -y update

# install libexif for our camera stuff TODO this should be in the sysroots
# but qrb5165_1 is missing the headers
RUN apt-get install -y libexif-dev:arm64

# eigen is header only, safe to install, used by ceres and other math libs
RUN apt-get install -y libeigen3-dev

# protobuf added for tensorflow so we don't need to install at build time
RUN apt-get install -y protobuf-compiler

# Add adb / fastboot
RUN apt-get install -y android-tools-adb android-tools-fastboot

# add Anthropic Claude Code client
RUN curl -fsSL https://claude.ai/install.sh | bash

# Create a wrapper script that redirects Claude's config to /opt/claude-home
# This prevents Claude from creating .claude.json, .claude/, etc in the working directory
RUN mkdir -p /opt/claude-home && \
    mv /root/.local/bin/claude /root/.local/bin/claude-real && \
    echo '#!/bin/bash' > /root/.local/bin/claude && \
    echo 'HOME=/opt/claude-home exec /root/.local/bin/claude-real "$@"' >> /root/.local/bin/claude && \
    chmod +x /root/.local/bin/claude

ENV PATH="/root/.local/bin:$PATH"


# clean up the package cache to save space
RUN apt-get -y clean


################################################################################
# KEEP AS MUCH ABOVE THE SYSROOT INSTALL AS POSSIBLE ^^^
# we update the sysroots a lot and want as little of the dockerfile to need
# rebuilding as possible when we do so.
################################################################################


# Create sysroot directories and extract rootfs contents
# do the test and add steps sequentially, otherwise updating the qcs6490 rootfs
# which happens often would need to rebuild the image by going all the way back
# before the 3 sysroot add steps which is slow
ENV SYSROOT_BASE=/opt/sysroots

ARG QRB5165_1_ROOTFS
RUN test -n "$QRB5165_1_ROOTFS" || (echo "ERROR: You must pass --build-arg QRB5165_1_ROOTFS=<value>" && exit 1)
ADD ${QRB5165_1_ROOTFS} ${SYSROOT_BASE}/qrb5165_1/

ARG QRB5165_2_ROOTFS
RUN test -n "$QRB5165_2_ROOTFS" || (echo "ERROR: You must pass --build-arg QRB5165_2_ROOTFS=<value>" && exit 1)
ADD ${QRB5165_2_ROOTFS} ${SYSROOT_BASE}/qrb5165_2/

ARG QCS6490_ROOTFS
RUN test -n "$QCS6490_ROOTFS" || (echo "ERROR: You must pass --build-arg QCS6490_ROOTFS=<value>" && exit 1)
ADD ${QCS6490_ROOTFS} ${SYSROOT_BASE}/qcs6490/


# fix all the symlinks in the sysroots that would have broken when copying over
COPY fix_sysroot_symlinks.sh /opt/
RUN bash /opt/fix_sysroot_symlinks.sh
RUN rm -rf /opt/fix_sysroot_symlinks.sh

# install 32-bit cross compile tools in the qrb5165_1 sysroot which aren't needed
# at runtime and are therefore not part of the system image. These could probably
# be pulled via 'apt download' while building the docker but they are small and
# easy to keep here. Need to be installed via dpkg-deb -x to put them in the
# sysroot instead of the host rootfs so we can keep all headers in the sysroot.
WORKDIR /tmp/bionic-pkgs
ADD qrb5165_arm32_glibc_2.27/ /tmp/bionic-pkgs
RUN for deb in /tmp/bionic-pkgs/*.deb; do \
	dpkg-deb -x "$deb" ${SYSROOT_BASE}/qrb5165_1/; \
done
RUN rm -rf /tmp/bionic-pkgs

# install voxl-opencv package to the qrb5165 sysroot to match the preinstalled
# official opencv distro that is already in the qrb5165-2 sysroot
WORKDIR /tmp/opencv
RUN wget http://voxl-packages.modalai.com/dists/qrb5165/sdk-1.5/binary-arm64/voxl-opencv_4.5.5-3_arm64.deb
RUN dpkg-deb -x voxl-opencv_4.5.5-3_arm64.deb ${SYSROOT_BASE}/qrb5165_1/
RUN rm -rf /tmp/opencv

# remove the qualcomm tensorflow lite package from the qrb5165_1 sysroot so it
# doesn't get in the way. We replace it with our own during SDK install so it
# will be gone at runtime too.
RUN dpkg --root=/opt/sysroots/qrb5165_1/ -r tensorflow-lite

## Now brute force the rest of the system libs that are missing a *.so symlink
## to the installed *.so.x and *.so.x.y.z libs. Target sysroots don't contain
## the dev version of mode libraries.
COPY generate_so_symlinks.sh /opt/
RUN bash /opt/generate_so_symlinks.sh
RUN rm -rf /opt/generate_so_symlinks.sh

## apply some patches, currently just fixing a bad cmake file from google benchmark
## that has a hard-coded path which is a known issue
COPY patch_sysroots.sh /opt/
RUN bash /opt/patch_sysroots.sh
RUN rm -rf /opt/patch_sysroots.sh

# add some open source android headers we need to build
ADD qrb5165_1_includes/ /opt/sysroots/qrb5165_1/usr/include/
ADD qrb5165_2_includes/ /opt/sysroots/qrb5165_2/usr/include/

COPY voxl-cross-linter.sh /usr/bin/voxl-cross-linter
RUN chmod +x /usr/bin/voxl-cross-linter

# add our toolchain files
COPY cross_toolchain/ /opt/cross_toolchain/

# add fun bash profile and utilities
COPY bash_utilities/ /



