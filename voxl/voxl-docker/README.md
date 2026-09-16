# voxl-docker

This project provides the tools needed to build the three docker images used for compiling VOXL packages.

You do not need to build these yourself, you can download the pre-built images from https://downloads.modalai.com

For overview of build environments and how to use them, go to https://docs.modalai.com/build-environments/



## voxl-docker bash script

This repo contains the "voxl-docker" bash script for easily launching docker images with the right permissions and with the local directory mounted in the docker for easy compilation. Most VOXL projects use this script in their build instructions.

Install this by running:

```
~/git/voxl-docker$ ./install-voxl-docker-script.sh
```


## voxl-cross

voxl-cross is used to build the vast majority of voxl-sdk packages.

It contains the full sysroot for VOXL 2 18.04 and 20.04 system images as well as QCS6490. While it can and does compile 32-bit libs for some packages, there is no more support for VOXL 1 APQ8096 platforms since voxl-cross 2.7.

YOU DON'T NEED TO BUILD THIS YOURSELF, you can just download it from https://downloads.modalai.com

Packages building in voxl-cross should use the following template:
https://gitlab.com/voxl-public/voxl-sdk/voxl-cross-template

Note this template does update over time as new macros and build targets are added.

### voxl-cross changelog

#### 4.8
    - Added Claude Code to the voxl-cross docker
    - Added adb to the voxl-cross docker
    - Updated to latest sysroots for qrb5165 (ubun 1.0 and ubun 2.0) and qcs6490
    - new boost and opencv packages in qcs6490 sysroot
    - gstreamer libs in qcs6490
    - fastcv.h added to qrb5165
#### 4.7
	- new qcs6490 sysroot 20260108-050337 with libjpegturbo
#### 4.6
	- new qcs6490 sysroot 20251210-050323
#### 4.5
	- remove old libfc_sensor from qrb5165-1 sysroot
	- add /usr/lib/qcm6490/ to qcs6490 toolchain link dirs
	- new sysroots for all 3 targets from nov 5, 2025 nightly

#### 4.4
	- updated qcs6490 sysroot
	- toolchains now use Scrt1.o crti.o and crtn.o from sysroots
	- add gstreamer headers to qrb5165 and qrb5165-2 sysroots

#### 4.3
	- Add qcs6490 sysroot
	- remove qualcomm tensorflow-lite package from qrb5165_1 sysroot
	- migrate to ubuntu 24.04 base image
	- add strict include directory order to toolchains to prevent conflicts with host OS includes
	- Make sure cmake doesn't find programs in the sysroots
	- add .so symlinks to libraries in the sysroots as a workaround for missing libXYS-dev packages
	- force CMAKE_CROSSCOMPILING ON in toolchains to prevent check_function_exists() from trying to run test executables
	- add libeigen3-dev headers to host filesystem since they are commonly used
	- new qrb5165-2 sysroot with suitesparse and other ceres dependencies

#### 4.1
	- add codec2 support for qrb5165_2
	- put qrb5165_1 toolchain back to gcc7 to prevent occational glibc conflicts
	- fix the symlink recovery process in sysroots
	- updated rootfs for qrb5165_2

#### 4.0
	- complete rearchitecture from cross 2.X and 3.X, 
	- now contains full sysroots for qrb5165 ubun1 and qrb5165 ubun2
	- based on ubuntu 20.04


### voxl-cross build instructions

If you really really really want to build this yourself:

1) download the following sysroots from https://downloads.modalai.com and place the voxl-cross directory.

	- ubun1.0_1.8.06-M0054-14.1a-perf-20260122-rootfs.tar
	- ubun2.0_2.0.1-perf-20260122-rootfs.tar
	- qcs6490_ubun24_rootfs-nightlies_qcs6490-ubun24-rootfs-0.0.2-20260121-050324.tar

2) run the install-cross-docker.sh script

```
~/git/voxl-docker$ ./install-cross-docker.sh
```

To compress a docker image for export after it's built:

```
docker save voxl-cross:V4.8 | gzip > voxl-cross_V4.8.tgz
```

## voxl-emulator

voxl-emulator simulates the VOXL system image and is built alongside the base system image. voxl-emulator can only build 32-bit ARM binaries using the native ARM gcc. It is an ARM docker image that runs on x86 using qemu as an emulator, it is therefore slower than voxl-cross which is native x86, but has the advantage of emulating the VOXL sysroot.

This must be downloaded from https://downloads.modalai.com and cannot be built by itself without rebuiling the entire system image from source.

## qrb5165-emulator

qrb5165-emulator simulates the qrb5165 system image and is built alongside the base system image. qrb5165-emulator builds 64-bit ARM binaries using the native ARM gcc. It is an ARM docker image that runs on x86 using qemu as an emulator, it is therefore slower than voxl-cross which is native x86, but has the advantage of emulating the qrb5165 platform sysroot.

More details can be found [here](qrb5165-emulator/README.md)

## How to build voxl-hexagon

Like voxl-cross, voxl-hexagon is an ubuntu based docker image, but contains the hexagon SDSP sdk for building SDSP projects like libvoxl_io. It also contains the opkg package manager for installing build dependencies.

Note you don't need to build this yourself, you can just download it from https://downloads.modalai.com

To build this yourself:

1) Download and place the following files into cross_toolchain/downloads

Hexagon SDK 3.1 install file: qualcomm_hexagon_sdk_3_1_eval.bin

* [downloads.modalai.com](downloads.modalai.com)

Linaro ARM compiler binaries: gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabi.tar.xz

* [https://releases.linaro.org/archive/14.11/components/toolchain/binaries/arm-linux-gnueabi](https://releases.linaro.org/archive/14.11/components/toolchain/binaries/arm-linux-gnueabi)

2) Run the install-hexagon-docker.sh script. This will also install voxl-docker.sh to usr/bin/ as did install-emulator-docker.sh in case the user wants the hexagon docker only.

```bash
./install-hexagon-docker.sh
```

## offline-voxl-sdk

A Docker for offline x64 devevelopment. A basic overview of offline-voxl-sdk is that the core libraries, tools and voxl-portal are pre-built and installed when the Docker is built. You can then use voxl-replay to playback logs and develop on a PC using this Docker which may speed up development. Not every package can be supported this way, voxl-qvio-server cannot be supported for instance.

```
$ ./install-offline-docker.sh
$ docker images
$ docker run --rm -it -p 80:80 -w /home/$(whoami) --volume="/dev/bus/usb:/dev/bus/usb" -e LOCAL_USER_ID=0 -e LOCAL_USER_NAME=root -e LOCAL_GID=0 -v ${PWD}:/home/root:rw -w /home/root offline-voxl-sdk:latest /bin/bash -l
```

To connect multiple terminals for multi-process debugging

```
$ docker ps
CONTAINER ID   IMAGE                     COMMAND          CREATED         STATUS         PORTS                NAMES
2422174faafe   offline-voxl-sdk:latest   "/bin/bash -l"   4 minutes ago   Up 4 minutes   0.0.0.0:80->80/tcp   blissful_thompson
$ docker exec -it 2422174faafe bash
```

```
$ cat /proc/sys/fs/pipe-max-size
```
