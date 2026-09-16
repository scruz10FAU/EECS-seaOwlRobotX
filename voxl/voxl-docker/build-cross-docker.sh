#!/bin/bash

set -e

VERSION="4.8"
RUN_SCRIPT=voxl-docker


CLEAN=""
if [ "$1" == "clean" ] || [ "$2" == "clean" ]; then
	CLEAN="--no-cache"
	echo "starting clean build"
fi

VERBOSE=""
if [ "$1" == "verbose" ] || [ "$2" == "verbose" ]; then
	VERBOSE="--progress=plain"
	echo "starting verbose build"
fi

QRB5165_1_ROOTFS="1.8.06-M0054-14.1a-perf-20260203-rootfs.tar.gz"
QRB5165_2_ROOTFS="2.0.1-perf-20260122-rootfs.tar.gz"
QCS6490_ROOTFS="qcs6490-ubun24-rootfs-0.0.6-20260311-040328.tar.gz"

# Check required files exist
REQUIRED_FILES=(
	"${QRB5165_1_ROOTFS}"
	"${QRB5165_2_ROOTFS}"
	"${QCS6490_ROOTFS}"
)
ERROR=false

for FILE in ${REQUIRED_FILES[@]} ; do
	if [ ! -f "voxl-cross/$FILE" ] ; then
		echo "Missing: $FILE"
		ERROR=true
	fi
done

if $ERROR ; then
	echo
	echo "Please following the instruction in the README to download"
	echo "these files from downloads.modalai.com and place in voxl-cross/"
	exit 1
fi


# Build Docker image
cd voxl-cross
docker build $CLEAN $VERBOSE\
			-t voxl-cross:V${VERSION} \
			--build-arg VERSION=${VERSION} \
			--build-arg QRB5165_1_ROOTFS="${QRB5165_1_ROOTFS}" \
			--build-arg QRB5165_2_ROOTFS="${QRB5165_2_ROOTFS}" \
			--build-arg QCS6490_ROOTFS="${QCS6490_ROOTFS}" \
			-f voxl-cross.Dockerfile .
docker tag voxl-cross:V${VERSION} voxl-cross:latest
cd ../

# install the voxl-docker helper script
echo "installing ${RUN_SCRIPT}.sh to /usr/local/bin/${RUN_SCRIPT}"
sudo install -m 0755 files/${RUN_SCRIPT}.sh /usr/local/bin/${RUN_SCRIPT}

echo "DONE"
