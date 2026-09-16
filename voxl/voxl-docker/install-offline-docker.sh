#!/bin/bash

set -e

VERSION="V1.0"


CLEAN=""
if [ "$1" == "clean" ]; then
	CLEAN="--no-cache"
	echo "starting clean build"
fi

# Add bash utilities
#cd bash_utilities
#./make_package.sh
#cd ..
#cp ./bash_utilities/bash_utilities.tar voxl-cross/

# Build Docker image
cd offline-voxl-sdk
docker build $CLEAN -t offline-voxl-sdk:${VERSION} -f offline-voxl-sdk.Dockerfile .
docker tag offline-voxl-sdk:${VERSION} offline-voxl-sdk:latest
cd ../

# Clean the bash utilities file
#rm voxl-cross/bash_utilities.tar

# install the voxl-docker helper script
#echo "installing ${RUN_SCRIPT}.sh to /usr/local/bin/${RUN_SCRIPT}"
#sudo install -m 0755 files/${RUN_SCRIPT}.sh /usr/local/bin/${RUN_SCRIPT}

echo "DONE"
