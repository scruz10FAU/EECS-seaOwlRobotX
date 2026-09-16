# qrb5165-emulator

## Summary

The `qrb5165-emulator` is an armv8 Ubuntu 18 Docker image that runs on your Linux PC to emulate qrb5165 based platforms such as RB5 Flight.

This project takes the rootfs output of the system image build (download below) and builds it on top of an armv8 Docker 'scratch' image. This provides developers an off-target workflow for developing for the qrb5165.

For example, we use this image in our build servers for CI.

## Requirements

- Ubuntu 18.04 host (other versions may work, but have not been tested)
- Docker
- QEMU, to install:

```bash
# install packages (18.04)
sudo apt-get install qemu binfmt-support qemu-user-static

# install packages (in 24.04)
sudo apt-get install qemu-user binfmt-support qemu-user-static

# Execute the registering scripts, see https://www.stereolabs.com/docs/docker/building-arm-container-on-x86/
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

## Usage

### To Use Prebuilt Image

- Download a prebuilt image by visiting <https://developer.modalai.com/> and locating `Docker Images > qrb5165-emulator-v1.5.tar.gz`
- Unzip using `gzip -d qrb5165-emulator-v1.5.tar.gz`
- Load `docker load < qrb5165-emulator-v1.5.tar` to load the image


### To Build

- Download the `rootfs-qrb5165-emulator-v1.5.tar.gz` image from <https://developer.modalai.com/>
- Copy the file into the `data` directory, for example, assuming the file was downloaded to `Downloads`:


```bash
cd qrb5165-emulator
mkdir data
cp ~/Downloads/rootfs-qrb5165-emulator-v1.5.tar data
```

- Unzip into a new `rootfs` directory:

```
cd qrb5165-emulator/data
mkdir rootfs
tar -xzvf rootfs-qrb5165-emulator-v1.5.tar -C rootfs
```

The version (e.g. `1.5`) should match the `IMAGE_TAG` variable in the `qrb5165-docker-build.sh` script.

Now, build the image by running the following:

```bash
cd ../
./qrb5165-emulator-build.sh
```

*Note:* if you get failures, try running this again:

```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```
