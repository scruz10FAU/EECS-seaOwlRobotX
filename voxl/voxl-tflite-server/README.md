# voxl-tflite-server

Use tensorflow lite on VOXL for various deep learning tasks


## Build dependencies

- libmodal-json
- libmodal-pipe
- qrb5165-tflite

## Build Environment

This project builds in the voxl-cross >=4.3 docker image

Follow the instructions here to build and install the voxl-cross docker image:
https://gitlab.com/voxl-public/voxl-docker


## Build Instructions

1) Launch the voxl-cross docker. At the time of writing, voxl-cross V4.3 is the latest

```bash
~/git/voxl-tflite-server$ voxl-docker -i voxl-cross
voxl-cross(4.3):~$
```

2) Install dependencies inside the docker. You must specify both the hardware platform and binary repo section to pull from. CI will use the `dev` binary repo for `dev` branch jobs, otherwise it will select the correct target SDK-release based on tags. When building yourself, you must decide what your intended target is, usually `dev`

You must also specify if you are building for a 1.X VOXL 2 system image (Ubtuntu 18.04) or a 2.X VOXL2 system image (Ubuntu 20.04). These are specified as "qrb5165" and "qrb5165-2" platforms respectively.



```bash
voxl-cross(4.3):~$ ./install_build_deps.sh qrb5165 dev
OR
voxl-cross(4.3):~$ ./install_build_deps.sh qrb5165-2 dev
```


3) Build scripts should take the hardware platform as an argument: `qrb5165` or `qrb5165-2`. CI will pass these arguments to the build script based on the job target. 


```bash
voxl-cross(4.3):~$ ./build.sh qrb5165
OR
voxl-cross(4.3):~$ ./build.sh qrb5165-2
```


4) Make a deb package while still inside the docker.

```bash
voxl-cross(4.3):~$ ./make_package.sh
```

This will make a new .deb package file in your working directory. The name and version number came from the package control file. If you are updating the package version, edit it there.

Optionally add the --timestamp argument to append the current data and time to the package version number in the debian package. This is done automatically by the CI package builder for development and nightly builds, however you can use it yourself if you like.


## Run Linter Check (optional)

```bash
voxl-cross(4.3):~$ voxl-cross-linter
```


## Deploy to VOXL

You can now push the deb package to VOXL and install with dpkg however you like. To do this over ADB, you may use the included helper script: deploy_to_voxl.sh. Do this outside of docker as your docker image probably doesn't have usb permissions for ADB.

```bash
(outside of docker)
voxl-tflite-server$ ./deploy_to_voxl.sh
```

This deploy script can also push over a network given sshpass is installed and the VOXL uses the default root password.


```bash
(outside of docker)
voxl-tflite-server$ ./deploy_to_voxl.sh ssh 192.168.1.xyz
```



# Implementing your Model in voxl-tflite-server

Integrating your models into the voxl-tflite-server is quite easy using the generic base classes provided as a template to allow your model to work seamlessly with the voxl hardware. 

## Set up

To get started, you must first move your __.tflite__ file to `/usr/bin/dnn/` on your voxl. You would also neded to do the same for your labels file.

In the function `initialize_model_settings` in the `main` file, add a conditional corresponding to your particular model in the same format as the others. Models in the voxl-tflite-server are indexed using two attributes, the model name and the model category. For example, the objection detection yolov8 model has the model name `YOLOV8` and the model category `OBJECT_DETECTION`.

Then in the `model_helper.cpp`, in the function `create_model_helper`, add a switch statement corresponding to your attributes and return an instantiation of your model class (more details on that in the next section)

## Model class

You need to create a header and source file for your model and write the corresponding Helper class. Your class must inherit the common base class provided in __model_helper.h__ and __model_helper.cpp__. 

Your class will have the following methods

* __preprocess__ : This method contains the preprocesing logic for your model before it is passed for inference. There is a default implementation provided in the model_helper.cpp. It can be overriden if needed. 

* __run_inference__: This method invokes the tflite interpreter using your `.tflite` file and passes in the preprocessed input. It can be overriden if your model requires different logic.

* __post_process__: This method needs to be implemented by you according to your model's requirements. This method is invoked by the __worker__ method.

* __worker__: This method performs operations external to the model which are required by other voxl functions such as passing postprocessed images to their relevant pipes and timestamping frames. This method invokes the postprocess method and must also be implemented based on model functionality. You may have a look at the other worker methods to get an idea on how to write your own.

By following the above steps, you will be able to add any custom model to your voxl with complete freedom to create your own pre-processing, inferencing, and post-processing logic. 








