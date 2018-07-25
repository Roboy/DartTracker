# Roboy DartTracker

This repository contains the DartTracker, a project making use of [DART](https://www.cc.gatech.edu/~afb/classes/CS7495-Fall2014/readings/dart.pdf) and [its implementation](https://github.com/ori-drs/dart). It extends it my using the input of a RealSense Camera, feeding the depth values into the dart core and using the results to update the positions of the tracked models. Arbitary models (in .obj format and with respective .sdf specification) can be specified and tracked.

## Installation 

1. Initially, setup your system to run the Xenial kernel as described [here](http://wiki.ros.org/librealsense#Installation_Prerequisites).
2. Install [CUDA 8.0](https://developer.nvidia.com/cuda-80-ga2-download-archive). __IMPORTANT__: Use the .run files and follow their instructions. Do not install the driver, solely the toolkit. Examples are needed since the dartExample makes use of some files there. 
__IMPORTANT__: Delete `/usr/local/cuda-8.0/samples/common/inc/GL/glew.h` and/or `/usr/local/cuda/samples/common/inc/GL/glew.h`, another library provides the same file with more functionalities, this one causes `catkin_make`to fail. Note: after this, the CUDA example will no longer work. Then, update your paths: 

	export PATH="$PATH:/usr/local/cuda-8.0"
	export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/cuda-8.0/lib64"

3. Install ros-kinetik as described on [their website](http://wiki.ros.org/kinetic/Installation/Ubuntu). 
4. Also build roboy-ros-control in order to be able execute `rosrun`. For this, follow the [github repository](https://github.com/Roboy/roboy-ros-control). 

5. Follow the implementations of the [original dart repository](https://github.com/ori-drs/dart). Since *for now* we a still making use of Pangolin, also install this feature. 

6. To build this repository, further packages are needed. (Some of these might already be installed, some might be missing. Try to find the packages themselves or a ros-kinetic-* version in case any build errors occur due to missing libraries)

	sudo apt install libmatheval-dev
	sudo apt install ros-kinetic-pcl-ros
	sudo apt install ros-kinetic-qt-ros

## Environment Setup

Build the repository in order to have the files needed in the following steps (adapt paths).
 
	cd /path/to/DartTracker/
	catkin_make

Source the environment (adapt the path) and add the uvcvideo to the kernel: 

	source /path/to/DartTracker/devel/setup.sh
	sudo modprobe uvcvideo

In case no roscore is running yet, start one:

	roscore

##Common issues: 

In case some packages could not be found, make sure the following path is included in your environment variables, otherwise add it. 

	export PYTHONPATH="$PYTHONPATH:/usr/lib/python2.7/dist-packages"

__Anaconda:__ In case you have anaconda installed, this may cause some libraries to not be found, since they might first be referred to the Anaconda path and libraries in these folders. If you can deinstall anaconda (remove the anaconda folder) or (temporarily) remove it from you environment variables, do so. AFTER that, rebuild everything (including Pangolin!) and it should be good to go. 

## Run the ROS package

Plug in the camera and execute the following command.

rosrun dart\_tracker roboy\_dart\_tracker

## Current Status

As of now, input of the realsense camera is not properly handled so that no models are detected yet. 

## Modifications

In order to work on the project, change the file `DartTracker/src/dart_tracker/src/main.cpp`, where camera input and tracked models are handled. 
