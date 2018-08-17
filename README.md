# Roboy DartTracker

This repository contains the DartTracker, a project making use of [DART](https://www.cc.gatech.edu/~afb/classes/CS7495-Fall2014/readings/dart.pdf) and [its implementation](https://github.com/ori-drs/dart). It extends it by using the input of a RealSense Camera, feeding the depth values into the dart core and using the results to update the positions of the tracked models. Arbitary models (in .obj format and with respective .sdf specification) can be specified and tracked.

## Installation 

1. Initially, setup your system to run the Xenial kernel as described [here](http://wiki.ros.org/librealsense#Installation_Prerequisites).
2. Install [CUDA 8.0](https://developer.nvidia.com/cuda-80-ga2-download-archive). __IMPORTANT__: Use the .run files and follow their instructions. **Do not** install the driver, solely the toolkit. Examples are needed since the dartExample makes use of some files there. 
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

Build the repository in order to have the files needed in the following steps (adapt paths). The build directive enables easier debugging when working with the built
 
	cd /path/to/DartTracker/
	catkin_make -DCMAKE_BUILD_TYPE=Debug 

Source the environment (adapt the path) and add the uvcvideo to the kernel: 

	source /path/to/DartTracker/devel/setup.sh
	sudo modprobe uvcvideo

In case no roscore is running yet, start one:

	roscore

## Common issues: 

In case some packages could not be found, make sure the following path is included in your environment variables, otherwise add it. 

	export PYTHONPATH="$PYTHONPATH:/usr/lib/python2.7/dist-packages"

__Anaconda:__ In case you have anaconda installed, this may cause some libraries to not be found, since they might first be referred to the Anaconda path and libraries in these folders. If you can deinstall anaconda (remove the anaconda folder) or (temporarily) remove it from you environment variables, do so. AFTER that, rebuild everything (including Pangolin!) and it should be good to go. 

__Models:__
All models appear mirrored in the application for some unknown reason, this is why all models are initially mirrored to later appear correctly again.

__Scale:__
Some models might not appear since the scale is too small or big. Tip: When working with a model, simply import the Ikea mug into the scene as well as a reference for the appropriate size and scale or apply a scale later on in the .xml file or when exporting the model. The application scale of the DartTracker application is in meters.

__Coordinate System:__
As far as known: x goes to the right on the horizontal axis, y goes up on the vertical axis and z points away from the camera into the scene. 

## Run the Application (for RealSense)

Plug in the camera and execute the following command.

	rosrun dart_tracker extended_demo

Alternatively, from the DartTracker/ folder, run the following command to work with the gdb debugger. 

	gdb --args devel/lib/dart_tracker/extended_demo

## Current Status

- The extended_demo uses the RealSense camera is used as a depth source.
- Tracking works for objects for which the initial position was set: 
		- One can set the positions by moving the sliders on the right side of the application window
		- As soon as position **and** rotation roughly match, untick the `sliderControl` checkbox. 
		- For further changes, check it again, move, and uncheck when finished. During that time, no tracking calculations take place which would otherwise overwrite the new position. 
- When multiple objects are tracked, it is be preferable for each to be defined with a model of its own, otherwise tracking might suffer.
- The pose of the xylophone is published on a ROS node using a geometry::Pose message.
- **ZEDm**: The initial implementation of the ZEDm Camera could not be tested, as the camera itself cannot be detected on our system so far. It appears to be a compatibility issue of kernel, Cuda, ZED SDK and ZED camera firmware version. A rosnode for the ZED can also be started if the camera can be detected. 

## Modifications

In order to work on the project, change the file `DartTracker/src/dart_tracker/src/extended_demo.cpp`, or the ZEDm_demo, where camera input and tracked models are handled. 
