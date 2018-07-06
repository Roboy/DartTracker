# Roboy DartTracker

This repository contains the DartTracker, a project making use of [DART](https://www.cc.gatech.edu/~afb/classes/CS7495-Fall2014/readings/dart.pdf) and [its implementation](https://github.com/ori-drs/dart). It extends it my using the input of a RealSense Camera, feeding the depth values into the dart core and using the results to update the positions of the tracked models. Arbitary models (in .obj format and with respective .sdf specification) can be specified and tracked.

## Installation 

Please follow the implementations of the [original dart repository](https://github.com/ori-drs/dart).

## Environment Setup

Build the repository in order to have the files needed in the following steps (adapt paths).
 
	cd /path/to/DartTracker/
	catkin_make

Source the environment (adapt the path) and add the uvcvideo to the kernel: 

	source /path/to/DartTracker/devel/setup.sh
	sudo modprobe uvcvideo

In case no roscore is running yet, start one:

	roscore

## Run the ROS package

Plug in the camera and execute the following command.

rosrun dart\_tracker roboy\_dart\_tracker

## Current Status

As of now, input of the realsense camera is not properly handled so that no models are detected yet. 

## Modifications

In order to work on the project, change the file `DartTracker/src/dart_tracker/src/main.cpp`, where camera input and tracked models are handled. 
