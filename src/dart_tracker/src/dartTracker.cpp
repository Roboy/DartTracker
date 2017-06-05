#include "dart_tracker/dartTracker.hpp"

DartTracker::DartTracker(){
    cudaSetDevice(0);
    cudaDeviceReset();
    tracker = new dart::Tracker;

    poseReduction = dart::loadParamMapPoseReduction("/home/roboy/workspace/DartTracker/src/dart_tracker/models/spaceJustin/justinHandParamMap.txt");

    const static int obsSdfSize = 64;
    const static float obsSdfResolution = 0.01*32/obsSdfSize;
    const static float defaultModelSdfResolution = 2e-3; //1.5e-3;
    const static float3 obsSdfOffset = make_float3(0,0,0.1);

    tracker->addModel("/home/roboy/workspace/DartTracker/src/dart_tracker/models/spaceJustin/spaceJustinHandRight.xml",
                      defaultModelSdfResolution,
                      defaultModelSdfPadding,
                     obsSdfSize,
                     obsSdfResolution,
                     make_float3(-0.5*obsSdfSize*obsSdfResolution) + obsSdfOffset, poseReduction);

    tracker->addDepthSource(&realsense);
}

DartTracker::~DartTracker(){

}

PLUGINLIB_EXPORT_CLASS(DartTracker, rviz::Panel)