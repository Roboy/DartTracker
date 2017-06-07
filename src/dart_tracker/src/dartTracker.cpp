#include "dart_tracker/dartTracker.hpp"
#include <sensor_msgs/image_encodings.h>


DartTracker::DartTracker(){
    if (!ros::isInitialized()) {
        int argc = 0;
        char *argv = nullptr;
        ros::init(argc, &argv, "roboy_managing_node",
                  ros::init_options::AnonymousName |
                  ros::init_options::NoRosout);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    realsense_pub = nh->advertise<sensor_msgs::Image>("/DartTracker/realsense_depth", 1);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();





    cudaSetDevice(0);
    cudaDeviceReset();
    tracker = new dart::Tracker;

    poseReduction = dart::loadParamMapPoseReduction("/home/roboy/workspace/DartTracker/src/dart_tracker/models/spaceJustin/justinHandParamMap.txt");

    const static int obsSdfSize = 64;
    const static float obsSdfResolution = 0.01*32/obsSdfSize;
    const static float defaultModelSdfResolution = 2e-3; //1.5e-3;
    const static float3 obsSdfOffset = make_float3(0,0,0.1);

    /*tracker->addModel("/home/roboy/workspace/DartTracker/src/dart_tracker/models/spaceJustin/spaceJustinHandRight.xml",
                      defaultModelSdfResolution,
                      defaultModelSdfPadding,
                     obsSdfSize,
                     obsSdfResolution,
                     make_float3(-0.5*obsSdfSize*obsSdfResolution) + obsSdfOffset, poseReduction);

    tracker->addDepthSource(&realsense);
     */

    realsensePubRunner = true;
    realsense_thread = new std::thread(&DartTracker::realsensePub, this);
    realsense_thread->detach();
}

DartTracker::~DartTracker(){
    realsensePubRunner = false;
    realsense_thread->join();
    delete tracker;
}



void DartTracker::realsensePub() {
    while(realsensePubRunner){
        realsense.advance();

        sensor_msgs::Image msg;
        msg.width = realsense.getDepthWidth();
        msg.height = realsense.getDepthHeight();
        msg.encoding = sensor_msgs::image_encodings::MONO16;
        uint8_t * depth_data = (uint8_t*)(realsense.getDepth());

        printf("Depth Image: %i\n", depth_data);
        for(int i = 0; i < 100; i++){
            printf("%i\n", depth_data[i]);
        }

        msg.data.assign(depth_data, depth_data+ 10);//(msg.width*msg.height*2));
        realsense_pub.publish(msg);


        boost::this_thread::sleep( boost::posix_time::milliseconds(20) );
    }
}



PLUGINLIB_EXPORT_CLASS(DartTracker, rviz::Panel)