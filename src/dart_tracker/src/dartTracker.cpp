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

    realsense_image_pub = nh->advertise<sensor_msgs::Image>("/DartTracker/realsense_color", 1);
    realsense_depth_pub = nh->advertise<PointCloud>("/DartTracker/realsense_depth", 1);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    cudaSetDevice(0);
    cudaDeviceReset();
    tracker = new dart::Tracker;

    const static int obsSdfSize = 64;
    const static float obsSdfResolution = 0.01*32/obsSdfSize;
    const static float defaultModelSdfResolution = 2e-3; //1.5e-3;
    const static float3 obsSdfOffset = make_float3(0,0,0.1);

    tracker->addModel("/home/roboy/workspace/DartTracker/src/dart_tracker/models/ikeaMug/ikeaMug.xml",
                                       0.5*defaultModelSdfResolution,
                                       0.5*0.5*defaultModelSdfResolution,
                                       64);

    tracker->addDepthSource(&realsense);

    dart::MirroredModel & model = tracker->getModel(0);
    dart::Pose & modelPose = tracker->getPose(0);

    const int depthWidth = realsense.getDepthWidth();
    const int depthHeight = realsense.getDepthHeight();

    const int predWidth = tracker->getPredictionWidth();
    const int predHeight = tracker->getPredictionHeight();

    dart::MirroredVector<uchar3> imgDepthSize(depthWidth*depthHeight);
    dart::MirroredVector<uchar3> imgPredSize(predWidth*predHeight);
    dart::MirroredVector<const uchar3 *> allSdfColors(tracker->getNumModels());
    for (int m=0; m<tracker->getNumModels(); ++m) {
        allSdfColors.hostPtr()[m] = tracker->getModel(m).getDeviceSdfColors();
    }
    allSdfColors.syncHostToDevice();


    float maxRotationDamping = 200;
    float maxTranslationDamping = 10;
    float infoAccumulationRate = 0.8;
    float poseVars[1][12];

    tracker->optimizePoses();

    // update accumulated info
    for (int m=0; m<tracker->getNumModels(); ++m) {
//        if (m == 1 && trackingMode == ModeIntermediate) { continue; }
        const Eigen::MatrixXf & JTJ = *tracker->getOptimizer()->getJTJ(m);
        ROS_INFO_STREAM(JTJ);
        if (JTJ.rows() == 0) { continue; }
        Eigen::MatrixXf & dampingMatrix = tracker->getDampingMatrix(m);
        for (int i=0; i<3; ++i) {
            dampingMatrix(i,i) = std::min(maxTranslationDamping,dampingMatrix(i,i) + infoAccumulationRate*JTJ(i,i));
        }
        for (int i=3; i<tracker->getPose(m).getReducedDimensions(); ++i) {
            dampingMatrix(i,i) = std::min(maxRotationDamping,dampingMatrix(i,i) + infoAccumulationRate*JTJ(i,i));
        }
    }

    float errPerObsPoint = tracker->getOptimizer()->getErrPerObsPoint(1,0);
    float errPerModPoint = tracker->getOptimizer()->getErrPerModPoint(1,0);

//    infoLog.Log(errPerObsPoint,errPerObsPoint+errPerModPoint,stabilityThreshold,resetInfoThreshold);

    for (int m=0; m<tracker->getNumModels(); ++m) {
        for (int i=0; i<tracker->getPose(m).getReducedArticulatedDimensions(); ++i) {
            poseVars[m][i+6] = tracker->getPose(m).getReducedArticulation()[i];
        }
        dart::SE3 T_cm = tracker->getPose(m).getTransformModelToCamera();
        poseVars[m][0] = T_cm.r0.w; T_cm.r0.w = 0;
        poseVars[m][1] = T_cm.r1.w; T_cm.r1.w = 0;
        poseVars[m][2] = T_cm.r2.w; T_cm.r2.w = 0;
        dart::se3 t_cm = dart::se3FromSE3(T_cm);
        poseVars[m][3] = t_cm.p[3];
        poseVars[m][4] = t_cm.p[4];
        poseVars[m][5] = t_cm.p[5];
        ROS_INFO("%f\t%f\t%f\n%f\t%f\t%f",poseVars[0][0],poseVars[m][1],poseVars[m][2],
                 poseVars[m][3],poseVars[m][4],poseVars[m][5]);
    }

//    realsensePubRunner = true;
//    realsense_thread = boost::shared_ptr<std::thread>(new std::thread(&DartTracker::realsensePub, this));
//    realsense_thread->detach();

    publish_transform = true;
    if(transform_thread==nullptr){
        transform_thread = boost::shared_ptr<std::thread>(new std::thread(&DartTracker::transformPublisher, this));
        transform_thread->detach();
    }

    // initialize realsense pose
    realsense_tf.setOrigin(tf::Vector3(0, 0, 2.0));
    tf::Quaternion quat;
    quat.setRPY(M_PI / 2, 0, 0);
    realsense_tf.setRotation(quat);

    pointcloud = PointCloudRGB::Ptr(new PointCloudRGB);
}

DartTracker::~DartTracker(){
    realsensePubRunner = false;
    if(realsense_thread->joinable())
        realsense_thread->join();
    publish_transform = false;
    if(transform_thread->joinable())
        transform_thread->join();
}



void DartTracker::realsensePub() {
    ros::Rate rate(30);

    while(realsensePubRunner){
        realsense.advance();

        publishDepthPointCloud();

        rate.sleep();

        Mat image = Mat(480, 640, CV_8UC3, (uint8_t*)realsense.color_frame);
        imshow("camera", image);
        waitKey(1);
//        cv_bridge::CvImage cvImage;
//        image.copyTo(cvImage.image);
//        sensor_msgs::Image msg;
//        cvImage.toImageMsg(msg);
//        msg.header.frame_id = "real_sense";
//        // Publish timestamp to synchronize frames.
//        msg.header.stamp = ros::Time::now();
//        msg.width = realsense.getColorWidth();
//        msg.height = realsense.getColorHeight();
//        msg.is_bigendian = false;
//        msg.step = sizeof(unsigned char) * 3 * msg.width;
//        realsense_image_pub.publish(msg);
        rate.sleep();
    }
}

void DartTracker::transformPublisher(){
    ros::Rate rate(5);
    while(publish_transform){
        tf_broadcaster.sendTransform(tf::StampedTransform(realsense_tf, ros::Time::now(), "world", "real_sense"));
        rate.sleep();
    }
}


void DartTracker::publishDepthPointCloud(){
    static uint seq = 0;
    realsense.generatePointCloud(pointcloud);
    sensor_msgs::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(*pointcloud,pointcloud_msg);
    pointcloud_msg.header.frame_id = "real_sense";
    pointcloud_msg.header.seq = seq++;
    pointcloud_msg.header.stamp = ros::Time::now();
    realsense_depth_pub.publish(pointcloud_msg);
}

PLUGINLIB_EXPORT_CLASS(DartTracker, rviz::Panel)