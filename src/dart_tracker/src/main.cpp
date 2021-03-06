#include "dart_tracker/dartTracker.hpp"
#include <GL/glx.h>

int main(int argc, char *argv[]){
    /* ROS setup */
    if (!ros::isInitialized()) {
        int argc = 0;
        char *argv = nullptr;
        ros::init(argc, &argv, "roboy_dart_tracker",
                  ros::init_options::AnonymousName |
                  ros::init_options::NoRosout);
    }
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);

    ros::Publisher realsense_image_pub = nh->advertise<sensor_msgs::Image>("/DartTracker/realsense_color", 1);
    ros::Publisher realsense_depth_pub = nh->advertise<PointCloud>("/DartTracker/realsense_depth", 1);

    boost::shared_ptr<ros::AsyncSpinner> spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    /*For use of OpenGL / GPU*/
    cudaSetDevice(0);
    cudaDeviceReset();
    glutInit(&argc, argv);
    glutCreateWindow("GLUT");

    glewInit();
    printf("OpenGL version supported by this platform (%s): \n", glGetString(GL_VERSION));

    /*Setup of Dart tracker, models and depthimage source*/
    dart::Tracker *tracker = new dart::Tracker;

    const static int obsSdfSize = 64;
    const static float obsSdfResolution = 0.01*32/obsSdfSize;
    const static float defaultModelSdfResolution = 2e-3; //1.5e-3;
    const static float3 obsSdfOffset = make_float3(0,0,0.1);

    tracker->addModel("/home/roboy/workspace/DartTracker/src/dart_tracker/models/testModel/shank.xml",
                      0.5*defaultModelSdfResolution,
                      0.5*0.5*defaultModelSdfResolution,
                      64);

    // Depth image source
    DartRealSense<uint16_t,uchar3> realsense;

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

    /*Some vars (?)*/
    float maxRotationDamping = 200;
    float maxTranslationDamping = 10;
    float infoAccumulationRate = 0.8;
    float poseVars[1][12];

    dart::OptimizationOptions & opts = tracker->getOptions();
    opts.lambdaObsToMod = 1;
    memset(opts.lambdaIntersection.data(),0,tracker->getNumModels()*tracker->getNumModels()*sizeof(float));
    opts.contactThreshold = 0.02;
    opts.planeNormal[0] =  make_float3(0,0,1);
    opts.planeNormal[2] = make_float3(0,0,1);
    opts.planeNormal[1] = make_float3(0,0,0);
    opts.regularization[0] = opts.regularization[1] = opts.regularization[2] = 0.01;

    float normalThreshold = -1.01f;
    float distanceThreshold = 0.0f;
    float handRegularization = 0.0f;
    float objectRegularization = 1.0f;
    float resetInfoThreshold = 1e-5;
    float stabilityThreshold = 5e-5;
    float lambdaModToObs = 0;
    float lambdaObsToMod = 0;
    float lambdaIntersection  = 1.0f;
    float lambdaContact = 0;
    float planeOffset = -0.05f;

    PointCloudRGB::Ptr pointcloud = PointCloudRGB::Ptr(new PointCloudRGB);
    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform realsense_tf;
    realsense_tf.setOrigin(tf::Vector3(0, 0, 2.0));
    tf::Quaternion quat;
    quat.setRPY(M_PI / 2, 0, 0);
    realsense_tf.setRotation(quat);

    /*Main loop - only works while connection to roscore*/
    while(ros::ok()) {
        realsense.advance();

        Mat image = Mat(480, 640, CV_8UC3, (uint8_t*)realsense.color_frame);
        imshow("camera", image);
        waitKey(1);

        /*Get and publish point cloud*/
        static uint seq = 0;
        realsense.generatePointCloud(pointcloud);
        sensor_msgs::PointCloud2 pointcloud_msg;
        pcl::toROSMsg(*pointcloud,pointcloud_msg);
        pointcloud_msg.header.frame_id = "real_sense";
        pointcloud_msg.header.seq = seq++;
        pointcloud_msg.header.stamp = ros::Time::now();
        realsense_depth_pub.publish(pointcloud_msg);

        /*Some stuff (?)*/
        tf_broadcaster.sendTransform(tf::StampedTransform(realsense_tf, ros::Time::now(), "world", "real_sense"));

        opts.lambdaIntersection[0 + 3*0] = lambdaIntersection; // right
        opts.lambdaIntersection[2 + 3*2] = lambdaIntersection; // left

        opts.lambdaIntersection[1 + 3*0] = lambdaIntersection; // object->right
        opts.lambdaIntersection[0 + 3*1] = lambdaIntersection; // right->object

        opts.lambdaIntersection[1 + 3*2] = lambdaIntersection; // object->left
        opts.lambdaIntersection[2 + 3*1] = lambdaIntersection; // left->object

        opts.normThreshold = normalThreshold;
        for (int m=0; m<tracker->getNumModels(); ++m) {
            opts.distThreshold[m] = distanceThreshold;
        }
        opts.regularization[0] = opts.regularization[1] = opts.regularization[2] = 0.01;
        opts.regularizationScaled[0] = handRegularization;
        opts.regularizationScaled[1] = objectRegularization;
        opts.regularizationScaled[2] = handRegularization;
        opts.planeOffset[2] = planeOffset;
        opts.lambdaObsToMod = lambdaObsToMod;
        opts.lambdaModToObs = lambdaModToObs;
        opts.planeOffset[0] = planeOffset;
        opts.numIterations = 3;

        //Magic!
        tracker->optimizePoses();

        // update accumulated info
        for (int m = 0; m < tracker->getNumModels(); ++m) {
            //if (m == 1 && trackingMode == ModeIntermediate) { continue; }
            const Eigen::MatrixXf &JTJ = *tracker->getOptimizer()->getJTJ(m);
            ROS_INFO_STREAM("\n" << JTJ);
            if (JTJ.rows() == 0) { continue; }
            Eigen::MatrixXf &dampingMatrix = tracker->getDampingMatrix(m);
            for (int i = 0; i < 3; ++i) {
                dampingMatrix(i, i) = std::min(maxTranslationDamping,
                                               dampingMatrix(i, i) + infoAccumulationRate * JTJ(i, i));
            }
            for (int i = 3; i < tracker->getPose(m).getReducedDimensions(); ++i) {
                dampingMatrix(i, i) = std::min(maxRotationDamping,
                                               dampingMatrix(i, i) + infoAccumulationRate * JTJ(i, i));
            }
        }

        float errPerObsPoint = tracker->getOptimizer()->getErrPerObsPoint(1, 0);
        float errPerModPoint = tracker->getOptimizer()->getErrPerModPoint(1, 0);

        ROS_INFO("\nerrPerObsPoint: %f\t\terrPerModPoint: %f", errPerObsPoint, errPerModPoint);

//    infoLog.Log(errPerObsPoint,errPerObsPoint+errPerModPoint,stabilityThreshold,resetInfoThreshold);
        // update poses (?)
        for (int m = 0; m < tracker->getNumModels(); ++m) {
            for (int i = 0; i < tracker->getPose(m).getReducedArticulatedDimensions(); ++i) {
                poseVars[m][i + 6] = tracker->getPose(m).getReducedArticulation()[i];
            }
            dart::SE3 T_cm = tracker->getPose(m).getTransformModelToCamera();
            poseVars[m][0] = T_cm.r0.w;
            T_cm.r0.w = 0;
            poseVars[m][1] = T_cm.r1.w;
            T_cm.r1.w = 0;
            poseVars[m][2] = T_cm.r2.w;
            T_cm.r2.w = 0;
            dart::se3 t_cm = dart::se3FromSE3(T_cm);
            poseVars[m][3] = t_cm.p[3];
            poseVars[m][4] = t_cm.p[4];
            poseVars[m][5] = t_cm.p[5];
            ROS_INFO("\n%f\t%f\t%f\n%f\t%f\t%f", poseVars[0][0], poseVars[m][1], poseVars[m][2],
                     poseVars[m][3], poseVars[m][4], poseVars[m][5]);
        }

        tracker->stepForward();
    }

//    realsensePubRunner = true;
//    realsense_thread = boost::shared_ptr<std::thread>(new std::thread(&DartTracker::realsensePub, this));
//    realsense_thread->detach();

    return 0;
}
