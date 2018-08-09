#ifndef Q_MOC_RUN
// qt
#include <QPainter>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTableWidget>
#include <QComboBox>
#include <QTimer>
#include <QScrollArea>
#include <QListWidget>
#include <QStyledItemDelegate>
// std
#include <map>
#include <thread>
#include <mutex>
#include <iostream>
#include <stdio.h>
#include <string.h>

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <cuda_runtime.h>
#include <vector_types.h>

#include "geometry/plane_fitting.h"
#include "img_proc/img_ops.h"
#include "optimization/priors.h"
#include "tracker.h"
#include "util/dart_io.h"
#include "util/gl_dart.h"
#include "util/image_io.h"
#include "util/ostream_operators.h"
#include "util/string_format.h"
#include "visualization/color_ramps.h"
#include "visualization/data_association_viz.h"
#include "visualization/gradient_viz.h"
#include "visualization/sdf_viz.h"
// ros
#include <ros/ros.h>
#include <rviz/panel.h>
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include "dart_tracker/dartRealSense.hpp"
#include <thread>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#endif

using namespace std;
using namespace Eigen;
using namespace cv;

#define EIGEN_DONT_ALIGN

enum PointColorings {
    PointColoringNone = 0,
    PointColoringRGB,
    PointColoringErr,
    PointColoringDA,
    NumPointColorings
};

enum DebugImgs {
    DebugColor=0,
    DebugObsDepth,
    DebugPredictedDepth,
    DebugObsToModDA,
    DebugModToObsDA,
    DebugObsToModErr,
    DebugModToObsErr,
    DebugJTJ,
    DebugN
};

enum TrackingMode {
    ModeObjOnTable,
    ModeIntermediate,
    ModeObjGrasped,
    ModeObjGraspedLeft
};

const static int panelWidth = 180;

static const int fullArmFingerTipFrames[10] = { 11, 15, 19, 23, 27,  38, 42, 46, 50, 54 };
static const int handFingerTipFrames[5] = { 4, 8, 12, 16, 20 };

class DartTracker : public rviz::Panel{
    Q_OBJECT
public:
    DartTracker();
    ~DartTracker();
    /**
    * Load all configuration data for this panel from the given Config object.
    * @param config rviz config file
    */
    virtual void load(const rviz::Config &config){};

    /**
     * Save all configuration data from this panel to the given
     * Config object.  It is important here that you call save()
     * on the parent class so the class id and panel name get saved.
     * @param config rviz config file
     */
    virtual void save(rviz::Config config) const{};

    void realsensePub();

    void transformPublisher();

    void publishDepthPointCloud();

private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    dart::Tracker *tracker;
    dart::ParamMapPoseReduction *poseReduction;
    float defaultModelSdfPadding = 0.07;
    DartRealSense<uint16_t,uchar3> realsense;

    boost::shared_ptr<std::thread> realsense_thread, transform_thread;
    bool realsensePubRunner = true, publish_transform = true;
    ros::Publisher realsense_image_pub, realsense_depth_pub;
    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform realsense_tf;
    PointCloudRGB::Ptr pointcloud;
};