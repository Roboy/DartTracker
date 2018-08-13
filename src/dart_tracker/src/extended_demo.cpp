#include <iostream>
#include <stdio.h>
#include <string.h>

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <pangolin/pangolin.h>
#include <pangolin/utils/timer.h>

#include <Eigen/Dense>

#include <cuda_runtime.h>
#include <vector_types.h>

#include "depth_sources/image_depth_source.h"
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
#include "geometry_msgs/Pose.h"

#define SHOWDEPTH true


#include "dart_tracker/dartTracker.hpp"

using namespace std;

//initial coords object?
static const dart::SE3 initialT_co(make_float4(0.262348, -0.955909, -0.131952, 0.0238097),
                                   make_float4(-0.620357, -0.271813, 0.735714, -0.178571),
                                   make_float4(-0.739142, -0.111156, -0.664314, 0.702381));



static float3 initialTableNorm = make_float3(0.0182391, 0.665761, -0.745942);
static float initialTableIntercept = -0.705196;

int main() {
    const float objObsSdfRes = 0.0025;
    const float3 objObsSdfOffset = make_float3(0,0,0);

    const std::string videoLoc = "src/dartExample/video";

    // -=-=-=- initializations -=-=-=-
    cudaSetDevice(0);
    cudaDeviceReset();

    pangolin::CreateWindowAndBind("Main",640+4*panelWidth+1,2*480+1);

    glewInit();
    dart::Tracker tracker;

    // -=-=-=- pangolin window setup -=-=-=-

    pangolin::CreatePanel("lim").SetBounds(0.0,1.0,1.0,pangolin::Attach::Pix(-panelWidth));
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(panelWidth));
    pangolin::CreatePanel("opt").SetBounds(0.0,1.0,pangolin::Attach::Pix(panelWidth), pangolin::Attach::Pix(2*panelWidth));
    pangolin::CreatePanel("pose").SetBounds(0.0,1.0,pangolin::Attach::Pix(-panelWidth), pangolin::Attach::Pix(-2*panelWidth));

    int glWidth = 640;
    int glHeight= 480;
    int glFL = 400;
    int glPPx = glWidth/2;
    int glPPy = glHeight/2;
    pangolin::OpenGlMatrixSpec glK = pangolin::ProjectionMatrixRDF_BottomLeft(glWidth,glHeight,glFL,glFL,glPPx,glPPy,0.01,1000);
    pangolin::OpenGlRenderState camState(glK);
    pangolin::View & camDisp = pangolin::Display("cam").SetAspect(640.0f/480.0f).SetHandler(new pangolin::Handler3D(camState));

    pangolin::View & imgDisp = pangolin::Display("img").SetAspect(640.0f/480.0f);
    pangolin::GlTexture imgTexDepthSize(320,240);
    pangolin::GlTexture imgTexPredictionSize(160,120);

    pangolin::DataLog infoLog;
    {
        std::vector<std::string> infoLogLabels;
        infoLogLabels.push_back("errObsToMod");
        infoLogLabels.push_back("errModToObs");
        infoLogLabels.push_back("stabilityThreshold");
        infoLogLabels.push_back("resetThreshold");
        infoLog.SetLabels(infoLogLabels);
    }

    pangolin::Display("multi")
            .SetBounds(1.0, 0.0, pangolin::Attach::Pix(2*panelWidth), pangolin::Attach::Pix(-2*panelWidth))
            .SetLayout(pangolin::LayoutEqual)
            .AddDisplay(camDisp)
                    //            .AddDisplay(infoPlotter)
            .AddDisplay(imgDisp);

    float defaultModelSdfPadding = 0.07;

    std::vector<pangolin::Var<float> *> sizeVars;
    printf("Using RealSense!\n");
    /* ROS setup */
    if (!ros::isInitialized()) {
        int argc = 0;
        char *argv = nullptr;
        ros::init(argc, &argv, "roboy_dart_tracker",
                  ros::init_options::AnonymousName |
                  ros::init_options::NoRosout);
    }

    //ros and rviz related
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);

    ros::Publisher realsense_image_pub = nh->advertise<sensor_msgs::Image>("DartTracker/realsense_color", 1);
    ros::Publisher realsense_depth_pub = nh->advertise<PointCloud>("DartTracker/realsense_depth", 1);
    ros::Publisher xylophone_pub = nh->advertise<geometry_msgs::Pose>("DartTracker/xylophone_pose/", 1);
    boost::shared_ptr<ros::AsyncSpinner> spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    PointCloudRGB::Ptr pointcloud = PointCloudRGB::Ptr(new PointCloudRGB);

    //publishing stuff
    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform realsense_tf;

    //---- DEPTH SOURCE --- multiple depth source not supported yet -> change order, first one gets priority
    DartRealSense<uint16_t,uchar3> *depthSource = new DartRealSense<uint16_t,uchar3>(640, 480);
    tracker.addDepthSource(depthSource);

    const int depthWidth = depthSource->getDepthWidth();
    const int depthHeight = depthSource->getDepthHeight();

    //visualisation
    //temp buffer to display depth in extra window
    uint16_t *tempDepth = (uint16_t *) malloc(depthSource->getDepthHeight()*depthSource->getDepthWidth() * sizeof(uint16_t));

    // ----

    cv::Mat image = cv::Mat();
    dart::Optimizer & optimizer = *tracker.getOptimizer();

    const static int obsSdfSize = 64;
    const static float obsSdfResolution = 0.01*32/obsSdfSize;
    const static float defaultModelSdfResolution = 2e-3; //1.5e-3;
    const static float3 obsSdfOffset = make_float3(0,0,0.1);

    pangolin::Var<float> modelSdfResolution("lim.modelSdfResolution",defaultModelSdfResolution,defaultModelSdfResolution/2,defaultModelSdfResolution*2);
    pangolin::Var<float> modelSdfPadding("lim.modelSdfPadding",defaultModelSdfPadding,defaultModelSdfPadding/2,defaultModelSdfPadding*2);

    dart::ParamMapPoseReduction * handPoseReduction = dart::loadParamMapPoseReduction("src/dartExample/models/spaceJustin/justinHandParamMap.txt");


    dart::LinearPoseReduction * justinPoseReduction = dart::loadLinearPoseReduction("src/dartExample/models/spaceJustin/spaceJustinPoseReduction.txt");

    //IMPORTANT: Xylophone model must come first, or change getPose(0) further down when publishing xylophone pos
    if(!tracker.addModel( "src/dart_tracker/models/xylophone/xylophone.xml",//"src/dart_tracker/models/mug/mug.xml"; //
                     0.5*modelSdfResolution,
                     modelSdfPadding,
                     64)){
        printf("Failed to load model!\n");
    }

    /*if(!tracker.addModel("src/dart_tracker/models/ikeaMug/ikeaMug.xml", //"xylophone/xylophone.xml" , //roboy_xylophone_left_arm/roboy_xylophone_left_arm.xml",
                     0.5*modelSdfResolution,
                     modelSdfPadding,
                     64))
        printf("Failed to load model!\n");
    */
    std::vector<pangolin::Var<float> * *> poseVars;

    pangolin::Var<bool> sliderControlled("pose.sliderControl",false,true);
    for (int m=0; m<tracker.getNumModels(); ++m) {

        const int dimensions = tracker.getModel(m).getPoseDimensionality();

        pangolin::Var<float> * * vars = new pangolin::Var<float> *[dimensions];
        poseVars.push_back(vars);
        string model_name = (tracker.getModel(m).getName()!= "")? string(tracker.getModel(m).getName()) : string("%d",m);
        poseVars[m][0] = new pangolin::Var<float>(dart::stringFormat("pose.%s x",model_name.c_str()),0,-0.5,0.5);
        poseVars[m][1] = new pangolin::Var<float>(dart::stringFormat("pose.%s y",model_name.c_str()),0,-0.5,0.5);
        poseVars[m][2] = new pangolin::Var<float>(dart::stringFormat("pose.%s z",model_name.c_str()),0.3,0.5,1.5);
        poseVars[m][3] = new pangolin::Var<float>(dart::stringFormat("pose.%s wx",model_name.c_str()),    0,-M_PI,M_PI);
        poseVars[m][4] = new pangolin::Var<float>(dart::stringFormat("pose.%s wy",model_name.c_str()),    0,-M_PI,M_PI);
        poseVars[m][5] = new pangolin::Var<float>(dart::stringFormat("pose.%s wz",model_name.c_str()), M_PI,-M_PI,M_PI);

        const dart::Pose & pose = tracker.getPose(m);
        for (int i=0; i<pose.getReducedArticulatedDimensions(); ++i) {
            poseVars[m][i+6] = new pangolin::Var<float>(dart::stringFormat("pose.%d %s",m,pose.getReducedName(i).c_str()),0,pose.getReducedMin(i),pose.getReducedMax(i));
        }

    }

    // pangolin variables
    static pangolin::Var<bool> trackFromVideo("ui.track",false,false,true);
    static pangolin::Var<bool> stepVideo("ui.stepVideo",false,false);
    static pangolin::Var<bool> stepVideoBack("ui.stepVideoBack",false,false);

    static pangolin::Var<float> sigmaPixels("ui.sigmaPixels",3.0,0.01,4);
    static pangolin::Var<float> sigmaDepth("ui.sigmaDepth",0.1,0.001,1);
    static pangolin::Var<float> focalLength("ui.focalLength",depthSource->getFocalLength().x,0.8*depthSource->getFocalLength().x,1.2*depthSource->getFocalLength().x);//475,525); //525.0,450.0,600.0);
    static pangolin::Var<bool> showEstimatedPose("ui.showEstimate",true,true);

    static pangolin::Var<float> levelSet("ui.levelSet",0.0,-10.0,10.0);
    static pangolin::Var<bool> showTrackedPoints("ui.showPoints",true,true);
    static pangolin::Var<int> pointColoringObs("ui.pointColoringObs",0,0,NumPointColorings-1);
    static pangolin::Var<int> pointColoringPred("ui.pointColoringPred",0,0,NumPointColorings-1);

    static pangolin::Var<float> planeOffset("ui.planeOffset",-0.03,-0.05,0);

    static pangolin::Var<int> debugImg("ui.debugImg",DebugN,0,DebugN);

    static pangolin::Var<bool> showObsSdf("ui.showObsSdf",false,true);
    static pangolin::Var<bool> showPredictedPoints("ui.showPredictedPoints",false,true);
    static pangolin::Var<bool> showCollisionClouds("ui.showCollisionClouds",false,true);

    static pangolin::Var<float> fps("ui.fps",0);

    // optimization options
    //variable style: name, initial value, min possible val, max possible val
    //TODO: VERY IMPORTANT: KEEP INITIAL VALUES -> OTHERWISE TRACKING WILL NOT WORK
    pangolin::Var<bool> iterateButton("opt.iterate",false,false);
    pangolin::Var<int> itersPerFrame("opt.itersPerFrame",3,0,30);
    pangolin::Var<float> normalThreshold("opt.normalThreshold",-1.01,-1.01,1.0);
    pangolin::Var<float> distanceThreshold("opt.distanceThreshold",0.035,0.0,0.1);
    pangolin::Var<float> handRegularization("opt.handRegularization",0.1,0,10); // 1.0
    pangolin::Var<float> objectRegularization("opt.objectRegularization",0.1,0,10); // 1.0
    pangolin::Var<float> resetInfoThreshold("opt.resetInfoThreshold",1.0e-5,1e-5,2e-5);
    pangolin::Var<float> stabilityThreshold("opt.stabilityThreshold",7.5e-6,5e-6,1e-5);
    pangolin::Var<float> lambdaModToObs("opt.lambdaModToObs",0.5,0,1);
    pangolin::Var<float> lambdaObsToMod("opt.lambdaObsToMod",1,0,1);
    pangolin::Var<float> lambdaIntersection("opt.lambdaIntersection",1.f,0,40);
    //pangolin::Var<float> selfIntersectWeight("opt.selfIntersectWeight",atof(argv[2]),0,40);
    pangolin::Var<float> lambdaContact("opt.lambdaContact",1.f,0,200);


    pangolin::Var<float> infoAccumulationRate("opt.infoAccumulationRate",0.1,0.0,1.0); // 0.8
    pangolin::Var<float> maxRotationDamping("opt.maxRotationalDamping",50,0,200);
    pangolin::Var<float> maxTranslationDamping("opt.maxTranslationDamping",5,0,10);

    pangolin::Var<float> tableNormX("opt.tableNormX",initialTableNorm.x,-1,1);
    pangolin::Var<float> tableNormY("opt.tableNormY",initialTableNorm.y,-1,1);
    pangolin::Var<float> tableNormZ("opt.tableNormZ",initialTableNorm.z,-1,1);
    pangolin::Var<float> tableIntercept("opt.tableIntercept",initialTableIntercept,-1,1);

    static pangolin::Var<bool> fitTable("opt.fitTable",true,true);
    static pangolin::Var<bool> subtractTable("opt.subtractTable",true,true);



    int fpsWindow = 10;
    pangolin::basetime lastTime = pangolin::TimeNow();

    const int predWidth = tracker.getPredictionWidth();
    const int predHeight = tracker.getPredictionHeight();

    dart::MirroredVector<uchar3> imgDepthSize(depthWidth*depthHeight);
    dart::MirroredVector<uchar3> imgPredSize(predWidth*predHeight);
    dart::MirroredVector<const uchar3 *> allSdfColors(tracker.getNumModels());
    for (int m=0; m<tracker.getNumModels(); ++m) {
        allSdfColors.hostPtr()[m] = tracker.getModel(m).getDeviceSdfColors();
    }
    allSdfColors.syncHostToDevice();

    // set up VBO to display point cloud
    //GL unsigned integer type
    GLuint pointCloudVbo,pointCloudColorVbo,pointCloudNormVbo;
    //generates buffer
    glGenBuffersARB(1,&pointCloudVbo);
    //makes buffer accessible
    glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudVbo);
    // stores data in array: Use DYNAMIC_DRAW when the data store contents will be modified repeatedly and used many times.
    glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(float4), tracker.getHostVertMap(),GL_DYNAMIC_DRAW_ARB); //NULL, GL_DYNAMIC_DRAW_ARB);

    glGenBuffersARB(1,&pointCloudColorVbo);
    glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudColorVbo);
    glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(uchar3),imgDepthSize.hostPtr(),GL_DYNAMIC_DRAW_ARB);

    glGenBuffersARB(1,&pointCloudNormVbo);
    glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudNormVbo);
    glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(float4),tracker.getHostNormMap(),GL_DYNAMIC_DRAW_ARB);

    dart::OptimizationOptions & opts = tracker.getOptions();
    opts.lambdaObsToMod = 1;
    memset(opts.lambdaIntersection.data(),0,tracker.getNumModels()*tracker.getNumModels()*sizeof(float));
    opts.contactThreshold = 0.02;
    opts.planeNormal[0] =  make_float3(0,0,1);
    opts.planeNormal[2] = make_float3(0,0,1);
    opts.planeNormal[1] = make_float3(0,0,0);
    //opts.planeNormal[3] = make_float3(0,0,0);
    opts.regularization[0] = opts.regularization[1] = opts.regularization[2] = 0.01; //= opts.regularization[3]

    float3 initialContact = make_float3(0,0.02,0);


    //enable tracking by default
    trackFromVideo = true;
    //enable color of point cloud using depth cam values
    pointColoringObs = 1;

    //Tracking won't start until initial position of objects set
    sliderControlled = true;


    // ------------------- main loop ---------------------
    for (int pangolinFrame=1; !pangolin::ShouldQuit(); ++pangolinFrame) {

        /*REALSENSE STUFF*/
        image.release();
        depthSource->advance();
        //displaying data
        if(depthSource->hasColor() && !SHOWDEPTH){
            //important: first height, then width
            cv::Mat temp = cv::Mat(depthSource->getDepthHeight(), depthSource->getDepthWidth(), CV_8UC3, (uint8_t*)depthSource->getColor());
            //mirror and change colours
            cv::flip(temp, image, 0);
            cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        }
        else{
            memcpy(tempDepth, depthSource->getDepth(), depthSource->getDepthWidth()*depthSource->getDepthHeight() * sizeof(uint16_t));
            //mirror and change colours
            //important: first height, then width
            cv::Mat temp = cv::Mat(depthSource->getDepthHeight(), depthSource->getDepthWidth(), CV_16U, tempDepth);
            //multiply all depth values by 100 to be able to see these
            for(int i = 0; i < depthSource->getDepthWidth() * depthSource->getDepthHeight(); i++) {
                *(tempDepth + i)  *= 100;
            }
            //flip along both axis (depth img upside down otherwise)
            cv::flip(temp, image, 0);
            temp.release();

        }
        //show
        cv::imshow("camera", image);
        cv::waitKey(1);

        //Get and publish point cloud. This can be displayed in RVIZ to see what the camera perceives (depth and color)
        depthSource->generatePointCloud(pointcloud);
        static uint seq = 0;
        sensor_msgs::PointCloud2 pointcloud_msg;
        pcl::toROSMsg(*pointcloud,pointcloud_msg);
        pointcloud_msg.header.frame_id = "real_sense";
        pointcloud_msg.header.seq = seq++;
        pointcloud_msg.header.stamp = ros::Time::now();
        realsense_depth_pub.publish(pointcloud_msg);
        //END REALSENSE

        if (pangolin::HasResized()) {
            pangolin::DisplayBase().ActivateScissorAndClear();
        }


        opts.focalLength = focalLength;
        opts.normThreshold = normalThreshold;
        //automatical val update ? -> ignore?
        for (int m=0; m<tracker.getNumModels(); ++m) {
            opts.distThreshold[m] = distanceThreshold;
        }
        opts.regularization[0] = opts.regularization[1] = opts.regularization[2]  = 0.01; //= opts.regularization[3]
        opts.regularizationScaled[0] = handRegularization;
        opts.regularizationScaled[1] = objectRegularization;
        opts.regularizationScaled[2] = handRegularization;
        opts.planeOffset[2] = planeOffset;
        opts.lambdaObsToMod = lambdaObsToMod;
        opts.lambdaModToObs = lambdaModToObs;
        opts.planeOffset[0] = planeOffset;
        opts.debugObsToModDA = pointColoringObs == PointColoringDA || (debugImg == DebugObsToModDA);
        opts.debugModToObsDA = debugImg == DebugModToObsDA;
        opts.debugObsToModErr = ((pointColoringObs == PointColoringErr) || (debugImg == DebugObsToModErr));
        opts.debugModToObsErr = ((pointColoringPred == PointColoringErr) || (debugImg == DebugModToObsErr));
        opts.debugJTJ = (debugImg == DebugJTJ);
        opts.numIterations = itersPerFrame;

        if (pangolin::Pushed(stepVideoBack)) {
            tracker.stepBackward();
        }

        bool iteratePushed = Pushed(iterateButton);

        if (pangolinFrame % fpsWindow == 0) {
            pangolin::basetime time = pangolin::TimeNow();
            if (trackFromVideo) {
                static int totalFrames = 0;
                static double totalTime = 0;
                totalFrames += fpsWindow;
                totalTime += pangolin::TimeDiff_s(lastTime,time);
                fps = totalFrames / totalTime;
            } else {
                fps = fpsWindow / pangolin::TimeDiff_s(lastTime,time);
            }
            lastTime = time;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        // Process this frame                                                                                   //
        //                                                                                                      //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        {

            static pangolin::Var<bool> filteredNorms("ui.filteredNorms",false,true);
            static pangolin::Var<bool> filteredVerts("ui.filteredVerts",false,true);

            if (filteredNorms.GuiChanged()) {
                tracker.setFilteredNorms(filteredNorms);
            } else if (filteredVerts.GuiChanged()) {
                tracker.setFilteredVerts(filteredVerts);
            } else if (sigmaDepth.GuiChanged()) {
                tracker.setSigmaDepth(sigmaDepth);
            } else if (sigmaPixels.GuiChanged()) {
                tracker.setSigmaPixels(sigmaPixels);
            }

            // update pose based on sliders in pangolin window
            if (sliderControlled) {
                for (int m=0; m<tracker.getNumModels(); ++m) {
                    for (int i=0; i<tracker.getPose(m).getReducedArticulatedDimensions(); ++i) {
                        tracker.getPose(m).getReducedArticulation()[i] = *poseVars[m][i+6];
                    }
                    tracker.getPose(m).setTransformModelToCamera(dart::SE3Fromse3(dart::se3(*poseVars[m][0],*poseVars[m][1],*poseVars[m][2],0,0,0))*
                                                                 dart::SE3Fromse3(dart::se3(0,0,0,*poseVars[m][3],*poseVars[m][4],*poseVars[m][5])));
                    tracker.updatePose(m);
                }
            } else {
                //TRACKING
                // run optimization method
                tracker.optimizePoses();

                // update accumulated info
                for (int m=0; m<tracker.getNumModels(); ++m) {

                    const Eigen::MatrixXf & JTJ = *tracker.getOptimizer()->getJTJ(m);
                    if (JTJ.rows() == 0) { continue; }
                    //used when calculating new JTJ matrix?
                    Eigen::MatrixXf & dampingMatrix = tracker.getDampingMatrix(m);
                    for (int i=0; i<3; ++i) {
                        dampingMatrix(i,i) = std::min((float)maxTranslationDamping,dampingMatrix(i,i) + infoAccumulationRate*JTJ(i,i));
                    }
                    for (int i=3; i<tracker.getPose(m).getReducedDimensions(); ++i) {
                        dampingMatrix(i,i) = std::min((float)maxRotationDamping,dampingMatrix(i,i) + infoAccumulationRate*JTJ(i,i));
                    }
                }

                float errPerObsPoint = optimizer.getErrPerObsPoint(1,0);
                float errPerModPoint = optimizer.getErrPerModPoint(1,0);

                infoLog.Log(errPerObsPoint,errPerObsPoint+errPerModPoint,stabilityThreshold,resetInfoThreshold);

                //update visualized values in pangolin
                for (int m=0; m<tracker.getNumModels(); ++m) {
                    for (int i=0; i<tracker.getPose(m).getReducedArticulatedDimensions(); ++i) {
                        *poseVars[m][i+6] = tracker.getPose(m).getReducedArticulation()[i];
                    }
                    dart::SE3 T_cm = tracker.getPose(m).getTransformModelToCamera();
                    // TODO why set values 0?????
                    *poseVars[m][0] = T_cm.r0.w; T_cm.r0.w = 0;
                    *poseVars[m][1] = T_cm.r1.w; T_cm.r1.w = 0;
                    *poseVars[m][2] = T_cm.r2.w; T_cm.r2.w = 0;
                    dart::se3 t_cm = dart::se3FromSE3(T_cm);
                    *poseVars[m][3] = t_cm.p[3];
                    *poseVars[m][4] = t_cm.p[4];
                    *poseVars[m][5] = t_cm.p[5];
                }

            }

        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        //                                                                                                      //
        // Render this frame                                                                                    //
        //                                                                                                      //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        glClearColor (1.0, 1.0, 1.0, 1.0);
        glShadeModel (GL_SMOOTH);
        float4 lightPosition = make_float4(normalize(make_float3(-0.4405,-0.5357,-0.619)),0);
        glLightfv(GL_LIGHT0, GL_POSITION, (float*)&lightPosition);

        camDisp.ActivateScissorAndClear(camState);

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHT0);
        glEnable(GL_NORMALIZE);
        glEnable(GL_LIGHTING);

        camDisp.ActivateAndScissor(camState);

        glPushMatrix();

        glColor4ub(0xff,0xff,0xff,0xff);
        if (showEstimatedPose) {

            glEnable(GL_COLOR_MATERIAL);

            glPushMatrix();

            for (int m=0; m<tracker.getNumModels(); ++m) {
                tracker.updatePose(m);
                tracker.getModel(m).render();
            }

            glPopMatrix();

        }

        glPointSize(1.0f);


        if (showObsSdf) {
            static pangolin::Var<float> levelSet("ui.levelSet",0,-10,10);

            for (int m=0; m<tracker.getNumModels(); ++m) {

                glPushMatrix();
                dart::glMultSE3(tracker.getModel(m).getTransformModelToCamera());
                tracker.getModel(m).syncObsSdfDeviceToHost();
                dart::Grid3D<float> * obsSdf = tracker.getModel(m).getObsSdf();
                tracker.getModel(m).renderSdf(*obsSdf,levelSet);
                glPopMatrix();
            }
        }

        if (showTrackedPoints) {

            glPointSize(4.0f);
            glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudVbo);
            glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(float4), tracker.getHostVertMap(),GL_DYNAMIC_DRAW_ARB); //NULL, GL_DYNAMIC_DRAW_ARB);
            //specifies buffer to contain vertices, important to know for drawarray()
            glEnableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);
            //defines array of vertex data:
            //(1) size: number of coords p. vertex elem {2,3,4}
            //(2) type: type of each coord of the array,
            //(3) offset: betw. consecutive vertices
            //(4) pointer: at first coord
            glVertexPointer(4, GL_FLOAT, 0, 0);

            switch (pointColoringObs) {
                case PointColoringNone:
                    //shows spoints which are given from depthsouce?
                    glColor3f(0.25,1,0.25);
                    glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudNormVbo);
                    glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(float4),tracker.getHostNormMap(),GL_DYNAMIC_DRAW_ARB);

                    glNormalPointer(GL_FLOAT, 4*sizeof(float), 0);
                    glEnableClientState(GL_NORMAL_ARRAY);
                    break;
                case PointColoringRGB:
                    glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudColorVbo);
                    glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(uchar3),depthSource->getAdjustedColor(),GL_DYNAMIC_DRAW_ARB);

                    glColorPointer(3,GL_UNSIGNED_BYTE, 0, 0);
                    glEnableClientState(GL_COLOR_ARRAY);
                    glDisable(GL_LIGHTING);
                    break;
                case PointColoringErr:
                {
                    static float errorMin = 0.0;
                    static float errorMax = 0.1;
                    float * dErr;
                    cudaMalloc(&dErr,depthWidth*depthHeight*sizeof(float));
                    dart::imageSquare(dErr,tracker.getDeviceDebugErrorObsToMod(),depthWidth,depthHeight);
                    dart::colorRampHeatMapUnsat(imgDepthSize.devicePtr(),dErr,depthWidth,depthHeight,errorMin,errorMax);
                    cudaFree(dErr);
                    imgDepthSize.syncDeviceToHost();
                    glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudColorVbo);
                    glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(uchar3),imgDepthSize.hostPtr(),GL_DYNAMIC_DRAW_ARB);
                    glColorPointer(3,GL_UNSIGNED_BYTE, 0, 0);
                    glEnableClientState(GL_COLOR_ARRAY);
                    glDisable(GL_LIGHTING);
                }
                    break;
                case PointColoringDA:
                {
                    const int * dDebugDA = tracker.getDeviceDebugDataAssociationObsToMod();
                    dart::colorDataAssociationMultiModel(imgDepthSize.devicePtr(),dDebugDA,allSdfColors.devicePtr(),depthWidth,depthHeight);
                    imgDepthSize.syncDeviceToHost();
                    glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudColorVbo);
                    glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(uchar3),imgDepthSize.hostPtr(),GL_DYNAMIC_DRAW_ARB);
                    glColorPointer(3,GL_UNSIGNED_BYTE, 0, 0);
                    glEnableClientState(GL_COLOR_ARRAY);
                    glDisable(GL_LIGHTING);
                }
                    break;
            }
            //This transforms the previously defined buffers into the points seen in the application and draws these
            glDrawArrays(GL_POINTS,0,depthWidth*depthHeight);
            glBindBuffer(GL_ARRAY_BUFFER_ARB,0);

            glDisableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_COLOR_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);

            glPointSize(1.0f);

        }


        static pangolin::Var<bool> showFingerContacts("ui.showContacts",true,true);

        if (showPredictedPoints) {

            glPointSize(4.0f);

            const float4 * dPredictedVertMap = tracker.getDevicePredictedVertMap();
            const int nPoints = tracker.getPredictionWidth()*tracker.getPredictionHeight();
            float4 * hPredictedVertMap = new float4[nPoints];
            cudaMemcpy(hPredictedVertMap,dPredictedVertMap,nPoints*sizeof(float4),cudaMemcpyDeviceToHost);


            glDisable(GL_LIGHTING);
            glBegin(GL_POINTS);

            if (pointColoringPred == PointColoringErr) {
                static pangolin::Var<float> errMin("ui.errMin",0,0,0.05);
                static pangolin::Var<float> errMax("ui.errMax",0.01,0,0.05);
                dart::MirroredVector<uchar3> debugImg(tracker.getPredictionWidth()*tracker.getPredictionHeight());
                dart::colorRampHeatMapUnsat(debugImg.devicePtr(),
                                            tracker.getDeviceDebugErrorModToObs(),
                                            depthWidth,depthHeight,
                                            errMin,errMax);
                debugImg.syncDeviceToHost();

                for (int i=0; i<nPoints; ++i) {
                    if (hPredictedVertMap[i].z > 0) {
                        uchar3 color = debugImg.hostPtr()[i];
                        glColor3ubv((unsigned char*)&color);
                        glVertex3f(hPredictedVertMap[i].x,hPredictedVertMap[i].y,hPredictedVertMap[i].z);
                    }
                }

            } else {

                for (int i=0; i<nPoints; ++i) {
                    if (hPredictedVertMap[i].z > 0) {
                        int id = round(hPredictedVertMap[i].w);
                        int model = id >> 16;
                        int sdf = id & 65535;
                        uchar3 color = tracker.getModel(model).getSdfColor(sdf);
                        glColor3ubv((unsigned char*)&color);
                        glVertex3f(hPredictedVertMap[i].x,hPredictedVertMap[i].y,hPredictedVertMap[i].z);
                    }
                }
            }

            glEnd();
            delete [] hPredictedVertMap;

            glPointSize(1.0f);
        }

        if (showCollisionClouds) {
            glPointSize(10);
            glColor3f(0,0,1.0f);
            glDisable(GL_LIGHTING);
            glBegin(GL_POINTS);
            for (int m=0; m<tracker.getNumModels(); ++m) {
                const float4 * collisionCloud = tracker.getCollisionCloud(m);
                for (int i=0; i<tracker.getCollisionCloudSize(m); ++i) {
                    int grid = round(collisionCloud[i].w);
                    int frame = tracker.getModel(m).getSdfFrameNumber(grid);
                    float4 v = tracker.getModel(m).getTransformModelToCamera()*
                               tracker.getModel(m).getTransformFrameToModel(frame)*
                               make_float4(make_float3(collisionCloud[i]),1.0);
                    glVertex3fv((float*)&v);
                }
            }
            glEnd();
            glEnable(GL_LIGHTING);

            glPointSize(1);
            glColor3f(1,1,1);
        }

        glPopMatrix();

        imgDisp.ActivateScissorAndClear();
        glDisable(GL_LIGHTING);
        glColor4ub(255,255,255,255);

        switch (debugImg) {
            case DebugColor:
            {
                if (depthSource->hasColor()) {
                    imgTexDepthSize.Upload(depthSource->getColor(),GL_RGB,GL_UNSIGNED_BYTE);
                    imgTexDepthSize.RenderToViewport();
                }
            }
                break;
            case DebugObsDepth:
            {

                static const float depthMin = 0.3;
                static const float depthMax = 1.0;

                const unsigned short * depth = depthSource->getDepth();

                for (int i=0; i<depthSource->getDepthWidth()*depthSource->getDepthHeight(); ++i) {
                    if (depth[i] == 0) {
                        imgDepthSize[i] = make_uchar3(128,0,0);
                    } else {
                        unsigned char g = std::max(0,std::min((int)(255*(depth[i]*depthSource->getScaleToMeters()-depthMin)/(float)(depthMax - depthMin)),255));
                        imgDepthSize[i] = make_uchar3(g,g,g);
                    }

                }

                imgTexDepthSize.Upload(imgDepthSize.hostPtr(),GL_RGB,GL_UNSIGNED_BYTE);
                imgTexDepthSize.RenderToViewport();
            }
            case DebugPredictedDepth:
            {
                static const float depthMin = 0.3;
                static const float depthMax = 1.0;

                const float4 * dPredictedVertMap = tracker.getDevicePredictedVertMap();
                static std::vector<float4> hPredictedVertMap(predWidth*predHeight);

                cudaMemcpy(hPredictedVertMap.data(),dPredictedVertMap,predWidth*predHeight*sizeof(float4),cudaMemcpyDeviceToHost);

                for (int i=0; i<predHeight*predWidth; ++i) {
                    const float depth = hPredictedVertMap[i].z;
                    if (depth == 0) {
                        imgPredSize[i] = make_uchar3(128,0,0);
                    } else {
                        unsigned char g = std::max(0,std::min((int)(255*(depth-depthMin)/(float)(depthMax - depthMin)),255));
                        imgPredSize[i] = make_uchar3(g,g,g);
                    }
                }

                imgTexPredictionSize.Upload(imgPredSize.hostPtr(),GL_RGB,GL_UNSIGNED_BYTE);
                imgTexPredictionSize.RenderToViewport();
            }
                break;
            case DebugObsToModDA:
            {
                dart::colorDataAssociationMultiModel(imgDepthSize.devicePtr(),
                                                     tracker.getDeviceDebugDataAssociationObsToMod(),
                                                     allSdfColors.devicePtr(),depthWidth,depthHeight);\
            imgDepthSize.syncDeviceToHost();
                imgTexDepthSize.Upload(imgDepthSize.hostPtr(),GL_RGB,GL_UNSIGNED_BYTE);
                imgTexDepthSize.RenderToViewport();
                break;
            }
            case DebugModToObsDA:
            {
                dart::colorDataAssociationMultiModel(imgPredSize.devicePtr(),
                                                     tracker.getDeviceDebugDataAssociationModToObs(),
                                                     allSdfColors.devicePtr(),predWidth,predHeight);\
            imgPredSize.syncDeviceToHost();
                imgTexPredictionSize.Upload(imgPredSize.hostPtr(),GL_RGB,GL_UNSIGNED_BYTE);
                imgTexPredictionSize.RenderToViewport();
                break;
            }
            case DebugObsToModErr:
            {
                static const float errMax = 0.01;
                dart::colorRampHeatMapUnsat(imgDepthSize.devicePtr(),
                                            tracker.getDeviceDebugErrorObsToMod(),
                                            depthWidth,depthHeight,
                                            0.f,errMax);
                imgDepthSize.syncDeviceToHost();
                imgTexDepthSize.Upload(imgDepthSize.hostPtr(),GL_RGB,GL_UNSIGNED_BYTE);
                imgTexDepthSize.RenderToViewport();
            }
                break;
            case DebugModToObsErr:
            {
                static const float errMax = 0.01;
                dart::colorRampHeatMapUnsat(imgPredSize.devicePtr(),
                                            tracker.getDeviceDebugErrorModToObs(),
                                            depthWidth,depthHeight,
                                            0.f,errMax);
                imgPredSize.syncDeviceToHost();
                imgTexPredictionSize.Upload(imgPredSize.hostPtr(),GL_RGB,GL_UNSIGNED_BYTE);
                imgTexPredictionSize.RenderToViewport();
            }
                break;
            case DebugJTJ:
                imgTexDepthSize.Upload(tracker.getOptimizer()->getJTJimg(),GL_RGB,GL_UNSIGNED_BYTE);
                imgTexDepthSize.RenderToViewportFlipY();
                break;
            default:
                break;
        }

        cudaError_t err = cudaGetLastError();
        if (err != cudaSuccess) {
            std::cerr << cudaGetErrorString(err) << std::endl;
        }

        pangolin::FinishFrame();

        // "main loop"????? -> next frame for demo here?
        if (pangolin::Pushed(stepVideo) || trackFromVideo || pangolinFrame == 1) {

            tracker.stepForward();

            // update table based on head movement
            {
                float4 tableNorm = make_float4(normalize(make_float3(tableNormX,tableNormY,tableNormZ)),0.f);
                float4 tablePoint = make_float4(make_float3(tableIntercept*tableNorm),1.f);
                tableNormX = tableNorm.x;
                tableNormY = tableNorm.y;
                tableNormZ = tableNorm.z;
                tableIntercept = dot(make_float3(tablePoint),make_float3(tableNorm));
            }

            static pangolin::Var<float> planeFitNormThresh("opt.planeNormThresh",0.25,-1,1);
            static pangolin::Var<float> planeFitDistThresh("opt.planeDistThresh",0.005,0.0001,0.005);

            if (fitTable) {
                float3 normal = normalize(make_float3(tableNormX,tableNormY,tableNormZ));
                float intercept = tableIntercept;
                dart::fitPlane(normal,
                               intercept,
                               tracker.getPointCloudSource().getDeviceVertMap(),
                               tracker.getPointCloudSource().getDeviceNormMap(),
                               tracker.getPointCloudSource().getDepthWidth(),
                               tracker.getPointCloudSource().getDepthHeight(),
                               planeFitDistThresh,
                               planeFitNormThresh,
                               1,
                               500);

                tableNormX = normal.x;
                tableNormY = normal.y;
                tableNormZ = normal.z;
                tableIntercept = intercept;
            }

            if (subtractTable) {
                tracker.subtractPlane(make_float3(tableNormX,tableNormY,tableNormZ),
                                      tableIntercept,0.005,-1.01);
            }

            float totalPerPointError = optimizer.getErrPerObsPoint(1,0) + optimizer.getErrPerModPoint(1,0);

        }

        /// Publish xylophone pose
        dart::SE3 T_cm = tracker.getPose(0).getTransformModelToCamera();
        geometry_msgs::Pose pose;
        geometry_msgs::Point point = geometry_msgs::Point();

        //get position values (scale in meters?)
        point.x = T_cm.r0.w;
        point.y = T_cm.r1.w;
        point.z = T_cm.r2.w;

        // todo : is this needed? does this change angles?
        T_cm.r0.w = 0;
        T_cm.r1.w = 0;
        T_cm.r2.w = 0;

        geometry_msgs::Quaternion quat = geometry_msgs::Quaternion();
        dart::se3 t_cm = dart::se3FromSE3(T_cm);
        Eigen::Quaternionf q = Eigen::AngleAxisf(t_cm.p[3], Eigen::Vector3f::UnitX())
                        * Eigen::AngleAxisf( t_cm.p[4], Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf( t_cm.p[5], Eigen::Vector3f::UnitZ());
        quat.x = q.x();
        quat.y = q.y();
        quat.z = q.z();
        quat.w = q.w();

        pose.position = point;
        pose.orientation = quat;
        xylophone_pub.publish(pose);

        // set realsense pose
        realsense_tf.setOrigin(tf::Vector3(point.x, point.y, point.z));
        tf::Quaternion quat;
        quat.setRPY(t_cm.p[3],t_cm.p[4],t_cm.p[5]);
        realsense_tf.setRotation(quat);
        tf_broadcaster.sendTransform(tf::StampedTransform(realsense_tf, ros::Time::now(), "world", "real_sense"));

    }
end:
    glDeleteBuffersARB(1,&pointCloudVbo);
    glDeleteBuffersARB(1,&pointCloudColorVbo);
    glDeleteBuffersARB(1,&pointCloudNormVbo);

    for (int m=0; m<tracker.getNumModels(); ++m) {
        for (int i=0; i<tracker.getPose(m).getReducedDimensions(); ++i) {
            delete poseVars[m][i];
        }
        delete [] poseVars[m];
    }

    for (uint i=0; i<sizeVars.size(); ++i) {
        delete sizeVars[i];
    }

    delete depthSource;

    return 0;
}
