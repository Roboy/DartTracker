#pragma once

#include <ros/ros.h>
#include <librealsense/rs.hpp>
#include "depth_sources/depth_source.h"
#include "util/mirrored_memory.h"

using namespace dart;

template <typename DepthType, typename ColorType>
class DartRealSense:public DepthSource<DepthType,ColorType> {
public:
    DartRealSense<DepthType,ColorType>(): DepthSource<DepthType,ColorType>(){
            realsense_ctx = boost::shared_ptr<rs::context>(new rs::context);
            ROS_INFO("There are %d connected RealSense devices.\n", realsense_ctx->get_device_count());
            if(realsense_ctx->get_device_count() == 0) {
                ROS_ERROR("no realsense connected");
                return;
            }else{
                realsense_dev = realsense_ctx->get_device(0);
                ROS_INFO("\nUsing device 0, an %s\n     Serial number: %s\n     Firmware version: %s\n",
                         realsense_dev->get_name(), realsense_dev->get_serial(), realsense_dev->get_firmware_version());

                this->_depthWidth = 640;
                this->_depthHeight = 480;

                // Configure all streams to run at VGA resolution at 60 frames per second
                realsense_dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
                realsense_dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
                realsense_dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 60);
                try { realsense_dev->enable_stream(rs::stream::infrared2, 640, 480, rs::format::y8, 60); }
                catch(...) { ROS_WARN("Device does not provide infrared2 stream.\n"); }
                realsense_dev->start();

                // Determine depth value corresponding to one meter
                _scaleToMeters = 1.0f / realsense_dev->get_depth_scale();

                // allocate data
#ifdef CUDA_BUILD
                _depthData = new MirroredVector<DepthType>(this->_depthWidth*this->_depthHeight);
#else
                _depthData = new DepthType[this->_depthWidth*this->_depthHeight];
#endif // CUDA_BUILD
            }
    };

    ~DartRealSense(){
#ifdef CUDA_BUILD
        delete _depthData;
#else
        delete [] _depthData;
#endif // CUDA_BUILD
    }

#ifdef CUDA_BUILD
    const DepthType * getDepth() const { return _depthData->hostPtr(); }
    const DepthType * getDeviceDepth() const { return _depthData->devicePtr(); }
#else
    const DepthType * getDepth() const { return _depthData; }
    const DepthType * getDeviceDepth() const { return 0; }
#endif // CUDA_BUILD

    const ColorType * getColor() const { return _colorData; }

    ColorLayout getColorLayout() const { return LAYOUT_RGB; }

    uint64_t getDepthTime() const { return _depthTimes[this->_frame]; }

    uint64_t getColorTime() const { return _colorTimes[this->_frame]; }

    void setFrame(const uint frame){
        this->_frame = frame;

        readDepth();
        if (this->_hasColor) {
            readColor();
        }
    };

    void advance(){
        realsense_dev->wait_for_frames();
        readDepth();
#ifdef CUDA_BUILD
        _depthData->syncHostToDevice();
#endif // CUDA_BUILD

        if (this->_hasColor) {
            readColor();
        }
    }

    bool hasRadialDistortionParams() const { return false; }


    float getScaleToMeters() const { return _scaleToMeters; }

private:
    void readDepth(){
        depth_frame = reinterpret_cast<const DepthType *>(realsense_dev->get_frame_data(rs::stream::depth));
    };

    void readColor(){
        color_frame = reinterpret_cast<const ColorType *>(realsense_dev->get_frame_data(rs::stream::color));
    };

#ifdef CUDA_BUILD
    MirroredVector<DepthType> * _depthData;
#else
    DepthType * _depthData;
#endif // CUDA_BUILD
    ColorType * _colorData;
    std::string _depthFormat;
    std::string _colorFormat;
    std::vector<ulong> _depthTimes;
    std::vector<ulong> _colorTimes;
    float _scaleToMeters;
    boost::shared_ptr<rs::context> realsense_ctx;
    rs::device * realsense_dev;
    const DepthType *depth_frame;
    const ColorType *color_frame;
};