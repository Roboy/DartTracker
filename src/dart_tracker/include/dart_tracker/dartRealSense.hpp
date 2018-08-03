#pragma once

#include <ros/ros.h>
#include <librealsense/rs.hpp>
#include "depth_sources/depth_source.h"
#include "util/mirrored_memory.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

using namespace dart;

#define NOISY       3.5		// Remove points past NOISY meters

template <typename DepthType, typename ColorType>
class DartRealSense:public DepthSource<DepthType,ColorType> {
public:
    DartRealSense<DepthType,ColorType>(int width=640, int height=480): DepthSource<DepthType,ColorType>(){
        realsense_ctx = boost::shared_ptr<rs::context>(new rs::context);
        ROS_INFO("There are %d connected RealSense devices.\n", realsense_ctx->get_device_count());
        if(realsense_ctx->get_device_count() == 0) {
            ROS_ERROR("no realsense connected");
            return;
        }else{
            realsense_dev = realsense_ctx->get_device(0);
            ROS_INFO("\nUsing device 0, an %s\n     Serial number: %s\n     Firmware version: %s\n",
                     realsense_dev->get_name(), realsense_dev->get_serial(), realsense_dev->get_firmware_version());

            this->_depthWidth = width;
            this->_depthHeight = height;

            this->_hasColor = true;
            this->_colorWidth = width;
            this->_colorHeight = height;
            _colorData = new ColorType[this->_colorWidth * this->_colorHeight];
            _adjustedColorData = new ColorType[this->_colorWidth * this->_colorHeight];


            // Configure all streams to run at VGA resolution at 60 frames per second
            realsense_dev->enable_stream(rs::stream::depth, width, height, rs::format::z16, 60);
            realsense_dev->enable_stream(rs::stream::color, width, height, rs::format::rgb8, 60);
            realsense_dev->enable_stream(rs::stream::infrared, width, height, rs::format::y8, 60);
            try { realsense_dev->enable_stream(rs::stream::infrared2, width, height, rs::format::y8, 60); }
            catch(...) { ROS_WARN("Device does not provide infrared2 stream.\n"); }

            depth_intrin     = realsense_dev->get_stream_intrinsics( rs::stream::depth );
            depth_to_color   = realsense_dev->get_extrinsics( rs::stream::depth, rs::stream::color );
            color_intrin     = realsense_dev->get_stream_intrinsics( rs::stream::color );

            this->_focalLength = make_float2(depth_intrin.fx, depth_intrin.fy);

            realsense_dev->start();

            // Determine depth value corresponding to one meter
            _scaleToMeters = realsense_dev->get_depth_scale();

#ifdef CUDA_BUILD
            _depthData = new MirroredVector<DepthType>(this->_depthWidth*this->_depthHeight);
#else
            _depthData = new DepthType[this->_depthWidth*this->_depthHeight];
#endif // CUDA_BUILD
            //fill data with initial content
            advance();
        }
    };

    ~DartRealSense(){
#ifdef CUDA_BUILD
        delete _depthData;
#else
        delete [] _depthData;
#endif // CUDA_BUILD
        delete _colorData;
        delete _adjustedColorData;
    }

#ifdef CUDA_BUILD
    const DepthType * getDepth() const { return _depthData->hostPtr(); }
    const DepthType * getDeviceDepth() const { return _depthData->devicePtr(); }
#else
    const DepthType * getDepth() const { return _depthData; }
    const DepthType * getDeviceDepth() const { return 0; }
#endif // CUDA_BUILD

    const ColorType * getColor() const { return _colorData; }

    const ColorType * getAdjustedColor() const { return _adjustedColorData; }

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

    //Generates point cloud which can be visualized in RVIZ. It scales the z values in order to view
    // needs to be called after adjustColorDataToDepth()
    int generatePointCloud( PointCloudRGB::Ptr rs_cloud_ptr ){
        // Depth dimension helpers
        int dw  = 0;
        int dh  = 0;
        int dwh = 0;

        dw = depth_intrin.width;
        dh = depth_intrin.height;

        dwh = dw * dh;

        // Set the cloud up to be used
        rs_cloud_ptr->clear( );
        rs_cloud_ptr->is_dense = false;
        rs_cloud_ptr->resize( dwh );

        // Iterate the data space
        // First, iterate across columns
        for( int dy = 0; dy < dh; dy++ )
        {

            // Second, iterate across rows
            for( int dx = 0; dx < dw; dx++ ) {
                uint i = dy * dw + dx;
                ushort depth_value = depth_frame[i];

                if (depth_value == 0)
                    continue;

                rs::float2 depth_pixel = {(float) dx, (float) dy};
                float depth_in_meters = depth_value * _scaleToMeters;

                // since depth cam and rgb cam have physical offset, the depth-coords need to be transformed to obtain the correct colour values
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);
                //Clamp since transformation sometimes exceeds boundaries & round since indices int
                const int cx = clamp((int) std::round(color_pixel.x), 0, dw - 1);
                const int cy = clamp((int) std::round(color_pixel.y), 0, dh - 1);

                static const float nan = std::numeric_limits<float>::quiet_NaN();

                // Set up logic to remove bad points
                bool depth_fail;
                bool color_fail;

                depth_fail = (depth_point.z > NOISY);
                color_fail = (cx < 0 || cy < 0 || cx > color_intrin.width || cy > color_intrin.height);

                // ==== Cloud Input Pointers ====

                // XYZ input access to cloud
                float *dp_x;
                float *dp_y;
                float *dp_z;

                dp_x = &(rs_cloud_ptr->points[i].x);
                dp_y = &(rs_cloud_ptr->points[i].y);
                dp_z = &(rs_cloud_ptr->points[i].z);

                // RGB input access to cloud
                uint8_t *cp_r;
                uint8_t *cp_g;
                uint8_t *cp_b;

                cp_r = &(rs_cloud_ptr->points[i].r);
                cp_g = &(rs_cloud_ptr->points[i].g);
                cp_b = &(rs_cloud_ptr->points[i].b);

                // ==== Cloud Input Data ====
                // Set up depth point data
                float real_x = 0;
                float real_y = 0;
                float real_z = 0;
                float adjusted_x = 0;
                float adjusted_y = 0;
                float adjusted_z = 0;

                real_x = depth_point.x;
                real_y = depth_point.y;
                real_z = depth_point.z;

                // Adjust point to coordinates
                adjusted_x = -1 * real_x;
                adjusted_y = -1 * real_y;
                adjusted_z = real_z;

                // Set up color point data
                int offset_val = (cy * dw + cx) * 3;
                const uint8_t *offset = ((const uint8_t *) color_frame + (cy * dw + cx) * 3);

                uint8_t raw_r = 0;
                uint8_t raw_g = 0;
                uint8_t raw_b = 0;
                uint8_t adjusted_r = 0;
                uint8_t adjusted_g = 0;
                uint8_t adjusted_b = 0;

                raw_r = *(offset);
                raw_g = *(offset + 1);
                raw_b = *(offset + 2);

                // Adjust color arbitrarily
                adjusted_r = raw_r;
                adjusted_g = raw_g;
                adjusted_b = raw_b;

                // ==== Cloud Point Evaluation ====
                // If bad point, remove & skip
                if (depth_fail || color_fail) {
                    *dp_x = *dp_y = *dp_z = (float) nan;
                    *cp_r = *cp_g = *cp_b = 0;
                    continue;
                }

                    // If valid point, add data to cloud
                else {
                    // Fill in cloud depth
                    *dp_x = adjusted_x;
                    *dp_y = adjusted_y;
                    *dp_z = adjusted_z;

                    // Fill in cloud color
                    *cp_r = adjusted_r;
                    *cp_g = adjusted_g;
                    *cp_b = adjusted_b;

                }
            }
        }

        return EXIT_SUCCESS;
    }

private:
    void readDepth(){
        depth_frame = reinterpret_cast<const DepthType *>(realsense_dev->get_frame_data(rs::stream::depth));
#ifdef CUDA_BUILD
        //copy and flip horizontally (mirror)
        for(int y = 0; y < this->_depthHeight; y ++){
            memcpy(_depthData->hostPtr() + y * this->_depthWidth, // start of row
                   depth_frame + this->_depthWidth * (this->_depthHeight - y -1),  // start of row from bottom to top
                   this->_depthWidth * sizeof(DepthType)); // one row
        }
#else
        //copy and flip horizontally (mirror)
        for(int y = 0; y < this->_depthHeight; y ++){
            memcpy(_depthData + y * this->_depthWidth, // dest: start of row
             depth_frame + this->_depthWidth * (this->_depthHeight - y -1),  // src: start of row from bottom to top
             this->_depthWidth* sizeof(DepthType)); // one row
        }
#endif
    };

    void readColor(){
        color_frame = reinterpret_cast<const ColorType *>(realsense_dev->get_frame_data(rs::stream::color));
        for(int y = 0; y < this->_colorHeight; y ++){
            memcpy(_colorData + y * this->_colorWidth, // dest: start of row
                   color_frame + this->_colorWidth * (this->_colorHeight - y -1),  // src: start of row from bottom to top
                   this->_colorWidth * sizeof(ColorType)); // one row
        }
        //fill adjustedColorMap
        adjustColorToDepth();
    };

    //the color values are adapted according to the current depth data (image slightly distorted since camera position differs)
    // uses data from color_frame and writes correct values into _ColorData
    void adjustColorToDepth() {
        // Depth dimension helpers
        int dw = 0;
        int dh = 0;
        int dwh = 0;

        dw = depth_intrin.width;
        dh = depth_intrin.height;

        dwh = dw * dh;

        // Iterate the data space
        // First, iterate across columns
        for (int dy = 0; dy < dh; dy++) {

            // Second, iterate across rows
            for (int dx = 0; dx < dw; dx++) {
                uint i = dy * dw + dx;
#ifdef CUDA_BUILD
                ushort depth_value = _depthData->hostPtr()[i];
#else
                ushort depth_value = _depthData[i];
#endif
                if (depth_value == 0)
                    continue;

                rs::float2 depth_pixel = {(float) dx, (float) dy};
                float depth_in_meters = depth_value * _scaleToMeters;

                // since depth cam and rgb cam have physical offset, the depth-coords need to be transformed to obtain the correct colour values
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);
                //Clamp since transformation sometimes exceeds boundaries & round since indices int
                const int cx = clamp((int) std::round(color_pixel.x), 0, dw - 1);
                const int cy = clamp((int) std::round(color_pixel.y), 0, dh - 1);

                bool depth_fail = (depth_point.z > NOISY);
                bool color_fail = (cx < 0 || cy < 0 || cx > color_intrin.width || cy > color_intrin.height);

                // ==== Input Pointers ====

                // RGB destination ptr
                uint8_t *cp_r = ((uint8_t *) _adjustedColorData) + (dy * dw + dx) * 3;
                uint8_t *cp_g = ((uint8_t *) _adjustedColorData) + (dy * dw + dx) * 3 + 1;
                uint8_t *cp_b = ((uint8_t *) _adjustedColorData) + (dy * dw + dx) * 3 + 2;

                // ==== Color Point Data ====
                const uint8_t *offset = ((const uint8_t *) _colorData + (cy * dw + cx) * 3);

                uint8_t adjusted_r = *(offset);
                uint8_t adjusted_g = *(offset + 1);
                uint8_t adjusted_b = *(offset + 2);

                // ==== Cloud Point Evaluation ====
                // If bad point, remove & skip
                if (depth_fail || color_fail) {
                    *cp_r = *cp_g = *cp_b = 0;
                    continue;
                }
                    // If valid point, add data to cloud
                else {

                    // Fill in cloud color
                    *cp_r = adjusted_r;
                    *cp_g = adjusted_g;
                    *cp_b = adjusted_b;
/*
                    // Depth dimension helpers
        int dw  = 0;
        int dh  = 0;
        int dwh = 0;

        dw = depth_intrin.width;
        dh = depth_intrin.height;

        dwh = dw * dh;

        // Iterate over the depth data
        for( int dy = 0; dy < dh; dy++ )
        {
            for( int dx = 0; dx < dw; dx++ ) {
                uint i = dy * dw + dx;
                ushort depth_value = depth_frame[i];

                if (depth_value == 0)
                    continue;

                rs::float2 depth_pixel = {(float) dx, (float) dy};
                float depth_in_meters = depth_value * _scaleToMeters;

                // since depth cam and rgb cam have physical offset, the depth-coords need to be transformed to obtain the correct colour values
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);
                //Clamp since transformation sometimes exceeds boundaries & round since indices int
                const int cx = clamp((int) std::round(color_pixel.x), 0, dw - 1);
                const int cy = clamp((int) std::round(color_pixel.y), 0, dh - 1);

                // Set up logic to remove bad points
                bool color_fail = (cx < 0 || cy < 0 || cx > color_intrin.width || cy > color_intrin.height);


                // ==== Pointer Setup ====
                //get shifted pixel from color_frame. also: flip image!!!
                const uint8_t *color = ((const uint8_t *) color_frame + ((dh - cy -1) * dw + cx) * 3);

                //set color at original position
                uint8_t *cp_r;
                uint8_t *cp_g;
                uint8_t *cp_b;

                cp_r = (uint8_t *) _colorData + (dy * dw + cx) * 3;
                cp_g = (uint8_t *) _colorData + (dy * dw + cx) * 3 + 1;
                cp_b = (uint8_t *) _colorData + (dy * dw + cx) * 3 + 2;

                // ==== Fill ColorData ====
                *cp_r = *(color);
                *cp_g = *(color+1);
                *cp_b = *(color+2);
            }
        }*/
                }
            }
        }
    }


#ifdef CUDA_BUILD
    MirroredVector<DepthType> * _depthData;
#else
    DepthType * _depthData;
#endif // CUDA_BUILD
    ColorType * _colorData;
    const ColorType *_adjustedColorData;
    std::string _depthFormat;
    std::string _colorFormat;
    std::vector<ulong> _depthTimes;
    std::vector<ulong> _colorTimes;
    float _scaleToMeters;
    boost::shared_ptr<rs::context> realsense_ctx;
    rs::device * realsense_dev;
public:
    rs::intrinsics depth_intrin;
    rs::extrinsics depth_to_color;
    rs::intrinsics color_intrin;
    const ColorType *color_frame;
    const DepthType *depth_frame;
};