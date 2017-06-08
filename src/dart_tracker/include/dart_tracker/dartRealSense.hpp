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
                this->_hasColor = true;

                // Configure all streams to run at VGA resolution at 60 frames per second
                realsense_dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
                realsense_dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
                realsense_dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 60);
                try { realsense_dev->enable_stream(rs::stream::infrared2, 640, 480, rs::format::y8, 60); }
                catch(...) { ROS_WARN("Device does not provide infrared2 stream.\n"); }

                depth_intrin     = realsense_dev->get_stream_intrinsics( rs::stream::depth );
                depth_to_color   = realsense_dev->get_extrinsics( rs::stream::depth, rs::stream::color );
                color_intrin     = realsense_dev->get_stream_intrinsics( rs::stream::color );

                this->_focalLength = make_float2(depth_intrin.fx, depth_intrin.fy);

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
                for( int dx = 0; dx < dw; dx++ )
                {
                    uint i = dy * dw + dx;
                    uint16_t depth_value = depth_frame[ i ];

                    if( depth_value == 0 )
                        continue;

                    rs::float2 depth_pixel = { (float)dx, (float)dy };
                    float depth_in_meters = depth_value /_scaleToMeters;

                    rs::float3 depth_point = depth_intrin.deproject( depth_pixel, depth_in_meters );
                    rs::float3 color_point = depth_to_color.transform(depth_point);
                    rs::float2 color_pixel = color_intrin.project(color_point);

                    const int cx = ( int )std::round( color_pixel.x );
                    const int cy = ( int )std::round( color_pixel.y );

                    static const float nan = std::numeric_limits<float>::quiet_NaN( );

                    // Set up logic to remove bad points
                    bool depth_fail;
                    bool color_fail;

                    depth_fail = ( depth_point.z > NOISY );
                    color_fail = ( cx < 0 || cy < 0 || cx > color_intrin.width || cy > color_intrin.height );

                    // ==== Cloud Input Pointers ====

                    // XYZ input access to cloud
                    float *dp_x;
                    float *dp_y;
                    float *dp_z;

                    dp_x = &( rs_cloud_ptr->points[ i ].x );
                    dp_y = &( rs_cloud_ptr->points[ i ].y );
                    dp_z = &( rs_cloud_ptr->points[ i ].z );

                    // RGB input access to cloud
                    uint8_t *cp_r;
                    uint8_t *cp_g;
                    uint8_t *cp_b;

                    cp_r = &( rs_cloud_ptr->points[ i ].r );
                    cp_g = &( rs_cloud_ptr->points[ i ].g );
                    cp_b = &( rs_cloud_ptr->points[ i ].b );

                    // ==== Cloud Input Data ====
                    // Set up depth point data
                    float real_x        = 0;
                    float real_y        = 0;
                    float real_z        = 0;
                    float adjusted_x    = 0;
                    float adjusted_y    = 0;
                    float adjusted_z    = 0;

                    real_x = depth_point.x;
                    real_y = depth_point.y;
                    real_z = depth_point.z;

                    // Adjust point to coordinates
                    adjusted_x = -1 * real_x;
                    adjusted_y = -1 * real_y;
                    adjusted_z = real_z;

                    // Set up color point data
                    const uint8_t *offset = ( (const uint8_t*)color_frame + ( cy * color_intrin.width + cx ) * 3 );

                    uint8_t raw_r       = 0;
                    uint8_t raw_g       = 0;
                    uint8_t raw_b       = 0;
                    uint8_t adjusted_r  = 0;
                    uint8_t adjusted_g  = 0;
                    uint8_t adjusted_b  = 0;

                    raw_r = *( offset );
                    raw_g = *( offset + 1 );
                    raw_b = *( offset + 2 );

                    // Adjust color arbitrarily
                    adjusted_r = raw_r;
                    adjusted_g = raw_g;
                    adjusted_b = raw_b;

                    // ==== Cloud Point Evaluation ====
                    // If bad point, remove & skip
                    if( depth_fail || color_fail )
                    {
                        *dp_x = *dp_y = *dp_z = (float) nan;
                        *cp_r = *cp_g = *cp_b = 0;
                        continue;
                    }

                        // If valid point, add data to cloud
                    else
                    {
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
public:
    rs::intrinsics depth_intrin;
    rs::extrinsics depth_to_color;
    rs::intrinsics color_intrin;
    const ColorType *color_frame;
    const DepthType *depth_frame;
};