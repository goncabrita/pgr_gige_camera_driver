/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// ROS communication
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h> // The file may be YAML or INI format
#include <std_msgs/String.h>
#include <polled_camera/publication_server.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include "pgr_gige_camera_driver/PGRGigECameraConfig.h"

// Standard libs
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <sstream>
#include <fstream>
#include <sys/stat.h>

#include "flycapture/FlyCapture2.h"

// The following macros makes the source easier to read as it's not 66% error handling!
#define PGRERROR_OK FlyCapture2::PGRERROR_OK
#define PRINT_FATAL_AND_BREAK {ROS_FATAL("PointGrey GigE Node - %s - %s", __FUNCTION__, error.GetDescription()); ROS_BREAK();}
#define PRINT_ERROR_AND_RETURN_FALSE {ROS_ERROR("PointGrey GigE Node - %s - %s", __FUNCTION__, error.GetDescription()); return false;}
#define PRINT_ERROR {ROS_ERROR("PointGrey GigE Node - %s - %s", __FUNCTION__, error.GetDescription());}

void frameDone(FlyCapture2::Image * frame, const void *pCallbackData);

class PGRCameraNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    image_transport::ImageTransport it_;
    image_transport::CameraPublisher pub_;

    // Camera
    FlyCapture2::GigECamera cam_;
    bool running_;
    int width_;
    int height_;

    // ROS messages
    sensor_msgs::Image img_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_mgr_;
    sensor_msgs::CameraInfoPtr cam_info_ptr_;

    std::string frame_id_;

    FlyCapture2::IPAddress ip_address_;
    int serial_number_;

public:
    PGRCameraNode(const ros::NodeHandle & node_handle, ros::NodeHandle param_nh):nh_(node_handle), nh_private_(param_nh), it_(param_nh), running_(false)
    {
        nh_private_.param<std::string>("frame_id", frame_id_, "pgr_camera");

        std::string camera_name;
        nh_private_.param<std::string>("camera_name", camera_name, "pgr_camera");
        std::string calibration_file;
        nh_private_.param<std::string>("frame_id", frame_id_, "pgr_camera");
        loadIntrinsics(calibration_file.c_str(), camera_name.c_str());

        bool external_trigger;
        nh_private_.param("external_trigger", external_trigger, false);
        int trigger_source;
        nh_private_.param("trigger_source", trigger_source, 0);
        bool fire_trigger;
        nh_private_.param("fire_trigger", fire_trigger, false);
        double trigger_frequency;
        nh_private_.param("trigger_frequency", trigger_frequency, 15.0);

        if(fire_trigger && !external_trigger)
        {
            ROS_FATAL("PointGrey GigE Node - %s - Fire trigger set to true but external triggering is not!", __FUNCTION__);
            ROS_BREAK();
        }

        std::string ip_address;
        bool got_ip_address = nh_private_.getParam("ip_address", ip_address);
        bool got_serial = nh_private_.getParam("serial_number", serial_number_);

        FlyCapture2::Error error;
        FlyCapture2::BusManager bus_manager;
        FlyCapture2::PGRGuid guid;

        if(!got_ip_address && !got_serial)
        {
            ROS_INFO("PointGrey GigE Node - %s - No camera serial number or IP address provided, looking for cameras...", __FUNCTION__);

            FlyCapture2::CameraInfo cam_info[128];
            unsigned int num_cam_info = 128;

            if((error = bus_manager.DiscoverGigECameras(cam_info, &num_cam_info)) != PGRERROR_OK)
                PRINT_FATAL_AND_BREAK;

            ROS_INFO("PointGrey GigE Node - %s - Found %u cameras! Using first camera.", __FUNCTION__, num_cam_info);

            if((error = bus_manager.GetCameraFromIndex(0, &guid)) != PGRERROR_OK)
                PRINT_FATAL_AND_BREAK;
        }
        else if(got_ip_address)
        {
            sscanf(ip_address.c_str(), "%u.%u.%u.%u", (unsigned int*)ip_address_.octets, (unsigned int*)&ip_address_.octets[1], (unsigned int*)&ip_address_.octets[2], (unsigned int*)&ip_address_.octets[3]);
            ROS_INFO("PointGrey GigE Node - %s - Getting camera on %u.%u.%u.%u", __FUNCTION__, ip_address_.octets[0], ip_address_.octets[1], ip_address_.octets[2], ip_address_.octets[3]);
            if((error = bus_manager.GetCameraFromIPAddress(ip_address_, &guid)) != PGRERROR_OK)
                PRINT_FATAL_AND_BREAK;
        }
        else
        {
            ROS_INFO("PointGrey GigE Node - %s - Getting camera with serial %u", __FUNCTION__, serial_number_);
            if((error = bus_manager.GetCameraFromSerialNumber(serial_number_, &guid)) != PGRERROR_OK)
                PRINT_FATAL_AND_BREAK;
        }

        ROS_INFO("PointGrey GigE Node - %s - Connecting to camera...", __FUNCTION__);

        if((error = cam_.Connect(&guid)) != PGRERROR_OK)
            PRINT_FATAL_AND_BREAK;

        ROS_DEBUG("PointGrey GigE Node - %s - Getting camera info...", __FUNCTION__);
        FlyCapture2::CameraInfo camera_info;
        if((error = cam_.GetCameraInfo(&camera_info)) != PGRERROR_OK)
            PRINT_ERROR;
        // Show information?

        ROS_DEBUG("PointGrey GigE Node - %s - Getting image info...", __FUNCTION__);
        FlyCapture2::GigEImageSettingsInfo image_info;
        if((error = cam_.GetGigEImageSettingsInfo(&image_info)) != PGRERROR_OK)
            PRINT_FATAL_AND_BREAK;

        FlyCapture2::GigEImageSettings image_settings;
        image_settings.offsetX = 0;
        image_settings.offsetY = 0;
        image_settings.height = image_info.maxHeight;
        image_settings.width = image_info.maxWidth;
        image_settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;
        if((error = cam_.SetGigEImageSettings(&image_settings)) != PGRERROR_OK)
            PRINT_FATAL_AND_BREAK;

        height_ = image_info.maxHeight;
        width_ = image_info.maxWidth;

        FlyCapture2::TriggerModeInfo trigger_info;
        FlyCapture2::TriggerMode trigger_mode;
        if(external_trigger)
        {
            if((error = cam_.GetTriggerModeInfo(&trigger_info)) != PGRERROR_OK)
                PRINT_FATAL_AND_BREAK;

            if(!trigger_info.present)
            {
                ROS_FATAL("PointGrey GigE Node - %s - Trigger is set but is not supported by the camera!", __FUNCTION__);
                ROS_BREAK();
            }

            if((error = cam_.GetTriggerMode(&trigger_mode)) != PGRERROR_OK)
                PRINT_FATAL_AND_BREAK;

            trigger_mode.onOff = true;
            trigger_mode.mode = 0;
            trigger_mode.parameter = 0;
            trigger_mode.source = trigger_source;

            if((error = cam_.SetTriggerMode(&trigger_mode)) != PGRERROR_OK)
                PRINT_FATAL_AND_BREAK;

            if(fire_trigger)
            {
                unsigned int data;

                // Change pin 2 to PWM
                data = 0x80040000;
                if((error = cam_.WriteRegister(0x1130, data)) != PGRERROR_OK)
                    PRINT_FATAL_AND_BREAK;

                unsigned int high_ticks = (1024000.0/trigger_frequency);
                unsigned int low_ticks = 64;
                if(high_ticks > 65535)
                {
                    low_ticks = high_ticks - 65535;
                    high_ticks = 65535;
                    if(low_ticks < 64)
                    {
                        high_ticks -= 64 - low_ticks;
                        low_ticks = 64;
                    }
                }
                else
                {
                    high_ticks -= low_ticks;
                }
                if(low_ticks > 65535 || high_ticks < 64)
                {
                    ROS_FATAL("PointGrey GigE Node - %s - Unable to run the trigger at the desired frequency!", __FUNCTION__);
                    ROS_BREAK();
                }

                data = (low_ticks << 16) + high_ticks;
                if((error = cam_.WriteRegister(0x1134, data)) != PGRERROR_OK)
                    PRINT_FATAL_AND_BREAK;

                ROS_INFO("PointGrey GigE Node - %s - 0x%X", __FUNCTION__, data);

                // Activate the PWM and set the cycles to infinite
                data = 0x8004FF00;
                if((error = cam_.WriteRegister(0x1130, data)) != PGRERROR_OK)
                    PRINT_FATAL_AND_BREAK;

                /*if((error = cam_.ReadRegister(0x1130, &data)) != PGRERROR_OK)
                    PRINT_FATAL_AND_BREAK;
                ROS_INFO("GPIO_CTRL_PIN_2 0x%X", data);
                if((error = cam_.ReadRegister(0x1134, &data)) != PGRERROR_OK)
                    PRINT_FATAL_AND_BREAK;
                ROS_INFO("GPIO_XTRA_PIN_2 0x%X", data);*/
            }
        }

        pub_ = it_.advertiseCamera("image", 1);

        cam_.SetCallback(&frameDone, (void*)this);

        ROS_INFO("PointGrey GigE Node - %s - Done!", __FUNCTION__);
    }

    void frameCallback(FlyCapture2::Image * frame)
    {
        cam_info_ptr_ = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(cam_info_mgr_->getCameraInfo()));

        if(processFrame(frame, img_, cam_info_ptr_))
            pub_.publish(img_, *cam_info_ptr_);
    }

    void reconfigure(pgr_gige_camera_driver::PGRGigECameraConfig & config, uint32_t level)
    {
        ROS_INFO("PointGrey GigE Node - %s - Reconfigure request received.", __FUNCTION__);

        if(level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
            stop();

        if(level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
            start();
    }

    ~PGRCameraNode()
    {
        stop();
        cam_.Disconnect();
    }

    void start()
    {
        if(running_)
            return;

        ROS_DEBUG("PointGrey GigE Node - %s", __FUNCTION__);
        cam_.StartCapture();

        running_ = true;
    }

    void stop()
    {
        if(!running_)
            return;

        ROS_DEBUG("PointGrey GigE Node - %s", __FUNCTION__);
        cam_.StopCapture();

        running_ = false;
    }

    static bool frameToImage(FlyCapture2::Image * frame, sensor_msgs::Image & image)
    {
        std::string encoding;

        // Get the raw image dimensions
        FlyCapture2::PixelFormat pixel_format;
        uint32_t rows, cols, stride;

        frame->GetDimensions(&rows, &cols, &stride, &pixel_format);

        FlyCapture2::BayerTileFormat bayer_format;
        bayer_format = frame->GetBayerTileFormat();

        if(bayer_format == FlyCapture2::NONE)
        {
            encoding = sensor_msgs::image_encodings::MONO8;
            return sensor_msgs::fillImage(image, encoding, rows, cols, stride, frame->GetData());
        }
        else
        {
            // Create a converted image
            FlyCapture2::Image converted_image;
            FlyCapture2::Error error;

            // TODO: Handle encoding directly from Bayer
            switch((FlyCapture2::PixelFormat)pixel_format)
            {
                case FlyCapture2::PIXEL_FORMAT_RAW8:

                    if((error = frame->Convert(FlyCapture2::PIXEL_FORMAT_RGB, &converted_image)) != PGRERROR_OK)
                        PRINT_ERROR_AND_RETURN_FALSE;

                    encoding = sensor_msgs::image_encodings::RGB8;
                    break;

                default:
                    return false;
                    break;
            }

            converted_image.GetDimensions(&rows, &cols, &stride, &pixel_format);
            return sensor_msgs::fillImage(image, encoding, rows, cols, stride, converted_image.GetData());
        }
    }


    bool processFrame(FlyCapture2::Image * frame, sensor_msgs::Image & img, sensor_msgs::CameraInfoPtr & cam_info)
    {
        if(!frameToImage(frame, img))
            return false;

        FlyCapture2::TimeStamp time_stamp = frame->GetTimeStamp();
        ros::Time camera_time(time_stamp.seconds + time_stamp.microSeconds/1000.0);
        img.header.stamp = cam_info->header.stamp = ros::Time::now();//camera_time + stamp_delta_;
        img.header.frame_id = frame_id_;

        // Throw out any CamInfo that's not calibrated to this camera mode
        if(cam_info->K[0] != 0.0 && (img.width != cam_info->width || img.height != cam_info->height))
        {
            cam_info.reset(new sensor_msgs::CameraInfo());
        }

        // If we don't have a calibration, set the image dimensions
        if(cam_info->K[0] == 0.0)
        {
            cam_info->width = width_ = img.width;
            cam_info->height = height_ = img.height;
        }

        return true;
    }

    void loadIntrinsics(std::string url_calib_file, std::string camera_name)
    {
        // Camera Information and data
        //string url_calib_file = "file://" + inifile;
        cam_info_mgr_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nh_private_, camera_name, url_calib_file));

        if(cam_info_mgr_->isCalibrated())
        {
            ROS_INFO("PointGrey GigE Node - %s - Camera has loaded calibration file '%s'", __FUNCTION__, url_calib_file.c_str());
        }
        else
        {
            ROS_WARN ("PointGrey GigE Node - %s - Camera is not calibrated!", __FUNCTION__);
        }
    }

};

// FIXME: How to make this a member function?
void frameDone(FlyCapture2::Image * frame, const void *pCallbackData)
{
    PGRCameraNode * node = (PGRCameraNode*)pCallbackData;
    node->frameCallback(frame);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pgr_gige_camera_node");

    typedef dynamic_reconfigure::Server<pgr_gige_camera_driver::PGRGigECameraConfig> Server;
    Server server;

    try
    {
        boost::shared_ptr<PGRCameraNode> camera(new PGRCameraNode(ros::NodeHandle(), ros::NodeHandle("~")));

        Server::CallbackType cb = boost::bind(&PGRCameraNode::reconfigure, camera, _1, _2);
        server.setCallback(cb);

        ros::spin ();

    }
    catch(std::runtime_error & e)
    {
        ROS_FATAL("Uncaught exception: '%s', aborting.", e.what ());
        ROS_BREAK();
    }

    return 0;

}
