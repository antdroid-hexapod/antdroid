/* antdroid_cam_control.hpp: header of node to control antdroid with visual stimulus
 *
 * Copyright (C) 2015 Alexander Gil and Javier Román
 *
 * This library is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


#ifndef ANTDROID_CAM_CONTROL_HPP_
#define ANTDROID_CAM_CONTROL_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <geometry_msgs/Twist.h>

static const std::string GREEN_HSV= "Green HSV Filter";
static const std::string GREEN_BALL= "Green Ball Detection";
static const std::string RED_HSV= "Red HSV Filter";
static const std::string RED_BALL= "Red Ball Detection";



static const bool RED = 1;
static const bool GREEN = 0;

static const int NEAR_BALL = 60; // radius of ball when is too near

static const int LEFT_WIDTH_ROTATE = 140;
static const int RIGHT_WIDTH_ROTATE = 500;


using namespace cv;


 
class ImageConverter
{
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::Subscriber _image_sub;
//    image_transport::Publisher image_pub_;
    ros::Publisher _vel_pub;
    bool _block_green;
    bool _is_test;

    int _count;

    public:
    ImageConverter();
 
    ~ImageConverter();
 
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    Mat redFilter(Mat src);
    Mat greenFilter(Mat src);
    Mat refineImage(Mat src);
    Mat trackBall(Mat src, const bool is_red);
};

#endif