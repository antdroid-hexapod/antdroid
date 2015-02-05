/* antdroid_cam_control.cpp: node to control antdroid with camera stimulus
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

#include <ros/ros.h>
#include "../include/antdroid_cam_control/antdroid_cam_control.hpp"
 
 
ImageConverter::ImageConverter(): _it(_nh),
                                _block_green(0),
                                _count(0),
                                _is_test(0)
{
    _image_sub = _it.subscribe("/camera/image", 1, 
    &ImageConverter::imageCb, this);
    _vel_pub = _nh.advertise<geometry_msgs::Twist>("/control_interpreter/cmd_vel", 1, true);

    _nh.getParam("/cam_control/test", _is_test);
    if(_is_test)
    {
        namedWindow(GREEN_HSV);
        namedWindow(GREEN_BALL);

        namedWindow(RED_HSV);
        namedWindow(RED_BALL);
    }
}

ImageConverter::~ImageConverter()
{
    if(_is_test)
    {
        destroyWindow(GREEN_HSV);
        destroyWindow(GREEN_BALL);

        destroyWindow(RED_HSV);
        destroyWindow(RED_BALL);
    }
}
 
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
  

    assert(cv_ptr->image.type() == CV_8UC3);
    cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2HSV);

    redFilter(cv_ptr->image);

    if(!_block_green)
        greenFilter(cv_ptr->image);

    waitKey(200);
}

   
Mat ImageConverter::redFilter(Mat src)
{
    Scalar HSV_RED_MIN(168,120,90);
    Scalar HSV_RED_MAX(179,230,230);

    inRange(src, HSV_RED_MIN, HSV_RED_MAX, src);

    if(_is_test)
        imshow(RED_HSV, src);

    src = refineImage(src);
    src = trackBall(src, RED);

    if(_is_test)
        imshow(RED_BALL, src);
    
    return src;
}

Mat ImageConverter::greenFilter(Mat src)
{
    Scalar HSV_GREEN_MIN(35,60,60);
    Scalar HSV_GREEN_MAX(70,230,230);

    inRange(src, HSV_GREEN_MIN, HSV_GREEN_MAX, src);

    if(_is_test)    
        imshow(GREEN_HSV, src);

    src = refineImage(src);
    src = trackBall(src, GREEN);

    if(_is_test)
        imshow(GREEN_BALL, src);
    return src;
}

Mat ImageConverter::refineImage(Mat src)
{

    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
    //erode(src, src, element);
    //imshow("erosion", src);
    dilate(src, src, element);
    //imshow("dilatacion", src);
    GaussianBlur( src, src, Size(9, 9), 2, 2 );

    return src;
}

Mat ImageConverter::trackBall(Mat src, const bool is_red)
{
    std::vector<Vec3f> circles;
 
    HoughCircles(src, circles, CV_HOUGH_GRADIENT, 1, 10, 100, 20, 1, 400);

    if(circles.size() > 0)
    {
        Vec3i c = circles[0];
        circle( src, Point(c[0], c[1]), c[2], Scalar(175,255,255), 3, CV_AA);
        circle( src, Point(c[0], c[1]), 2, Scalar(175,255,255), 3, CV_AA);
        
        if(_is_test)
        {
            ROS_INFO_STREAM("x:" << c[0]);
            ROS_INFO_STREAM("y:" << c[1]);
            ROS_INFO_STREAM("R:" << c[2]);
        }

        geometry_msgs::Twist vel;
        if(c[2] < NEAR_BALL )
        {
            if(c[0] < LEFT_WIDTH_ROTATE)
                vel.angular.z = 1;
            else if(c[0] > RIGHT_WIDTH_ROTATE)
                vel.angular.z = -1;
            else if (is_red)
                vel.linear.x = -1;
            else
                vel.linear.x = 1;
        }
        else
        {
            //if(is_red)
                //attack
            //else
                //balance_down
        }

        if(vel.linear.x != 0 || vel.angular.z != 0)
            _vel_pub.publish(vel);

        if(is_red)
            _block_green = 1;
    }

    else 
        _count += 1;
        if(_count > 10)
        {
            _block_green = 0;
            _count = 0;           
        }



    return src;
}

