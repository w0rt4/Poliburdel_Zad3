#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "mavrosCommand.hpp"

class realsenseImagetransport
{
	public:
		realsenseImagetransport(ros::NodeHandle* nodehandle);
		virtual ~realsenseImagetransport();
		cv_bridge::CvImageConstPtr getpicture();
		
		
	private:
		void imageCb(const sensor_msgs::ImageConstPtr& msg);
		image_transport::ImageTransport _it();
		image_transport::Subscriber image_sub_;
		ros::NodeHandle nh_;
		cv_bridge::CvImageConstPtr cv_ptr;
};
