#include "realsenseImagetransport.hpp"

realsenseImagetransport::realsenseImagetransport(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
	image_transport::ImageTransport _it(nh_);
	image_sub_ = _it.subscribe("camera/color/image_raw", 1, &realsenseImagetransport::imageCb, this);
}

realsenseImagetransport::~realsenseImagetransport(){
}

void realsenseImagetransport::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

cv_bridge::CvImageConstPtr realsenseImagetransport::getpicture()
{
	return cv_ptr;
}
