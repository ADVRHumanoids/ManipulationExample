#include "vision_io.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


cv::Mat VisionIO::getRGBImage()
{
	return rgb_img;
}

cv::Mat VisionIO::getDepImage()
{
	return dep_img;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VisionIO::getPointCloud()
{
	return point_cloud_ptr;
}

int VisionIO::getCameraInfo(int &cam_width, int &cam_height)
{
	cam_width = camera_width;
	cam_height = camera_height;
	return 0;
}

std::string VisionIO::getHeaderFrame()
{
	return header_frame;
}

void VisionIO::rgbCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	if (cv_ptr->image.rows > 100 && cv_ptr->image.cols > 100)
		rgb_img = cv_ptr->image;
}

void VisionIO::depCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	if (cv_ptr->image.rows > 100 && cv_ptr->image.cols > 100)
		dep_img = cv_ptr->image;

}

void VisionIO::pclCb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	//std::cout << "Getting pcl " << std::endl;
	pcl::fromROSMsg(*msg, *point_cloud_ptr);
	//std::cout << "Point cloud size in CB: " << point_cloud_ptr->points.size() << std::endl;
}

void VisionIO::camInfoCb(const sensor_msgs::CameraInfoPtr& msg)
{
//	    camera_info[0] = msg->K[0]; //fx
//	    camera_info[1] = msg->K[4]; //fy
//	    camera_info[2] = msg->K[2]; //cx
//	    camera_info[3] = msg->K[5]; //cy

    camera_width = msg->width;
    camera_height = msg->height;

//	    std::cout << "... Camera info:" << std::endl;
//	    cout << "             fx:" << camera_info[0] << endl;
//	    cout << "             fy:" << camera_info[1] << endl;
//	    cout << "             cx:" << camera_info[2] << endl;
//	    cout << "             cy:" << camera_info[3] << endl;
//	    std::cout << "          width:" << camera_width << std::endl;
//	    std::cout << "         height:" << camera_height << std::endl;
}