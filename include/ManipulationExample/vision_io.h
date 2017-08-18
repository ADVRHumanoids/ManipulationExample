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

class VisionIO
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber rgb_sub;
	image_transport::Subscriber dep_sub;
	ros::Subscriber pcl_sub;
	ros::Subscriber cam_sub;

	//image_transport::Publisher image_pub_;

	int camera_id;  // 0 for asus; 1 for multisense

	cv::Mat rgb_img;
	cv::Mat dep_img;
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr;
	std::string header_frame;

	int camera_width, camera_height;



public:
	VisionIO(int cid = 0):	it(nh),
							point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>),  // remember to init point_cloud_ptr for boost
							camera_id (cid)
	{
		if (camera_id == 0) // using asus
		{
			rgb_sub = it.subscribe("/camera/rgb/image_rect_color", 1, &VisionIO::rgbCb, this);
			dep_sub = it.subscribe("/camera/depth/image", 1, &VisionIO::depCb, this);
			pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, &VisionIO::pclCb, this);
			cam_sub = nh.subscribe("/camera/rgb/camera_info", 1, &VisionIO::camInfoCb, this);

			header_frame = "/camera_rgb_optical_frame";
		}
		else  				// using multisense
		{
			rgb_sub = it.subscribe("/multisense/left/image_color", 1, &VisionIO::rgbCb, this);
			dep_sub = it.subscribe("/multisense/depth", 1, &VisionIO::depCb, this);
			pcl_sub = nh.subscribe("/multisense/organized_image_points2", 1, &VisionIO::pclCb, this);
			cam_sub = nh.subscribe("/multisense/left/camera_info", 1, &VisionIO::camInfoCb, this);

			header_frame = "/multisense/left_camera_optical_frame";
		}
	}

	~VisionIO()
	{
	}

	cv::Mat getRGBImage()
	{
		return rgb_img;
	}
	cv::Mat getDepImage()
	{
		return dep_img;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud()
	{
		return point_cloud_ptr;
	}

	int getCameraInfo(int &cam_width, int &cam_height)
	{
		cam_width = camera_width;
		cam_height = camera_height;
		return 0;
	}

	std::string getHeaderFrame()
	{
		return header_frame;
	}

	void rgbCb(const sensor_msgs::ImageConstPtr& msg)
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

	void depCb(const sensor_msgs::ImageConstPtr& msg)
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

	void pclCb(const sensor_msgs::PointCloud2ConstPtr& msg)
	{
		//std::cout << "Getting pcl " << std::endl;
		pcl::fromROSMsg(*msg, *point_cloud_ptr);
		//std::cout << "Point cloud size in CB: " << point_cloud_ptr->points.size() << std::endl;
	}

	void camInfoCb(const sensor_msgs::CameraInfoPtr& msg)
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

};

