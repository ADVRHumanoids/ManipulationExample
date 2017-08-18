// NOT USE THIS CLASS


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
private:
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
	    if (false) // using asus
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

	virtual cv::Mat getRGBImage();
	virtual cv::Mat getDepImage();
	virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud();
	virtual int getCameraInfo(int &cam_width, int &cam_height);
	virtual std::string getHeaderFrame();
	virtual void rgbCb(const sensor_msgs::ImageConstPtr& msg);
	virtual void depCb(const sensor_msgs::ImageConstPtr& msg);
	virtual void pclCb(const sensor_msgs::PointCloud2ConstPtr& msg);
	virtual void camInfoCb(const sensor_msgs::CameraInfoPtr& msg);
	

};

