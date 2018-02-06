/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <ManipulationExample_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <iostream>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


// //OpenCV
// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// 
// //PCL
// #include <pcl/features/feature.h>
// #include <pcl/common/centroid.h>
// #include <pcl/common/time.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <boost/thread/thread.hpp>
// #include <pcl/common/common_headers.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// 
// //cv_bridge
// #include <cv_bridge/cv_bridge.h>


/* Specify that the class XBotPlugin::ManipulationExample is a XBot RT plugin with name "ManipulationExample" */
REGISTER_XBOT_PLUGIN(ManipulationExample, XBotPlugin::ManipulationExample)

namespace XBotPlugin {


// // Add constructor    
// ManipulationExample::ManipulationExample(): point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>)
// {
// 
// }

	

bool ManipulationExample::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


//     /* Save robot to a private member. */
//     _robot = robot;
// 
//     /* Initialize a logger which saves to the specified file. Remember that
//      * the current date/time is always appended to the provided filename,
//      * so that logs do not overwrite each other. */
//     
//     _logger = XBot::MatLogger::getLogger("/tmp/ManipulationExample_log");
//     
//     // ROS init
//     int argc = 1;
//     const char *arg = "dummy_arg";
//     char* argg = const_cast<char*>(arg);
//     char** argv = &argg;
// 
//     ros::init(argc, argv, "ManipulationExample");
// 
//     // nh and service segment_control client
//     _nh = std::make_shared<ros::NodeHandle>();
//     _client = _nh->serviceClient<ADVR_ROS::advr_segment_control>("segment_control");


    /* Save robot to a private member. */
    //_robot = robot;
    


    
    // log
    _logger = XBot::MatLogger::getLogger("/tmp/ManipulationExample_log");

    
    // ROS
    int argc = 1;
    const char *arg = "dummy_arg";
    char* argg = const_cast<char*>(arg);
    char** argv = &argg;    
    ros::init(argc, argv, "ManipulationExample");
    ros::NodeHandle* node_handle = new ros::NodeHandle;
    _nh = std::shared_ptr<ros::NodeHandle>(node_handle);

    
//     // Vision
//     sub_rgb = (*_nh).subscribe ("/multisense/left/image_color", 1, &ManipulationExample::rgb_callback, this);
//     sub_depth = (*_nh).subscribe ("/multisense/depth", 1, &ManipulationExample::depth_callback, this);
//     sub_camera_info = (*_nh).subscribe("/multisense/left/camera_info", 1, &ManipulationExample::camera_info_callback, this);
//     //sub_point_cloud = (*_nh).subscribe("/multisense/organized_image_points2", 1, &ManipulationExample::pointcloud_callback, this); // Real camera
//     sub_point_cloud = (*_nh).subscribe("/multisense/points2", 1, &ManipulationExample::pointcloud_callback, this); // In simulation
    
//     sub_vs_rh_obj_pose_2D = (*_nh).subscribe("vs_rh_obj_pose_2D", 1, &ManipulationExample::point_right_callback, this); 
//     sub_vs_rh_obj_pose_2D_FAKE = (*_nh).subscribe("vs_rh_obj_pose_2D_FAKE", 1, &ManipulationExample::point_right_callback_FAKE, this); 
    
//     pub_vs_rh_obj_pose_3D = (*_nh).advertise<geometry_msgs::PoseStamped>("vs_rh_obj_pose_3D", 1); //publish when has "point_right_callback"
//     pub_vs_rh_obj_pose_3D_FAKE = (*_nh).advertise<geometry_msgs::PoseStamped>("vs_rh_obj_pose_3D_FAKE", 1);  // publish with rgb_callback (not work) --> add to point_right_callback_FAKE -  just fake data; in camera frame
    
        
    // FSM robot
    fsm.shared_data()._nh =  std::make_shared<ros::NodeHandle>();
    //fsm.shared_data().command = command;
    //fsm.shared_data().current_command = current_command;
    
    _robot = handle->getRobotInterface();
    fsm.shared_data().current_command = std::shared_ptr<XBot::Command>(&current_command);
    
    
    fsm.shared_data()._client = _nh->serviceClient<ADVR_ROS::advr_segment_control>("segment_control");
    
    fsm.shared_data()._pub_rb_rh_grasp_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_rh_grasp_pose", 1);
    fsm.shared_data()._pub_rb_lh_grasp_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_lh_grasp_pose", 1);
    fsm.shared_data()._pub_rb_rh_pregrasp_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_rh_pregrasp_pose", 1);
    fsm.shared_data()._pub_rb_lh_pregrasp_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_lh_pregrasp_pose", 1);
    
    // Walk-man review
    fsm.shared_data()._pub_rb_rh_debris_raise_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_rh_debris_raise_pose", 1);
    fsm.shared_data()._pub_rb_lh_debris_raise_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_lh_debris_raise_pose", 1);
    
    fsm.shared_data()._pub_rb_rh_valve_turn_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_rh_valve_turn_pose", 1);
    fsm.shared_data()._pub_rb_lh_valve_turn_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_lh_valve_turn_pose", 1);
    
    // Others
    fsm.shared_data()._pub_rb_rh_raise_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_rh_raise_pose", 1);
    fsm.shared_data()._pub_rb_lh_raise_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_lh_raise_pose", 1);
    fsm.shared_data()._pub_rb_last_rh_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_last_rh_pose", 1);
    fsm.shared_data()._pub_rb_last_lh_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_last_lh_pose", 1);
    fsm.shared_data()._pub_rb_contain_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_contain_pose", 1);
    fsm.shared_data()._pub_rb_pour_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_pour_pose", 1);
    fsm.shared_data()._pub_rb_rotate_pose = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("rb_rotate_pose", 1);
    
    fsm.shared_data()._pub_grasp_plugin_rh = fsm.shared_data()._nh->advertise<std_msgs::String>("/grasp/RWrMot3/goalGrasp",1);
    fsm.shared_data()._pub_grasp_plugin_lh = fsm.shared_data()._nh->advertise<std_msgs::String>("/grasp/LWrMot3/goalGrasp",1);
    fsm.shared_data()._grasp_client = fsm.shared_data()._nh->serviceClient<ADVR_ROS::advr_grasp_control_srv>("grasp_control");
    
    
    /*Saves robot as shared variable between states*/
    //fsm.shared_data()._robot= robot;
    fsm.shared_data()._robot= handle->getRobotInterface();
    
    /*Registers states*/
    fsm.register_state(std::make_shared<myfsm::Home>());
    fsm.register_state(std::make_shared<myfsm::Detect>());
    fsm.register_state(std::make_shared<myfsm::Move>());
    fsm.register_state(std::make_shared<myfsm::Prereach>());
    fsm.register_state(std::make_shared<myfsm::Reach>());
    fsm.register_state(std::make_shared<myfsm::Grasp>());
    fsm.register_state(std::make_shared<myfsm::Ungrasp>());
    
    fsm.register_state(std::make_shared<myfsm::Debris_Raise>());
    fsm.register_state(std::make_shared<myfsm::Valve_Turn>());
    
    // some states for pouring, pick-place
    fsm.register_state(std::make_shared<myfsm::Raise>());
    fsm.register_state(std::make_shared<myfsm::Carry>());
    fsm.register_state(std::make_shared<myfsm::Reset>());
    fsm.register_state(std::make_shared<myfsm::Pour>());
    fsm.register_state(std::make_shared<myfsm::Rotate>());
    
    
    
    // Initialize the FSM with the initial state
    _robot->getJointPosition(fsm.shared_data()._q0);
    fsm.init("Home");
    


    
  

    return true;


}

void ManipulationExample::on_start(double time)
{
    
    _robot->getMotorPosition(fsm.shared_data()._q0);
    _start_time = time;
    
    std::cout << name << "NO HOMING YET" << std::endl;
    
}

void ManipulationExample::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /ManipulationExample_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
    std::cout << name << " STOPPED!!!" << std::endl;
}


void ManipulationExample::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
    
    
//     // Go to homing
//     if( (time - _first_loop_time) <= _homing_time ){
//         _q = _q0 + 0.5*(1-std::cos(3.1415*(time - _first_loop_time)/_homing_time))*(_q_home-_q0);
//         _robot->setPositionReference(_q);
//         _robot->move();
//         return;
// 
//     }
    
    // Run fsm
    fsm.run(time, 0.01);
    
    // Spin ROS
    ros::spinOnce();
    

}

bool ManipulationExample::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}

/*
void ManipulationExample::camera_info_callback(const sensor_msgs::CameraInfoPtr& msg)
{
    camera_info[0] = msg->K[0]; //fx
    camera_info[1] = msg->K[4]; //fy
    camera_info[2] = msg->K[2]; //cx
    camera_info[3] = msg->K[5]; //cy
    
    camera_width = msg->width;
    camera_height = msg->height;

//     std::cout << "... Camera info:" << std::endl;
//     std::cout << "             fx:" << camera_info[0] << std::endl;
//     std::cout << "             fy:" << camera_info[1] << std::endl;
//     std::cout << "             cx:" << camera_info[2] << std::endl;
//     std::cout << "             cy:" << camera_info[3] << std::endl;
//     std::cout << "          width:" << camera_width   << std::endl;
//     std::cout << "         height:" << camera_height  << std::endl;

}



void ManipulationExample::rgb_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr1;
    try
    {
	cv_ptr1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	rgb_img = cv_ptr1->image;
	
	cv::imshow("RGB Image - Robot", rgb_img);
	cv::waitKey(1); // show image - do not disable
	
    }
    catch (cv_bridge::Exception& ex)
    {
	ROS_ERROR("cv_bridge exception: %s", ex.what());
    }
    

    
}


void ManipulationExample::depth_callback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
	dep_img = cv_ptr->image;
    }
    catch (cv_bridge::Exception& ex)
    {
	ROS_ERROR("cv_bridge exception: %s", ex.what());
    }

//    cv::imshow("BGR8", cv_ptr->image);
//    cv::imshow("RGB8", cv_ptr_2->image);
//    cv::waitKey(30); // show image
 
}


void ManipulationExample::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    
//  cout << "I'm in stereo CB ..." << endl;
    pcl::fromROSMsg(*msg, *point_cloud_ptr);
    //std::cout << "Point cloud size in CB: " << point_cloud_ptr->points.size() << endl;
    
}*/

/*
void ManipulationExample::point_right_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    geometry_msgs::Point point_right;
    point_right.x = msg->x;
    point_right.y = msg->y;
    point_right.z = msg->z;
    
    std::cout << "Got point_right_2D: " << point_right.x << " " << point_right.y << " " << point_right.z << std::endl;
    
    // transform to 3D
    pcl::PointXYZ point_temp;
    //point_temp = pclp->points[static_cast<int>(point2d.y)*cam_width + static_cast<int>(point2d.x)];
    //point_temp = point_cloud_ptr->points[static_cast<int>(i)*camera_width + static_cast<int>(j)]; // i=row, j=colum
    point_temp = point_cloud_ptr->points[static_cast<int>(point_right.y)*camera_width + static_cast<int>(point_right.x)];  
    std::cout << "Point temp in 3D: " << point_temp.x << " " << point_temp.y << " " << point_temp.z << std::endl; // got point ok
    
    if (!std::isnan(point_temp.x) && point_temp.z > 0.3) //got a good real point
    {
	std::cout << "Point temp in 3D (good): " << point_temp.x << " " << point_temp.y << " " << point_temp.z << std::endl;
	
	grasp_pose_right.header.frame_id = "multisense/left_camera_optical_frame"; // check later
	//grasp_pose_right.header.frame_id = "world"; // do not work
	grasp_pose_right.pose.position.x = point_temp.x;
	grasp_pose_right.pose.position.y = point_temp.y;
	grasp_pose_right.pose.position.z = point_temp.z;
	grasp_pose_right.pose.orientation.x = 0;
	grasp_pose_right.pose.orientation.y = 0;
	grasp_pose_right.pose.orientation.z = 0;
	grasp_pose_right.pose.orientation.w = 1;  // Identity - no rotation
	
	pub_vs_rh_obj_pose_3D.publish(grasp_pose_right); //publish ok	
    }
}*/


// void ManipulationExample::point_right_callback_FAKE(const geometry_msgs::Point::ConstPtr& msg)
// {
//     geometry_msgs::Point point_right;
//     point_right.x = msg->x;
//     point_right.y = msg->y;
//     point_right.z = msg->z;
//     
//     //std::cout << "Got FAKE 2D point: " << point_right.x << " " << point_right.y << " " << point_right.z << std::endl;
//     
//     // Publish fake data - debug
//     geometry_msgs::PoseStamped fake_pose;
//     fake_pose.header.frame_id = "multisense/left_camera_optical_frame"; // publish in camera frame
//     fake_pose.pose.position.x = 0.38;
//     fake_pose.pose.position.y = 0.05;
//     fake_pose.pose.position.z = 0.63;
//     fake_pose.pose.orientation.x = 0;
//     fake_pose.pose.orientation.y = 0;
//     fake_pose.pose.orientation.z = 0;
//     fake_pose.pose.orientation.w = 1;  // Identity - no rotation
//     std::cout << "Publishing FAKE pose ... " << std::endl;
//     pub_vs_rh_obj_pose_3D_FAKE.publish(fake_pose); //publish ok	
//     
// }








}
