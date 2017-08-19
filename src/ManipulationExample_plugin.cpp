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


//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//PCL
#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>
#include <pcl/common/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

//cv_bridge
#include <cv_bridge/cv_bridge.h>


/* Specify that the class XBotPlugin::ManipulationExample is a XBot RT plugin with name "ManipulationExample" */
REGISTER_XBOT_PLUGIN(ManipulationExample, XBotPlugin::ManipulationExample)

namespace XBotPlugin {


// Add constructor    
ManipulationExample::ManipulationExample(): point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>)
{

}

	
bool ManipulationExample::init_control_plugin(std::string path_to_config_file,
                                                    XBot::SharedMemory::Ptr shared_memory,
                                                    XBot::RobotInterface::Ptr robot)
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
    _robot = robot;
    
    // log
    _logger = XBot::MatLogger::getLogger("/tmp/ManipulationExample_log");

    
//     // HOMING 
//     _robot->getRobotState("home", _q_home);
//     _robot->sense();
//     _robot->getJointPosition(_q0);
//     _robot->getStiffness(_k0);
//     _robot->getDamping(_d0);
//     _k = _k0;
//     _d = _d0;
//     _q = _q0;
//     _qref = _q0;
// 
//     std::cout << "_q_home from SRDF : " << _q_home << std::endl;
//     _time = 0;
//     _homing_time = 4;
// 
//     _robot->print();
// 
//     _l_hand_pos = _l_hand_ref = 0.0;
//     _close_hand = true;

    
    // ROS
    int argc = 1;
    const char *arg = "dummy_arg";
    char* argg = const_cast<char*>(arg);
    char** argv = &argg;    
    ros::init(argc, argv, "ManipulationExample");
    ros::NodeHandle* node_handle = new ros::NodeHandle;
    _nh = std::shared_ptr<ros::NodeHandle>(node_handle);

    
    // Vision
    sub_rgb = (*_nh).subscribe ("/multisense/left/image_color", 1, &ManipulationExample::rgb_callback, this);
    sub_depth = (*_nh).subscribe ("/multisense/depth", 1, &ManipulationExample::depth_callback, this);
    sub_camera_info = (*_nh).subscribe("/multisense/left/camera_info", 1, &ManipulationExample::camera_info_callback, this);
    //sub_point_cloud = (*_nh).subscribe("/multisense/organized_image_points2", 1, &ManipulationExample::pointcloud_callback, this); // Real camera
    sub_point_cloud = (*_nh).subscribe("/multisense/points2", 1, &ManipulationExample::pointcloud_callback, this); // In simulation
    sub_vision_data = (*_nh).subscribe("vision_data", 1, &ManipulationExample::vision_data_callback, this);  // get data from vision module
    sub_point_right = (*_nh).subscribe("vs_point_right_2D", 1, &ManipulationExample::point_right_callback, this); 
    
    pub_pose_right_3D = (*_nh).advertise<geometry_msgs::PoseStamped>("vs_pose_right_3D", 1);
    
//     // FSM vision
//     fsm.shared_data().point_cloud_ptr = point_cloud_ptr;
//     fsm.shared_data().vision_string = vision_string;
    
    
    // FSM robot
    fsm.shared_data().command = command;
    fsm.shared_data().current_command = current_command;
    fsm.shared_data()._client = _nh->serviceClient<ADVR_ROS::advr_segment_control>("segment_control");
    
    /*Saves robot as shared variable between states*/
    fsm.shared_data()._robot= robot;
    
    /*Registers states*/
    fsm.register_state(std::make_shared<myfsm::Home>());
    fsm.register_state(std::make_shared<myfsm::Idle>());
    fsm.register_state(std::make_shared<myfsm::Move_RH>());
    fsm.register_state(std::make_shared<myfsm::Grasp_RH>());
    fsm.register_state(std::make_shared<myfsm::Grasp_RH_Done>());
    
    // Initialize the FSM with the initial state
    _robot->getJointPosition(fsm.shared_data()._q0);
    fsm.init("Home");
    


    
  

    return true;


}

void ManipulationExample::on_start(double time)
{
//     /* This function is called on plugin start, i.e. when the start command
//      * is sent over the plugin switch port (e.g. 'rosservice call /ManipulationExample_switch true').
//      * Since this function is called within the real-time loop, you should not perform
//      * operations that are not rt-safe. */
// 
//     /* Save the plugin starting time to a class member */
//     _robot->getMotorPosition(_q0);
// 
//     /* Save the robot starting config to a class member */
//     _start_time = time;
//     
//     // sense and sync model
//     _robot->sense();
//     
//     // Get currnt Left hand pose
//     Eigen::Affine3d pose;
//     geometry_msgs::Pose start_frame_pose;
//     _robot->model().getPose("LSoftHand", pose);
//     
//     // from eigen to ROS pose
//     tf::poseEigenToMsg (pose, start_frame_pose);
// 
//     // define the PoseStamped start_frame amd end_frame
//     geometry_msgs::PoseStamped start_frame;
//     start_frame.pose = start_frame_pose;
//     
//     geometry_msgs::PoseStamped end_frame;
//     end_frame.pose = start_frame_pose;
//     end_frame.pose.position.y += 0.3;
//     end_frame.pose.position.z += 0.3;
//     
//     trajectory_utils::Cartesian start;
//     start.distal_frame = "LSoftHand";
//     start.frame = start_frame;
//     
//     trajectory_utils::Cartesian end;
//     end.distal_frame = "LSoftHand";
//     end.frame = end_frame;
//     
//     // define the first segment
//     trajectory_utils::segment s1;
//     s1.type.data = 0;        // min jerk traj
//     s1.T.data = 5.0;         // traj duration 5 second      
//     s1.start = start;        // start pose
//     s1.end = end;            // end pose 
//     
//     // only one segment in this example
//     std::vector<trajectory_utils::segment> segments;
//     segments.push_back(s1);
//     
//     // prapere the advr_segment_control
//     ADVR_ROS::advr_segment_control srv;
//     srv.request.segment_trj.header.frame_id = "world";
//     srv.request.segment_trj.header.stamp = ros::Time::now();
//     srv.request.segment_trj.segments = segments;
//     
//     // call the service
//     _client.call(srv);
//     ros::spinOnce();
    
    
//     // for homing
//     _first_loop_time = time;
//     _robot->sense();
//     _robot->getJointPosition(_q0);
//     std::cout << name << " HOMING STARTED++--*--*!!!" << std::endl;
    
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
	cv::waitKey(1); // to show image
	
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
//    cv::waitKey(30); // to show image
 
}


void ManipulationExample::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    
//  cout << "I'm in stereo CB ..." << endl;
    pcl::fromROSMsg(*msg, *point_cloud_ptr);
    //std::cout << "Point cloud size in CB: " << point_cloud_ptr->points.size() << endl;
    
}

void ManipulationExample::vision_data_callback(const std_msgs::String::ConstPtr& msg)
{

    vision_string = msg->data.c_str();  // 7_0.999388_404_256_532_372_OBJandCEN_4_469_277_CENTROIDSeperator_5_474_319
    std::cout << "Vision data string: " << vision_string << std::endl;
    
    // parse string to and get 3D centroid point
    
     
     
    //// send back to FSM - not work
    //fsm.shared_data().vision_string = vision_string;
}

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
	
	grasp_pose_right.header.frame_id = ""; // check later
	grasp_pose_right.pose.position.x = point_temp.x;
	grasp_pose_right.pose.position.y = point_temp.y;
	grasp_pose_right.pose.position.z = point_temp.z;
	grasp_pose_right.pose.orientation.x = 0;
	grasp_pose_right.pose.orientation.y = 0.1;
	grasp_pose_right.pose.orientation.z = 0;
	grasp_pose_right.pose.orientation.w = 1;
	
	pub_pose_right_3D.publish(grasp_pose_right); //publish ok
	 
	
	
    }
}








}
