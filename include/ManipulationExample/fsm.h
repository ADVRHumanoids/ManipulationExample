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
#include <XBotInterface/StateMachine.h>

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <ADVR_ROS/advr_segment_control.h>
#include <ADVR_ROS/advr_grasp_control_srv.h>

#include <trajectory_utils/segment.h>
#include <trajectory_utils/Cartesian.h>

#include <eigen_conversions/eigen_msg.h>
#include <XBotCore-interfaces/XDomainCommunication.h>

// TF
#include <tf/transform_listener.h>
#include <Eigen/Dense>

//PCL
#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>
#include <pcl/common/time.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

namespace myfsm{

/*Example how to define a custom Event*/
/*  class MyEvent : public XBot::FSM::Event{
    public:
      MyEvent(int id): id(id) {}
      
      int id;    
    };
*/

/*Example how to define a custom Message*/   
/*  class MyMessage : public XBot::FSM::Message {
    public:
      
      MyMessage (int id):id(id){};
      
      int id;
    };
*/
    struct SharedData
    {
	// ros stuff
	std::shared_ptr<ros::NodeHandle> _nh;
	ros::Publisher _pub_rb_rh_grasp_pose;    // publisher for right hand final grasp pose - in world frame (rb - robot)
	ros::Publisher _pub_rb_rh_pregrasp_pose; // publisher for right hand pregrasp pose
	
	ros::Publisher _pub_rb_lh_grasp_pose;
	ros::Publisher _pub_rb_lh_pregrasp_pose;
	    
	ros::Publisher _pub_rb_rh_raise_pose;
	ros::Publisher _pub_rb_lh_raise_pose;
	
	ros::Publisher _pub_rb_last_rh_pose;
	ros::Publisher _pub_rb_last_lh_pose;
	
	ros::Publisher _pub_rb_contain_pose;
	
	ros::Publisher _pub_grasp_plugin_rh; // grasp plugin for right hand
	ros::Publisher _pub_grasp_plugin_lh; 
	ros::ServiceClient _grasp_client;
	
	XBot::RobotInterface::Ptr _robot;

	ros::ServiceClient _client;
	
	XBot::SubscriberRT<XBot::Command> command;
	XBot::Command current_command;

	Eigen::VectorXd _q0;
      
	
	Eigen::Affine3d ei_start_rh_pose, ei_start_lh_pose; // keep starting pose of left hand or right hand
	
	// Command string for reading poses
	//std::string pose_cmd_ = "hose_pose";
	
	// Working frames
	const std::string world_frame = "world_odom";
	const std::string left_camera_frame = "multisense/left_camera_optical_frame";
	
	// grasp pose
	geometry_msgs::PoseStamped::ConstPtr grasp_pose, pregrasp_pose, raise_pose, contain_pose;  // must use ConstPtr, not Ptr
	
	// last right hand, left hand pose
	geometry_msgs::PoseStamped::ConstPtr pst_last_rh_pose, pst_last_lh_pose;
	
	// ros topic for 3D pose of objects (in camera frame) - for right hand, left hand
	const std::string vs_rh_obj_pose_3D = "vs_rh_obj_pose_3D";  // MUST BE THE SAME IN: pub_rh_obj_pose_3D = (*_nh).advertise<geometry_msgs::PoseStamped>("vs_rh_obj_pose_3D", 1);
	const std::string vs_lh_obj_pose_3D = "vs_lh_obj_pose_3D";
	const std::string vs_rh_obj_pose_3D_FAKE = "vs_rh_obj_pose_3D_FAKE";  // fake pose - for debugging
	const std::string vs_lh_obj_pose_3D_FAKE = "vs_lh_obj_pose_3D_FAKE";  // fake pose for left hand
	const std::string vs_contain_pose_3D = "vs_contain_pose_3D";
	
	const std::string rh_id = "rh"; // id for choosing right hand
	const std::string lh_id = "lh";
	const std::string side_grasp = "sidegrasp"; // grasping type: sidegrasp --> move hand parallel to homing pose
	const std::string top_grasp = "topgrasp";    // grasping type: topgrasp --> grasp from top to bottom
	
	//std::string current_hand = rh_id;  // current hand in use - default is right hand
	std::string current_hand = lh_id;  // current hand in use - default is right hand
	std::string current_grasp_strategy = side_grasp; // current grasp strategy - default is side grasp
	//std::string current_grasp_strategy = top_grasp; // current grasp strategy - default is topgrasp
	
	// debug 
	bool verbose_print = true;
	const std::string str_seperator = "===========================================================";
    };
    
    // ad tfHandler class
    class tfHandler
    {
    public:
	tfHandler():
	_listener(), _gm_transform(), _transform()
	{}

	/** \brief TBD. */
	
	bool getTransformTf (const std::string& parent,
			    const std::string& child,
			    Eigen::Affine3d& world_T_bl)
	{
	    try
	    {
		ros::Time now = ros::Time::now();
		//if(_listener.waitForTransform(child, parent, now, ros::Duration(5.0)))
		//{
		_listener.lookupTransform(child, parent,  ros::Time(0), _transform);
		tf::transformTFToMsg(_transform, _gm_transform);
		tf::transformMsgToEigen(_gm_transform, world_T_bl);
		return true;
		//}
		//else
		//  return false;
	    }
	    catch (tf::TransformException ex)
	    {
		ROS_ERROR("%s",ex.what());
		return false;
	    }
	}
	
	bool getTransformTf_direct (const std::string& parent,
				    const std::string& child,
				    tf::StampedTransform& _tf_transform_direct)  // return tf Transform instead of Eigen
	{
	    try
	    {
		ros::Time now = ros::Time::now();
		//if(_listener.waitForTransform(child, parent, now, ros::Duration(5.0)))
		//{
		_listener.lookupTransform(child, parent,  ros::Time(0), _tf_transform_direct);
		return true;
		//}
		//else
		//  return false;
	    }
	    catch (tf::TransformException ex)
	    {
		ROS_ERROR("%s",ex.what());
		return false;
	    }
	}

    private:
	tf::TransformListener _listener;
	geometry_msgs::Transform _gm_transform;
	tf::StampedTransform _transform;
    };
    
     
    
    
    class MacroState : public  XBot::FSM::State< MacroState , SharedData >
    {
      public:
        virtual void entry(const XBot::FSM::Message& msg) {};
        virtual void react(const XBot::FSM::Event& e){};
	
	// TF
	tfHandler tf;
    };  

 
    
    class Home : public MacroState
    {
	virtual std::string get_name() const { return "Home"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:
        
    };
     
    
    class Detect : public MacroState
    {
	virtual std::string get_name() const { return "Detect"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:
        
    };
     

    class Move : public MacroState // Move = Prereach + Reach
    {
	virtual std::string get_name() const { return "Move"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:
        
    };
    
    class Prereach : public MacroState
    {
	virtual std::string get_name() const { return "Prereach"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:
        
    };
    
    class Reach : public MacroState
    {
	virtual std::string get_name() const { return "Reach"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:
        
     };
     
    class Grasp : public MacroState
    {
	virtual std::string get_name() const { return "Grasp"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:
        
     };
     
    class Ungrasp : public MacroState
    {
	virtual std::string get_name() const { return "Ungrasp"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:
        
     };
     
     
    class Raise : public MacroState
    {
	virtual std::string get_name() const { return "Raise"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:
        
     };
 
}
