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
	ros::Publisher _pub_rh_grasp_pose;    // publisher for right hand final grasp pose
	ros::Publisher _pub_rh_pregrasp_pose; // publisher for right hand pregrasp pose
	
	
	XBot::RobotInterface::Ptr _robot;

	ros::ServiceClient _client;
	geometry_msgs::PoseStamped::ConstPtr _hose_grasp_pose;
	//geometry_msgs::PoseStamped _hose_grasp_pose;

	XBot::SubscriberRT<XBot::Command> command;
	XBot::Command current_command;

	Eigen::VectorXd _q0;
      
	// Starting left and right hand poses
	Eigen::Affine3d sl_hand_pose, sr_hand_pose;
	
	// Command string for reading poses
	//std::string pose_cmd_ = "hose_pose";
	
	// Working frame id
	std::string frame_id_ = "world_odom";
	
	// right hand grasp pose
	geometry_msgs::PoseStamped::ConstPtr rh_grasp_pose, rh_pregrasp_pose;  // must use ConstPtr, not Ptr
	
	// ID of ros topic - for right hand
	std::string rh_grasp_topic = "vs_rh_grasp_topic_3D";
	
	const std::string rh_id = "rh"; // id for choosing right hand
	const std::string lh_id = "lh";
	const std::string side_grasp = "sidegrasp"; // grasping type: sidegrasp --> move hand parallel to homing pose
	const std::string top_grasp = "topgrasp";    // grasping type: topgrasp --> grasp from top to bottom
	
	std::string current_hand = rh_id;  // current hand in use - default is right hand
	std::string current_grasp_strategy = side_grasp; // current grasp strategy - default is side grasp
	
	
	
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
				    tf::StampedTransform& _tf_transform_direct)
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
     
    class Reach : public MacroState
    {
	virtual std::string get_name() const { return "Reach"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:
        
     };
     
    class Idle : public MacroState
    {
	virtual std::string get_name() const { return "Idle"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:
        
     };
     
 
    class Move_RH : public MacroState
    {
	virtual std::string get_name() const { return "Move_RH"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:
        
     };
 
    class Grasp_RH : public MacroState
    {
	virtual std::string get_name() const { return "Grasp_RH"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:

     };
 
    class Grasp_RH_Done : public MacroState
    {
	virtual std::string get_name() const { return "Grasp_RH_Done"; }

	virtual void run(double time, double period);

	virtual void entry(const XBot::FSM::Message& msg);

	virtual void react(const XBot::FSM::Event& e);

	virtual void exit ();

	private:
        
     };
 
}
