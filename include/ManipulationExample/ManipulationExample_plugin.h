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

#ifndef ManipulationExample_PLUGIN_H_
#define ManipulationExample_PLUGIN_H_

#include <XCM/XBotControlPlugin.h>

#include <ros/ros.h>

#include <ADVR_ROS/advr_segment_control.h>

#include <trajectory_utils/segment.h>
#include <trajectory_utils/Cartesian.h>

#include "fsm.h"

namespace XBotPlugin 
{
/**
 * @brief ManipulationExample XBot RT Plugin
 *
**/
    class ManipulationExample : public XBot::XBotControlPlugin
    {

    public:

	virtual bool init_control_plugin(std::string path_to_config_file,
					XBot::SharedMemory::Ptr shared_memory,
					XBot::RobotInterface::Ptr robot);

	virtual bool close();

	virtual void on_start(double time);

	virtual void on_stop(double time);

    protected:

	virtual void control_loop(double time, double period);

    private:

	// Robot
	XBot::RobotInterface::Ptr _robot;
	
	// Loger
	XBot::MatLogger::Ptr _logger;

	// Homing
	Eigen::VectorXd _q0, _q_home, _q, _k, _d, _k0, _d0, _qref;
	double _time, _homing_time, _first_loop_time;
	//ForceTorqueSensor::ConstPtr _l_arm_ft;
	Eigen::Matrix<double, 6, 1> _l_arm_wrench;
	double _l_hand_pos;
	double _l_hand_ref;
	bool _close_hand;
	
	// ROS
	std::shared_ptr<ros::NodeHandle> _nh;
	ros::ServiceClient _client;
	
	// State Machine
	XBot::FSM::StateMachine< myfsm::MacroState , myfsm::SharedData > fsm;
	
	// Utils
	double _start_time;
    };

}

#endif // ManipulationExample_PLUGIN_H_
