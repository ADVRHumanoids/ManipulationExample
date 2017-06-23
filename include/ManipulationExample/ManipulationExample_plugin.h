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

//#include "RTControl/XBotRTControlClass.h"
#include "fsm_definition.h"

namespace XBotPlugin {

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

    XBot::RobotInterface::Ptr _robot;

    double _start_time;

    Eigen::VectorXd _q0;

    XBot::MatLogger::Ptr _logger;
    
    std::shared_ptr<ros::NodeHandle> _nh;
    ros::ServiceClient _client;

    // State Machine
    XBot::FSM::StateMachine< myfsm::MacroState , myfsm::SharedData > fsm;
    
    // ------ homing ---------------------------
    double _time, _homing_time, _first_loop_time;
    Eigen::VectorXd _q_home, _qref;
    XBot::JointNameMap _q_name;
    
    //
    //XBotRTControlClass RTControl;
    
    //
    Eigen::VectorXd _q, _q_msr;
    
    //
    std::vector<double> FTSensor, HandFTSensor, qall;
    
    std::map< std::string, XBot::ForceTorqueSensor::ConstPtr > ft_map;
    std::map< std::string, XBot::ImuSensor::ConstPtr >  imu_map;
    
};

}

#endif // ManipulationExample_PLUGIN_H_
