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

/* Specify that the class XBotPlugin::ManipulationExample is a XBot RT plugin with name "ManipulationExample" */
REGISTER_XBOT_PLUGIN(ManipulationExample, XBotPlugin::ManipulationExample)

namespace XBotPlugin {

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

    
	
    // FSM
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
    
    
    // log
    //_robot->initLog("/tmp/homing_example_log", 100000);


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



}
