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


    /* Save robot to a private member. */
    _robot = robot;

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */
    
    _logger = XBot::MatLogger::getLogger("/tmp/ManipulationExample_log");
    
    // ROS init
    int argc = 1;
    const char *arg = "dummy_arg";
    char* argg = const_cast<char*>(arg);
    char** argv = &argg;

    ros::init(argc, argv, "ManipulationExample");

    return true;


}

void ManipulationExample::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /ManipulationExample_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
    
    // nh and service segment_control client
//     _nh = std::make_shared<ros::NodeHandle>();
//     _client = _nh->serviceClient<ADVR_ROS::advr_segment_control>("segment_control");
//     _feedBack = _nh->subscribe("Manipulation_status",1,&ManipulationExample::on_manipulation_status,this);
//     manipulation_status = false;
// 
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
//     _robot->model().getPose("RSoftHand", pose);
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
//     end_frame.pose.position.x += 0.3;
//     end_frame.pose.position.z += 0.3;
//     
//     trajectory_utils::Cartesian start;
//     start.distal_frame = "RSoftHand";
//     start.frame = start_frame;
//     
//     trajectory_utils::Cartesian end;
//     end.distal_frame = "RSoftHand";
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
     
      _nh = std::make_shared<ros::NodeHandle>();
      _grasp_client = _nh->serviceClient<ADVR_ROS::advr_grasp_control_srv>("grasp_control");
      _feedBack = _nh->subscribe("Manipulation_status",1,&ManipulationExample::on_manipulation_status,this);
      manipulation_status = false;
      
     
      
    
    
    // hands
//     std::map<std::string, XBot::Hand::Ptr> hands = _robot->getHand();
//     // RHand
//     auto r_hand_it = hands.find("r_handj");
//     if(r_hand_it != hands.end()) {
//         _RHand = r_hand_it->second;
//     }
//     // LHand
//     auto l_hand_it = hands.find("l_handj");
//     if(l_hand_it != hands.end()) {
//         _LHand = l_hand_it->second;
//     }
      
      ADVR_ROS::advr_grasp_control_srv srv;
      srv.request.right_grasp = 0.0;
      srv.request.left_grasp = 1.0;
       // call the service
      _grasp_client.call(srv);
    
}

void ManipulationExample::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /ManipulationExample_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void ManipulationExample::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
    
//     command.read(current_command);
//     advr_grasp_control.srvadvr_grasp_control.srv
//     if(current_command.str() == "open RHand" && (_RHand != nullptr)) {
//         DPRINTF("Opening RHand\n");
//         _RHand->grasp(0.0);
//     }
//     else if(current_command.str() == "open LHand" && (_LHand != nullptr)) {
//         DPRINTF("Opening LHand\n");
//         _LHand->grasp(0.0);
//     }
//     else if(current_command.str() == "close RHand" && (_RHand != nullptr)) {
//         DPRINTF("Closing RHand\n");
//         _RHand->grasp(1.0);
//     }
//     else if(current_command.str() == "close LHand" && (_LHand != nullptr)) {
//         DPRINTF("Closing LHand\n");
//         _LHand->grasp(1.0);
//     }
//     
//     
//     _robot->move();


//     ADVR_ROS::advr_grasp_control srv;
//     
//     command.read(current_command);
//     
//     if(current_command.str() == "open RHand" && (_RHand != nullptr)) {
//         DPRINTF("Opening RHand\n");
//         srv.request.grasp = false;
//     }
//     else if(current_command.str() == "open LHand" && (_LHand != nullptr)) {
//         DPRINTF("Opening LHand\n");
//          srv.request.grasp = false;
//     }
//     else if(current_command.str() == "close RHand" && (_RHand != nullptr)) {
//         DPRINTF("Closing RHand\n");
//          srv.request.grasp = true;
//     }
//     else if(current_command.str() == "close LHand" && (_LHand != nullptr)) {
//         DPRINTF("Closing LHand\n");
//          srv.request.grasp = true;
//     }
//     
//       // call the service
//       _client.call(srv);
     
    if (manipulation_status == false){
        
//         std::cout<<"MANIPULATION DONE"<<std::endl;
    } 
    else{
      
//         std::cout<<"MANIPULATION STATUS RUNNING"<<std::endl;
    }
    
    ros::spinOnce();
    
    return;

}

bool ManipulationExample::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}

void ManipulationExample::on_manipulation_status(const std_msgs::Bool::ConstPtr& msg)
{
  
  manipulation_status = msg->data;

}

}
