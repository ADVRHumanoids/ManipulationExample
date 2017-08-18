#include <vector>

#include "fsm.h"




void myfsm::Homing::react(const XBot::FSM::Event& e) 
{

}

void myfsm::Homing::entry(const XBot::FSM::Message& msg)
{

    std::cout << "Homing_entry" << std::endl;
  
/*  
    //CALL SERVICE TO MOVE
    // send a trajectory for the end effector as a segment
    
    shared_data()._robot->sense();    
    
    // RIGHT HAND
    
    Eigen::Affine3d poseRightHand;
    geometry_msgs::Pose start_frame_pose;

    shared_data()._robot->model().getPose("RSoftHand", "Waist", poseRightHand);
    tf::poseEigenToMsg (poseRightHand, start_frame_pose);

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame.pose = start_frame_pose;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "RSoftHand";
    start.frame = start_frame;
    
    // define the end frame - RIGHT HAND
    geometry_msgs::PoseStamped end_frame;
    
    end_frame.pose = start_frame_pose;    

    end_frame.pose.position.x = 0.306;
    end_frame.pose.position.y = -0.393;
    end_frame.pose.position.z = -0.069;    
    
    end_frame.pose.orientation.x = -0.068;
    end_frame.pose.orientation.y = -0.534;
    end_frame.pose.orientation.z = 0.067;
    end_frame.pose.orientation.w = 0.840;    
    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
    end.frame = end_frame;

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 5.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = end;            // end pose 
    
    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);
    
    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "Waist";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;
    
    // call the service
    shared_data()._client.call(srv);    
    
    
    // at the moment not possible to move both hands in the same state
    
//     /******************************************************************/
//     
//     // LEFT HAND
//     
//     Eigen::Affine3d poseLeftHand;
//     geometry_msgs::Pose start_frame_pose2;
// 
//     shared_data()._robot->model().getPose("LSoftHand",poseLeftHand);
//     tf::poseEigenToMsg (poseLeftHand, start_frame_pose2);
// 
//     // define the start frame 
//     geometry_msgs::PoseStamped start_frame2;
//     start_frame2.pose = start_frame_pose2;
// 
//     trajectory_utils::Cartesian start2;
//     start2.distal_frame = "LSoftHand";
//     start2.frame = start_frame2;
//     
//     // define the end frame - LEFT HAND
//     geometry_msgs::PoseStamped end_frame2;
//     end_frame2.pose.position.x = 0.732;
//     end_frame2.pose.position.y = 0.250;
//     end_frame2.pose.position.z = 0.029;
//     
//     end_frame2.pose.orientation.x = -0.669;
//     end_frame2.pose.orientation.y = -0.464;
//     end_frame2.pose.orientation.z = 0.120;
//     end_frame2.pose.orientation.w = 0.568;
//     
//     trajectory_utils::Cartesian end2;
//     end2.distal_frame = "LSoftHand";
//     end2.frame = end_frame2;
// 
// 
//     // define the first segment
//     trajectory_utils::segment s2;
//     s2.type.data = 0;        // min jerk traj
//     s2.T.data = 5.0;         // traj duration 5 second      
//     s2.start = start2;        // start pose
//     s2.end = end2;            // end pose 
//     
//     // only one segment in this example
//     std::vector<trajectory_utils::segment> segments2;
//     segments2.push_back(s2);
//     
//     // prapere the advr_segment_control
//     ADVR_ROS::advr_segment_control srv2;
//     srv2.request.segment_trj.header.frame_id = "Waist";
//     srv2.request.segment_trj.header.stamp = ros::Time::now();
//     srv2.request.segment_trj.segments = segments2;
//     
//     // call the service
//     shared_data()._client.call(srv2);
//     
//     
//     /******************************************************************/   */ 

}


void myfsm::Homing::run(double time, double period)
{
  
  std::cout << "Homing run" << std::endl;
  
  //TBD: Check if the RH has reached the homing_pose
  
//   // blocking reading: wait for a command
//   if(shared_data().command.read(shared_data().current_command))
//   {
//     std::cout << "Command: " << shared_data().current_command.str() << std::endl;
// 
//     // Homing failed
//     if (!shared_data().current_command.str().compare("homing_fail"))
//       transit("Homing");
//     
//     // Homing Succeeded
//     if (!shared_data().current_command.str().compare("homing_success"))
//       transit("Reached");
//   }

}


void myfsm::Homing::exit ()
{

 

}
