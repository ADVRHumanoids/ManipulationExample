#include <string>
#include <eigen_conversions/eigen_msg.h>

#include "fsm.h"



//Begin Home State
void myfsm::Home::react (const XBot::FSM::Event& e)
{
    std::cout << "Home react" << std::endl;
}

void myfsm::Home::entry (const XBot::FSM::Message& msg)
{
      
}

void myfsm::Home::run (double time, double period)
{
    std::cout << "State: Home run" << std::endl;
    
    std::string message_string = shared_data().vision_string;
    std::cout << "--- Got message: in Home run: " << message_string << std::endl;
    
    // blocking reading: wait for a command
    //if(!shared_data().command.read(shared_data().current_command))
    //  std::cout << shared_data().current_command.str() << std::endl;

//     // Wait for RH_Pose, i.e. the Hose Grasp Pose (hose_grasp_pose)
//     shared_data()._hose_grasp_pose =
//     ros::topic::waitForMessage<geometry_msgs::PoseStamped>("hose_grasp_pose");
// 
//     // Debug msg
//     std::cout << "Got pose message: " << std::endl;
//     std::cout << shared_data()._hose_grasp_pose->pose.position.x << std::endl;

        // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
	std::cout << "Command: " << shared_data().current_command.str() << std::endl;

	// RH Move failed
	if (!shared_data().current_command.str().compare("done"))
	    transit("Idle");
    }
    
    
}

void myfsm::Home::exit ()
{

}
//End Home State



//Begin Idle State
void myfsm::Idle::react (const XBot::FSM::Event& e)
{
    std::cout << "Home react" << std::endl;
}

void myfsm::Idle::entry (const XBot::FSM::Message& msg)
{
      
}

void myfsm::Idle::run (double time, double period)
{
    std::cout << "State: Idle run" << std::endl;

//     // blocking reading: wait for a command
//     //if(!shared_data().command.read(shared_data().current_command))
//     //  std::cout << shared_data().current_command.str() << std::endl;
// 
    
    
//     // Wait for RH_Pose, i.e. the Hose Grasp Pose (hose_grasp_pose)
//     shared_data()._hose_grasp_pose =
//     ros::topic::waitForMessage<geometry_msgs::PoseStamped>("hose_grasp_pose");
// 
//     // Debug msg
//     std::cout << "Got pose message: " << std::endl;
//     std::cout << shared_data()._hose_grasp_pose->pose.position.x << std::endl;

    // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
	std::cout << "Command: " << shared_data().current_command.str() << std::endl;

	// RH Move failed
	if (!shared_data().current_command.str().compare("move"))
	    transit("Move_RH");
    }

}

void myfsm::Idle::exit ()
{

}
//End Idle State



// Begin Move_RH State
void myfsm::Move_RH::react (const XBot::FSM::Event& e)
{

}

void myfsm::Move_RH::entry (const XBot::FSM::Message& msg)
{
    
    ///////////////////////////////////
     // Info messages
    std::cout << "State: Move_LH entry" << std::endl;
//     std::cout << "Pub /hose_pose for left hand..." << std::endl;

//     // Wait for RH_Pose, i.e. the Hose Grasp Pose (hose_grasp_pose)
//     shared_data ()._hose_grasp_pose =
//     ros::topic::waitForMessage<geometry_msgs::PoseStamped>
// 	(shared_data ().pose_cmd_);

    //     // Wait for RH_Pose, i.e. the Hose Grasp Pose (hose_grasp_pose)
    //shared_data ().rh_grasp_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(shared_data ().rh_grasp_pose_topic);
	
	

    // Move LH to LH_Pose (1 mid point in the z-axis fixed dist)

    // Only for first time sense and sync to the model
    //TBD: put flag to check if it is first run to retrieve hands pose
    shared_data()._robot->sense();
    Eigen::Affine3d world_T_bl;
    std::string fb;
    shared_data()._robot->model().getFloatingBaseLink(fb);
    std::cout << "--- floating base link: " << fb << std::endl;
    
    tf.getTransformTf(fb, shared_data ().frame_id_, world_T_bl);
    std::cout << "--- Affine3d world_T_bl: " << world_T_bl.matrix() << std::endl;
    
    shared_data()._robot->model().setFloatingBasePose(world_T_bl);
    shared_data()._robot->model().update();

    // Get current hand
    KDL::Frame hand_pose_KDL;
    Eigen::Affine3d hand_pose, r_hand_pose, camera_to_world;
    geometry_msgs::Pose start_hand_pose, r_start_hand_pose, camera_pose_msgs;

    // Get hands poses
    //shared_data()._robot->model().getPose("LSoftHand", hand_pose);
    //shared_data().sl_hand_pose = hand_pose;
    shared_data()._robot->model().getPose("RSoftHand", r_hand_pose);
    shared_data().sr_hand_pose = r_hand_pose;
    
    //// Get camera pose: will transform a point from camera_frame to world frame ???
    shared_data()._robot->model().getPose("multisense/left_camera_optical_frame", camera_to_world);
    
//     // convert object pose to tf
//     tf::Transform cam_to_target;
//     tf::poseMsgToTF(shared_data ().rh_grasp_pose->pose, cam_to_target);
    

    // Transform from Eigen::Affine3d to geometry_msgs::Pose
    //tf::poseEigenToMsg (hand_pose, start_hand_pose);
    tf::poseEigenToMsg (r_hand_pose, r_start_hand_pose);

    // Define the start frame as geometry_msgs::PoseStamped
    //geometry_msgs::PoseStamped start_hand_pose_stamped;
    //start_hand_pose_stamped.pose = start_hand_pose;
    geometry_msgs::PoseStamped r_start_hand_pose_stamped;
    r_start_hand_pose_stamped.pose = r_start_hand_pose;

    // Create the Cartesian trajectories
    trajectory_utils::Cartesian start_traj;
    //start_traj.distal_frame = "LSoftHand";
    //start_traj.frame = start_hand_pose_stamped;
    start_traj.distal_frame = "RSoftHand";
    start_traj.frame = r_start_hand_pose_stamped;
    
    

    // define the end frame
    //geometry_msgs::PoseStamped end_hand_pose_stamped;
    //end_hand_pose_stamped.pose = start_hand_pose;

    geometry_msgs::PoseStamped r_end_hand_pose_stamped;

    r_end_hand_pose_stamped.pose.position.x = 0.619;
    r_end_hand_pose_stamped.pose.position.y = -0.29;
    r_end_hand_pose_stamped.pose.position.z = 0.873;
    r_end_hand_pose_stamped.pose.orientation.x = 0;
    r_end_hand_pose_stamped.pose.orientation.y = -0.5591931143131625;
    r_end_hand_pose_stamped.pose.orientation.z = 0;
    r_end_hand_pose_stamped.pose.orientation.w = 0.8290374303399975;
    shared_data().rh_grasp_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(r_end_hand_pose_stamped));
    

    //r_end_hand_pose_stamped.pose = shared_data().rh_grasp_pose->pose;    
//     r_end_hand_pose_stamped.pose.position = shared_data().rh_grasp_pose->pose.position;    
//     r_end_hand_pose_stamped.pose.orientation.x = 0;
//     r_end_hand_pose_stamped.pose.orientation.y = 0;
//     r_end_hand_pose_stamped.pose.orientation.z = 0;
//     r_end_hand_pose_stamped.pose.orientation.w = 1; // No rotation
    
    
    
    
    trajectory_utils::Cartesian end;
    //end.distal_frame = "LSoftHand";
    //end.frame = end_hand_pose_stamped;
    end.distal_frame = "RSoftHand";
    //end.frame = r_end_hand_pose_stamped;
    end.frame = *shared_data().rh_grasp_pose; // rh_grasp_pose is a pointer --> need *

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 1.0;        // traj duration 1 second      
    s1.start = start_traj;   // start pose
    s1.end = end;            // end pose 

    
//     start_traj.frame = end_hand_pose_stamped;
// 
//     end_hand_pose_stamped.pose.position.z =
//     shared_data()._hose_grasp_pose->pose.position.z;
//     end.frame = end_hand_pose_stamped;
//     
//     end_hand_pose_stamped.pose.position.z = 0.1;
//     end.frame = end_hand_pose_stamped;

//     // define the second segment
//     trajectory_utils::segment s2;
//     s2.type.data = 0;        // min jerk traj
//     s2.T.data = 10.0;        // traj duration 1 second      
//     s2.start = start_traj;   // start pose
//     s2.end = end;            // end pose 

    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back (s1);
    //segments.push_back (s2);

    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = shared_data ().frame_id_;
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;

    // call the service
    shared_data()._client.call(srv);

//     // save last hand pose
//     shared_data()._last_lh_pose =
//     boost::shared_ptr<geometry_msgs::PoseStamped>
// 	(new geometry_msgs::PoseStamped (end_hand_pose_stamped));
//     shared_data()._last_rh_pose =
//     boost::shared_ptr<geometry_msgs::PoseStamped>
// 	(new geometry_msgs::PoseStamped (r_end_hand_pose_stamped));
	
    // Info msg
    //std::cout << "Send \"lh_move_fail\" or \"success\" msg..." << std::endl;
    std::cout << "Leaving Move_RH_Entry ..." << std::endl;




}

void myfsm::Move_RH::run (double time, double period)
{
    std::cout << "Move_RH run" << std::endl;
  
  
    //transit("Grasp_RH");
    
    
    
  
    // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // RH Move failed
    if (!shared_data().current_command.str().compare("finish"))
	transit("Home");

    // RH Move Succeeded
    if (!shared_data().current_command.str().compare("success"))
	transit("Grasp_RH");
    }
  
}

void myfsm::Move_RH::exit ()
{

}

// End Move_RH State


// Begin Grasp_RH State
void myfsm::Grasp_RH::react (const XBot::FSM::Event& e)
{

}

void myfsm::Grasp_RH::entry(const XBot::FSM::Message& msg)
{

}

void myfsm::Grasp_RH::run(double time, double period)
{
    std::cout << "Grasp_RH run" << std::endl;

    //TBD: check if the move has failed
    bool move_rh_fail = false;

    if (move_rh_fail)
    transit("Move_Fail");

    //TBD: Grasp with RH

    transit("Grasp_RH_Done");
}

void myfsm::Grasp_RH::exit ()
{

}

// End Grasp_RH State


// Begin Grasp_RH_Done
void myfsm::Grasp_RH_Done::react (const XBot::FSM::Event& e)
{

}

void myfsm::Grasp_RH_Done::entry (const XBot::FSM::Message& msg)
{

}

void myfsm::Grasp_RH_Done::run (double time, double period)
{
    std::cout << "Grasp_RH_Done run" << std::endl;

    //TBD: Check if RH_Grasp or the Move_RH has failed
    bool grasp_rh_fail = false;
    bool move_rh_fail = false;

    if (grasp_rh_fail)
    transit("Grasp_Fail");

    if (move_rh_fail)
    transit("Move_Fail");

    //TBD: Wait for RH_Pose (orientation + fixed position displayment)

    transit("Orient_RH");
}

void myfsm::Grasp_RH_Done::exit ()
{

}
// End Grasp_RH_Done