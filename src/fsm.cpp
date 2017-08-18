#include "fsm.h"
#include <eigen_conversions/eigen_msg.h>
#include <string>


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
//      // Move RH to RH_Pose (1 mid point in the z-axis fixed dist)
// 
//     std::cout << "Move_RH entry" << std::endl;
// 
//     
//     
//     // sense and sync model
//     shared_data()._robot->sense();
// 
//     // define in the init of your plugin and put the client in the shared data struct
//     //ros::ServiceClient client =
//     //  shared_data()._nh->serviceClient<ADVR_ROS::advr_segment_control>("segment_control");
// 
//     // Get currnt hand 
//     Eigen::Affine3d pose;
//     geometry_msgs::Pose start_frame_pose;
//     
//     //std::string pose_frame_id = "/multisense/left_camera_optical_frame";
//     std::string pose_frame_id = "Waist";
// 
//     //shared_data()._robot->model().getPose("RSoftHand", "Waist", pose);
//     shared_data()._robot->model().getPose("RSoftHand", pose_frame_id, pose);
//     shared_data()._robot->model().getPose("RSoftHand", pose);
//     tf::poseEigenToMsg (pose, start_frame_pose);
// 
//     // define the start frame 
//     geometry_msgs::PoseStamped start_frame;
//     start_frame.pose = start_frame_pose;
// 
//     // define the end frame
//     geometry_msgs::PoseStamped end_frame;
//     end_frame.pose = start_frame_pose;
// 
// //     // from message
// //     end_frame.pose.position.x = shared_data()._hose_grasp_pose->pose.position.x;
// //     end_frame.pose.position.y = shared_data()._hose_grasp_pose->pose.position.y;
// //     end_frame.pose.position.z = shared_data()._hose_grasp_pose->pose.position.z-0.1;
// // 
// //     end_frame.pose.orientation.x = shared_data()._hose_grasp_pose->pose.orientation.x;
// //     end_frame.pose.orientation.y = shared_data()._hose_grasp_pose->pose.orientation.y;
// //     end_frame.pose.orientation.z = shared_data()._hose_grasp_pose->pose.orientation.z;
// //     end_frame.pose.orientation.w = shared_data()._hose_grasp_pose->pose.orientation.w;
// 
//     // hard code
//     end_frame.pose.position.x = 0.306;
//     end_frame.pose.position.y = -0.393;
//     end_frame.pose.position.z = -0.069;    
//     
//     end_frame.pose.orientation.x = -0.068;
//     end_frame.pose.orientation.y = -0.534;
//     end_frame.pose.orientation.z = 0.067;
//     end_frame.pose.orientation.w = 0.840;  
// 
//     // trajectory
//     trajectory_utils::Cartesian start;
//     start.distal_frame = "RSoftHand";
//     start.frame = start_frame;
// 
//     trajectory_utils::Cartesian end;
//     end.distal_frame = "RSoftHand";
//     end.frame = end_frame;
// 
// 
//     // define the first segment
//     trajectory_utils::segment s1;
//     s1.type.data = 0;        // min jerk traj
//     s1.T.data = 5.0;         // traj duration 5 second      
//     s1.start = start;        // start pose
//     s1.end = end;            // end pose 
// 
// //     start.frame = end_frame;
// // 
// //     end_frame.pose.position.x = shared_data()._hose_grasp_pose->pose.position.x;
// //     end_frame.pose.position.y = shared_data()._hose_grasp_pose->pose.position.y;
// //     end_frame.pose.position.z = shared_data()._hose_grasp_pose->pose.position.z;
// // 
// //     end_frame.pose.orientation.x = shared_data()._hose_grasp_pose->pose.orientation.x;
// //     end_frame.pose.orientation.y = shared_data()._hose_grasp_pose->pose.orientation.y;
// //     end_frame.pose.orientation.z = shared_data()._hose_grasp_pose->pose.orientation.z;
// //     end_frame.pose.orientation.w = shared_data()._hose_grasp_pose->pose.orientation.w;
// //     
// // 
// //     end.frame = end_frame;
// 
// //     // define the first segment
// //     trajectory_utils::segment s2;
// //     s2.type.data = 0;        // min jerk traj
// //     s2.T.data = 5.0;         // traj duration 5 second      
// //     s2.start = start;        // start pose
// //     s2.end = end;            // end pose 
// 
//     // only one segment in this example
//     std::vector<trajectory_utils::segment> segments;
//     segments.push_back (s1);
//     //segments.push_back (s2);
// 
//     // prapere the advr_segment_control
//     ADVR_ROS::advr_segment_control srv;
//     //srv.request.segment_trj.header.frame_id = "Waist";
//     srv.request.segment_trj.header.frame_id = "world";
//     srv.request.segment_trj.header.stamp = ros::Time::now();
//     srv.request.segment_trj.segments = segments;
// 
//     // call the service
//     shared_data()._client.call(srv);
    
    
//     //////////////////////////////////////////////////////////////////////////////////
//       shared_data()._robot->sense();    
//     
//     // RIGHT HAND
//     
//     Eigen::Affine3d poseRightHand;
//     geometry_msgs::Pose start_frame_pose;
// 
//     shared_data()._robot->model().getPose("RSoftHand", "Waist", poseRightHand);
//     tf::poseEigenToMsg (poseRightHand, start_frame_pose);
// 
//     // define the start frame 
//     geometry_msgs::PoseStamped start_frame;
//     start_frame.pose = start_frame_pose;
//     
//     trajectory_utils::Cartesian start;
//     start.distal_frame = "RSoftHand";
//     start.frame = start_frame;
//     
//     // define the end frame - RIGHT HAND
//     geometry_msgs::PoseStamped end_frame;
//     
//     end_frame.pose = start_frame_pose;    
// 
//     end_frame.pose.position.x = 0.659;
//     end_frame.pose.position.y = -0.29;
//     end_frame.pose.position.z = -0.074;    
//     
//     end_frame.pose.orientation.x = 0.0;
//     end_frame.pose.orientation.y = -0.7071070192004544;
//     end_frame.pose.orientation.z = 0.0;
//     end_frame.pose.orientation.w = 0.7071070192004544;      
//     
// //     end_frame.pose.position.x = 0.841;
// //     end_frame.pose.position.y = 0.051;
// //     end_frame.pose.position.z = 0.134;        
// //     
// //     end_frame.pose.orientation.x = -0.110;
// //     end_frame.pose.orientation.y = 0.739;
// //     end_frame.pose.orientation.z = -0.481;
// //     end_frame.pose.orientation.w = -0.459;  
//     
//     trajectory_utils::Cartesian end;
//     end.distal_frame = "RSoftHand";
//     end.frame = end_frame;
// 
//     // define the first segment
//     trajectory_utils::segment s1;
//     s1.type.data = 0;        // min jerk traj
//     s1.T.data = 15.0;         // traj duration 5 second      
//     s1.start = start;        // start pose
//     s1.end = end;            // end pose 
//     
//     // only one segment in this example
//     std::vector<trajectory_utils::segment> segments;
//     segments.push_back(s1);
//     
//     // prapere the advr_segment_control
//     ADVR_ROS::advr_segment_control srv;
//     srv.request.segment_trj.header.frame_id = "Waist";
//     srv.request.segment_trj.header.stamp = ros::Time::now();
//     srv.request.segment_trj.segments = segments;
//     
//     // call the service
//     shared_data()._client.call(srv);
    
    
    
    ///////////////////////////////////
     // Info messages
    std::cout << "State: Move_LH entry" << std::endl;
//     std::cout << "Pub /hose_pose for left hand..." << std::endl;

//     // Wait for RH_Pose, i.e. the Hose Grasp Pose (hose_grasp_pose)
//     shared_data ()._hose_grasp_pose =
//     ros::topic::waitForMessage<geometry_msgs::PoseStamped>
// 	(shared_data ().pose_cmd_);

    // Move LH to LH_Pose (1 mid point in the z-axis fixed dist)

    // Only for first time sense and sync to the model
    //TBD: put flag to check if it is first run to retrieve hands pose
    shared_data()._robot->sense();
    Eigen::Affine3d world_T_bl;
    std::string fb;
    shared_data()._robot->model().getFloatingBaseLink(fb);
    tf.getTransformTf(fb, shared_data ().frame_id_, world_T_bl);
    shared_data()._robot->model().setFloatingBasePose(world_T_bl);
    shared_data()._robot->model().update();

    // Get current hand
    KDL::Frame hand_pose_KDL;
    Eigen::Affine3d hand_pose, r_hand_pose;
    geometry_msgs::Pose start_hand_pose, r_start_hand_pose;

    // Get hands poses
    shared_data()._robot->model().getPose("LSoftHand", hand_pose);
    shared_data().sl_hand_pose = hand_pose;
    shared_data()._robot->model().getPose("RSoftHand", r_hand_pose);
    shared_data().sr_hand_pose = r_hand_pose;

    // Transform from Eigen::Affine3d to geometry_msgs::Pose
    tf::poseEigenToMsg (hand_pose, start_hand_pose);
    tf::poseEigenToMsg (r_hand_pose, r_start_hand_pose);

    // Define the start frame as geometry_msgs::PoseStamped
    geometry_msgs::PoseStamped start_hand_pose_stamped;
    start_hand_pose_stamped.pose = start_hand_pose;
    geometry_msgs::PoseStamped r_start_hand_pose_stamped;
    r_start_hand_pose_stamped.pose = r_start_hand_pose;

    // Create the Cartesian trajectories
    trajectory_utils::Cartesian start_traj;
    start_traj.distal_frame = "LSoftHand";
    start_traj.frame = start_hand_pose_stamped;

    // define the end frame
    geometry_msgs::PoseStamped end_hand_pose_stamped;
    end_hand_pose_stamped.pose = start_hand_pose;

    geometry_msgs::PoseStamped r_end_hand_pose_stamped;
    r_end_hand_pose_stamped.pose = r_start_hand_pose;

    //end_hand_pose_stamped.pose.position = shared_data()._hose_grasp_pose->pose.position;
    
    end_hand_pose_stamped.pose.position.x = 0.5;
    end_hand_pose_stamped.pose.position.y = 0.1;
    end_hand_pose_stamped.pose.position.z = 0.2;

    //end_hand_pose_stamped.pose.orientation =
    //  shared_data()._hose_grasp_pose->pose.orientation;

    end_hand_pose_stamped.pose.orientation.x =  0.094453573229;
    end_hand_pose_stamped.pose.orientation.y = -0.324954947790;
    end_hand_pose_stamped.pose.orientation.z = -0.601136949013;
    end_hand_pose_stamped.pose.orientation.w =  0.723959372439;

    trajectory_utils::Cartesian end;
    end.distal_frame = "LSoftHand";
    end.frame = end_hand_pose_stamped;

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 10.0;        // traj duration 1 second      
    s1.start = start_traj;   // start pose
    s1.end = end;            // end pose 

    start_traj.frame = end_hand_pose_stamped;

    end_hand_pose_stamped.pose.position.z =
    shared_data()._hose_grasp_pose->pose.position.z;
    end.frame = end_hand_pose_stamped;

    // define the first segment
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = 10.0;        // traj duration 1 second      
    s2.start = start_traj;   // start pose
    s2.end = end;            // end pose 

    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back (s1);
    segments.push_back (s2);

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
    std::cout << "Send message done!" << std::endl;




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