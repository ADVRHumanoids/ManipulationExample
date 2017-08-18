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
    // Wait for RH_Pose, i.e. the Hose Grasp Pose (hose_grasp_pose)
    shared_data()._hose_grasp_pose =
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("hose_grasp_pose");

    // Debug msg
    std::cout << "Got pose message: " << std::endl;
    std::cout << shared_data()._hose_grasp_pose->pose.position.x << std::endl;

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
     // Move RH to RH_Pose (1 mid point in the z-axis fixed dist)

    std::cout << "Move_RH entry" << std::endl;

    // sense and sync model
    shared_data()._robot->sense();

    // define in the init of your plugin and put the client in the shared data struct
    //ros::ServiceClient client =
    //  shared_data()._nh->serviceClient<ADVR_ROS::advr_segment_control>("segment_control");

    // Get currnt hand 
    Eigen::Affine3d pose;
    geometry_msgs::Pose start_frame_pose;
    
    //std::string pose_frame_id = "/multisense/left_camera_optical_frame";
    std::string pose_frame_id = "Waist";

    //shared_data()._robot->model().getPose("RSoftHand", "Waist", pose);
    shared_data()._robot->model().getPose("RSoftHand", pose_frame_id, pose);
    shared_data()._robot->model().getPose("RSoftHand", pose);
    tf::poseEigenToMsg (pose, start_frame_pose);

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame.pose = start_frame_pose;

    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    end_frame.pose = start_frame_pose;

    // from message
    end_frame.pose.position.x = shared_data()._hose_grasp_pose->pose.position.x;
    end_frame.pose.position.y = shared_data()._hose_grasp_pose->pose.position.y;
    end_frame.pose.position.z = shared_data()._hose_grasp_pose->pose.position.z-0.1;

    end_frame.pose.orientation.x = shared_data()._hose_grasp_pose->pose.orientation.x;
    end_frame.pose.orientation.y = shared_data()._hose_grasp_pose->pose.orientation.y;
    end_frame.pose.orientation.z = shared_data()._hose_grasp_pose->pose.orientation.z;
    end_frame.pose.orientation.w = shared_data()._hose_grasp_pose->pose.orientation.w;

//     end_frame.pose.position.x = 0.5;
//     end_frame.pose.position.y = -0.4;
//     end_frame.pose.position.z = -0.1;
    
    trajectory_utils::Cartesian start;
    start.distal_frame = "RSoftHand";
    start.frame = start_frame;

    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
    end.frame = end_frame;


    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 5.0;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = end;            // end pose 

//     start.frame = end_frame;
// 
//     end_frame.pose.position.x = shared_data()._hose_grasp_pose->pose.position.x;
//     end_frame.pose.position.y = shared_data()._hose_grasp_pose->pose.position.y;
//     end_frame.pose.position.z = shared_data()._hose_grasp_pose->pose.position.z;
// 
//     end_frame.pose.orientation.x = shared_data()._hose_grasp_pose->pose.orientation.x;
//     end_frame.pose.orientation.y = shared_data()._hose_grasp_pose->pose.orientation.y;
//     end_frame.pose.orientation.z = shared_data()._hose_grasp_pose->pose.orientation.z;
//     end_frame.pose.orientation.w = shared_data()._hose_grasp_pose->pose.orientation.w;
//     
// 
//     end.frame = end_frame;

//     // define the first segment
//     trajectory_utils::segment s2;
//     s2.type.data = 0;        // min jerk traj
//     s2.T.data = 5.0;         // traj duration 5 second      
//     s2.start = start;        // start pose
//     s2.end = end;            // end pose 

    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back (s1);
    //segments.push_back (s2);

    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    //srv.request.segment_trj.header.frame_id = "Waist";
    srv.request.segment_trj.header.frame_id = "world";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;

    // call the service
    shared_data()._client.call(srv);

}

void myfsm::Move_RH::run (double time, double period)
{
    //std::cout << "Move_RH run" << std::endl;
  
    //TBD: Check if the RH has reached the hose_grasp_pose
  
    //transit("Grasp_RH");
    
    
    
  
    // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // RH Move failed
    if (!shared_data().current_command.str().compare("rh_move_fail"))
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