#include "fsm_definition.h"


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

    // Wait for RH_Pose, i.e. the Hose Grasp Pose (hose_grasp_pose)
    shared_data()._hose_grasp_pose =
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("hose_grasp_pose");

    // Debug msg
    std::cout << "Got pose message: " << std::endl;
    std::cout << shared_data()._hose_grasp_pose->pose.position.x << std::endl;

    transit("Move_RH");
}

void myfsm::Home::exit ()
{

}
//End Home State




// Begin Move_RH State
void myfsm::Move_RH::react (const XBot::FSM::Event& e)
{

}

void myfsm::Move_RH::entry (const XBot::FSM::Message& msg)
{
    // Move RH to RH_Pose (1 mid point in the z-axis fixed dist)

    std::cout << "Move_RH run" << std::endl;

    // sense and sync model
    shared_data()._robot->sense();

    // define in the init of your plugin and put the client in the shared data struct
    //ros::ServiceClient client =
    //  shared_data()._nh->serviceClient<ADVR_ROS::advr_segment_control>("segment_control");

    std::cout << "Move_RH run 2" << std::endl;

    // Get currnt hand 
    Eigen::Affine3d pose;
    geometry_msgs::Pose start_frame_pose;

    shared_data()._robot->model().getPose("RSoftHand", "Waist", pose);
    //shared_data()._robot->model().getPose("LSoftHand", pose);
    tf::poseEigenToMsg (pose, start_frame_pose);

    // define the start frame 
    geometry_msgs::PoseStamped start_frame;
    start_frame.pose = start_frame_pose;

    // define the end frame
    geometry_msgs::PoseStamped end_frame;
    end_frame.pose = start_frame_pose;
    //end_frame.pose.position.y += 0.3;
    //end_frame.pose.position.z += 0.3;
    end_frame.pose.position.x = shared_data()._hose_grasp_pose->pose.position.x;
    end_frame.pose.position.y = shared_data()._hose_grasp_pose->pose.position.y;
    end_frame.pose.position.z = shared_data()._hose_grasp_pose->pose.position.z;

    //end_frame.pose.orientation.x = shared_data()._hose_grasp_pose->pose.orientation.x;
    //end_frame.pose.orientation.y = shared_data()._hose_grasp_pose->pose.orientation.y;
    //end_frame.pose.orientation.z = shared_data()._hose_grasp_pose->pose.orientation.z;
    //end_frame.pose.orientation.w = shared_data()._hose_grasp_pose->pose.orientation.w;

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

    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back (s1);

    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "Waist";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;

    std::cout <<"before" <<std::endl;
    // call the service
    shared_data()._client.call(srv);
    std::cout <<"aftr" <<std::endl;

}

void myfsm::Move_RH::run (double time, double period)
{
  std::cout << "Move_RH run" << std::endl;
  
  //TBD: Check if the RH has reached the hose_grasp_pose
  
  //transit("Grasp_RH");
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