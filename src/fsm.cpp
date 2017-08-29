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
    std::cout << shared_data().str_seperator << std::endl;
    std::cout << "State: HOME ENTRY" << std::endl;
    
}

void myfsm::Home::run (double time, double period)
{
    std::cout << "========================================================================" << std::endl;
    std::cout << "State: HOME RUN" << std::endl;
    std::cout << "Current hand: " << shared_data().current_hand << std::endl;
    std::cout << "Current grasp strategy: " << shared_data().current_grasp_strategy << std::endl;
    std::cout << "You can change hand (rh or lh) or grasp stragegy (topgrasp or sidegrasp)" << std::endl;
    
//     // blocking reading: wait for a command
//     if(shared_data().command.read(shared_data().current_command))
//     {
// // 	std::string str_cmd = shared_data().current_command.str();
// // 	std::cout << "Received command: " << str_cmd << std::endl;
// // 	if (str_cmd.compare(shared_data().rh_id) == 0)
// // 	{
// // 	    shared_data().current_hand = shared_data().rh_id;
// // 	    std::cout << "Updated current hand to: " << shared_data().current_hand;
// // 	}
// // 	
// // 	if (str_cmd.compare(shared_data().lh_id) == 0)
// // 	{
// // 	    shared_data().current_hand = shared_data().lh_id;
// // 	    std::cout << "Updated current hand to: " << shared_data().current_hand;
// // 	}
// 
// 	// TODO: update grasp strategy
// 	
// 	// RH Move failed
// 	if (!shared_data().current_command.str().compare("reach"))
// 	    //transit("Idle");
// 	    transit("Reach");
//     }
    
     // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
	std::string str_cmd = shared_data().current_command.str();
	std::cout << "Received command: " << str_cmd << std::endl;
	
	// TBD
	if (!str_cmd.compare("reach"))
	    transit("Reach");
	
	if (!str_cmd.compare("idle"))
	    transit("Idle");
	
	if (!str_cmd.compare("detect"))
	    transit("Detect");
	
	if (!str_cmd.compare("grasp"))
	    transit("Grasp");
	
	if (!str_cmd.compare("ungrasp"))
	    transit("Ungrasp");
	
	if (!str_cmd.compare("move"))
	    transit("Move");
	
    }
    
}

void myfsm::Home::exit ()
{

}
//End Home State



//Begin Detect State
void myfsm::Detect::react (const XBot::FSM::Event& e)
{
    std::cout << "Detect react" << std::endl;
}

void myfsm::Detect::entry (const XBot::FSM::Message& msg)
{
    std::cout << shared_data().str_seperator << std::endl;
    std::cout << "State: DETECT ENTRY" << std::endl;
   
    /////////////////////////////////////////////////////////////////////////////////////////
    // Update robot model
    shared_data()._robot->sense();
    Eigen::Affine3d world_T_bl;
    std::string fb;
    shared_data()._robot->model().getFloatingBaseLink(fb);
    tf.getTransformTf(fb, shared_data ().world_frame, world_T_bl);
    shared_data()._robot->model().setFloatingBasePose(world_T_bl);
    shared_data()._robot->model().update();
    if (shared_data().verbose_print) std::cout << "Update robot model done!" << std::endl;
    

    shared_data()._robot->model().getPose("RSoftHand", shared_data().ei_start_rh_pose);
    if (shared_data().verbose_print) std::cout << "Get starting pose from RIGHT HAND ok!" << std::endl;
    
    shared_data()._robot->model().getPose("LSoftHand", shared_data().ei_start_lh_pose);
    if (shared_data().verbose_print) std::cout << "Get starting pose from LEFT HAND ok!" << std::endl;
    
    
    // Convert start hand pose (eigen) to geometry msg (pose)
    geometry_msgs::Pose geo_pose_start_rh_pose;
    tf::poseEigenToMsg (shared_data().ei_start_rh_pose, geo_pose_start_rh_pose);
    
    // Convert geometry msg (pose) to geometry msg (pose stamped)
    geometry_msgs::PoseStamped geo_posestamped_start_rh_pose;
    geo_posestamped_start_rh_pose.pose = geo_pose_start_rh_pose;
    
    geo_posestamped_start_rh_pose.header.frame_id = shared_data().world_frame; // MUST SET THE HEADER TO world_frame - getPose() return in world frame
    
    // update last rh pose
    shared_data().pst_last_rh_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(geo_posestamped_start_rh_pose));
    // publish last rh pose
    shared_data()._pub_rb_last_rh_pose.publish(geo_posestamped_start_rh_pose);
    std::cout << "Publishing the last right hand pose ..." << std::endl;
    
    //TODO: Left Hand
    

    /////////////////////////////////////////////////////////////////////////////////////////
    
    
    
    //// define the end frame - from vision module
    // wait for vision message
    if (shared_data().current_hand == shared_data().rh_id)
    {
	//shared_data ().grasp_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(shared_data ().vs_rh_obj_pose_3D);
	shared_data ().grasp_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(shared_data ().vs_rh_obj_pose_3D_FAKE); // got fake pose - to debug
	if (shared_data().verbose_print) std::cout << "GOT MESSAGE FROM vs_rh_grasp_topic_3D" << std::endl;
    }
    else
	shared_data ().grasp_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(shared_data ().vs_lh_obj_pose_3D);
    
    // convert object pose_stamped --> pose --> eigen
    tf::Transform tf_cam_to_obj;
    geometry_msgs::Pose temp_pose;
    temp_pose.position = shared_data().grasp_pose->pose.position;
    temp_pose.orientation = shared_data().grasp_pose->pose.orientation;
    tf::poseMsgToTF(temp_pose, tf_cam_to_obj);
    geometry_msgs::Transform geo_trs_cam_to_obj;
    tf::transformTFToMsg(tf_cam_to_obj, geo_trs_cam_to_obj);
    Eigen::Affine3d eigen_cam_to_obj;
    tf::transformMsgToEigen(geo_trs_cam_to_obj, eigen_cam_to_obj);
    
    // get transformation from world to cam
    Eigen::Affine3d eigen_world_to_cam;
    //tf.getTransformTf("world_odom", "multisense/left_camera_optical_frame", world_to_cam); // wrong order
    //tf.getTransformTf("multisense/left_camera_optical_frame", "world_odom", world_to_cam); // ok
    tf.getTransformTf(shared_data().left_camera_frame, shared_data().world_frame, eigen_world_to_cam);
    
    // get final world to object transform
    Eigen::Affine3d eigen_world_to_object;
    eigen_world_to_object = eigen_world_to_cam * eigen_cam_to_obj;
    
    // convert eign to msg pose
    geometry_msgs::Pose geo_pose_end_grasp_pose;
    tf::poseEigenToMsg (eigen_world_to_object, geo_pose_end_grasp_pose);      
    
    // keep the position only
    geometry_msgs::PoseStamped geo_posestamped_grasp_pose;
    geo_posestamped_grasp_pose.pose.position = geo_pose_end_grasp_pose.position;
    std::cout << "--- POSITION 1: " << geo_posestamped_grasp_pose.pose.position.x << " " 
				    << geo_posestamped_grasp_pose.pose.position.y << " " 
				    << geo_posestamped_grasp_pose.pose.position.z << std::endl;

//     geo_posestamped_grasp_pose.pose.orientation.x = 0;
//     geo_posestamped_grasp_pose.pose.orientation.y = -0.5591931143131625;
//     geo_posestamped_grasp_pose.pose.orientation.z = 0;
//     geo_posestamped_grasp_pose.pose.orientation.w = 0.8290374303399975;  // first hard code pose
				    
//     geo_posestamped_grasp_pose.pose.orientation.x = 0;
//     geo_posestamped_grasp_pose.pose.orientation.y = 0;
//     geo_posestamped_grasp_pose.pose.orientation.z = 0;
//     geo_posestamped_grasp_pose.pose.orientation.w = 1; // No rotation - should be the same with world frame
				    
    geo_posestamped_grasp_pose.pose.orientation.x = 0;
    geo_posestamped_grasp_pose.pose.orientation.y = -0.7071;  // rotate 270 along y axis - "sidegrasp" pose
    geo_posestamped_grasp_pose.pose.orientation.z = 0;
    geo_posestamped_grasp_pose.pose.orientation.w = 0.7071; 

    
    
    //temp_pose_stamp.header.frame_id = "world_odom";
    geo_posestamped_grasp_pose.header.frame_id = shared_data().world_frame;
    // need to cast the variable as the pointer --> reverse later!!!
    shared_data().grasp_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(geo_posestamped_grasp_pose));
     
    // publish obj pose message in world frame
    if (shared_data().current_hand == shared_data().rh_id)
	shared_data()._pub_rb_rh_grasp_pose.publish(geo_posestamped_grasp_pose);
    else
	shared_data()._pub_rb_lh_grasp_pose.publish(geo_posestamped_grasp_pose);
    
    
    // find PREGRASP pose ----------------------------------------------------------
    geometry_msgs::PoseStamped geo_posestamped_pregrasp_pose;
    geo_posestamped_pregrasp_pose = *shared_data().grasp_pose; // same location, same orientation
    geo_posestamped_pregrasp_pose.pose.position.x -= 0.1;  // (x from the robot to further)  - close to the robot
    geo_posestamped_pregrasp_pose.pose.position.y -= 0.1;  // (y from right to left) - far to the right
    geo_posestamped_pregrasp_pose.pose.position.z += 0.0;  // (z is up) no change --- BASE ON THE ORIGINAL world_frame (in middle of two feet)
    shared_data().pregrasp_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(geo_posestamped_pregrasp_pose)); // construct the pregrasp pose
    // publish obj pregrasp_pose message in world frame
    if (shared_data().current_hand == shared_data().rh_id)
    {
	shared_data()._pub_rb_rh_pregrasp_pose.publish(geo_posestamped_pregrasp_pose);
	if (shared_data().verbose_print)
	    std::cout << "Publishing rh_pregrasp_pose pose" << std::endl;
    }
    else
	shared_data()._pub_rb_lh_pregrasp_pose.publish(geo_posestamped_pregrasp_pose);
    
    
    // find RAISE pose ----------------------------------------------------------
    geometry_msgs::PoseStamped geo_posestamped_raise_pose;
    geo_posestamped_raise_pose = *shared_data().grasp_pose; // same location, same orientation
    geo_posestamped_raise_pose.pose.position.x -= 0.1;  // (x from the robot to further) - close to the robot
    geo_posestamped_raise_pose.pose.position.y -= 0.1;  // (y from right to left) - far to the right
    geo_posestamped_raise_pose.pose.position.z += 0.1;  // (z is up) --- BASE ON THE ORIGINAL world_frame (in middle of two feet)
    shared_data().raise_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(geo_posestamped_raise_pose)); // construct the pregrasp pose
    // publish obj pregrasp_pose message in world frame
    if (shared_data().current_hand == shared_data().rh_id)
    {
	shared_data()._pub_rb_rh_raise_pose.publish(geo_posestamped_raise_pose);
	if (shared_data().verbose_print)
	    std::cout << "Publishing rh_raise_pose" << std::endl;
    }
    else
	shared_data()._pub_rb_rh_raise_pose.publish(geo_posestamped_raise_pose); // CHANGE to left hand LATER !!!!!!!
	
	
    /////////////////////////////////////////////////////////////
    // GET CURRENT RIGHT HAND POSE AND Transform --> TO HAVE GOOD FAKE TEST POSE
    
	
    
}

void myfsm::Detect::run (double time, double period)
{
    std::cout << shared_data().str_seperator << std::endl;
    std::cout << "State: DETECT RUN" << std::endl;
    
    // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
	std::string str_cmd = shared_data().current_command.str();
	std::cout << "Received command: " << str_cmd << std::endl;
	
	if (!str_cmd.compare("prereach"))
	    transit("Prereach");
	
	if (!str_cmd.compare("move"))
	    transit("Move");
	
	if (!str_cmd.compare("home"))
	    transit("Home");
    }
    
}

void myfsm::Detect::exit ()
{

}
//End Detect State




//Begin Prereach State
void myfsm::Prereach::react (const XBot::FSM::Event& e)
{
    std::cout << "Prereach react" << std::endl;
}

void myfsm::Prereach::entry (const XBot::FSM::Message& msg)
{
    
    // =====================================================================================
    // Create the Cartesian trajectories - starting ...
    trajectory_utils::Cartesian start_traj;
    start_traj.distal_frame = "RSoftHand";
    start_traj.frame = *shared_data().pst_last_rh_pose;
    
    
    // Create the Cartesian trajectories - ending ...
    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
    //end.frame = r_end_hand_pose_stamped;
    end.frame = *shared_data().pregrasp_pose; // to test hardcode pose; _grasp_pose is a pointer --> need *

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 5.0;         // traj duration 1 second      
    s1.start = start_traj;   // start pose
    s1.end = end;            // end pose 

    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back (s1);
   

    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = shared_data ().world_frame;
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;

    // call the service
    shared_data()._client.call(srv);
    
    // update the last right hand pose to the pregrasp pose
    shared_data().pst_last_rh_pose = shared_data().pregrasp_pose;
    shared_data()._pub_rb_last_rh_pose.publish(*shared_data().pst_last_rh_pose);
    
}

void myfsm::Prereach::run (double time, double period)
{
    std::cout << shared_data().str_seperator << std::endl;
    std::cout << "State: PREREACH RUN" << std::endl;
    
    // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
	std::string str_cmd = shared_data().current_command.str();
	std::cout << "Received command: " << str_cmd << std::endl;
	
	if (!str_cmd.compare("reach"))
	    transit("Reach");
	
	if (!str_cmd.compare("home"))
	    transit("Home");
    }
    
}

void myfsm::Prereach::exit ()
{

}
//End Prereach State




void myfsm::Reach::entry(const XBot::FSM::Message& msg)
{
    std::cout << shared_data().str_seperator << std::endl;
    std::cout << "State: REACH ENTRY" << std::endl;
    
//     // Update robot model
//     shared_data()._robot->sense();
//     Eigen::Affine3d world_T_bl;
//     std::string fb;
//     shared_data()._robot->model().getFloatingBaseLink(fb);
//     tf.getTransformTf(fb, shared_data ().world_frame, world_T_bl);
//     shared_data()._robot->model().setFloatingBasePose(world_T_bl);
//     shared_data()._robot->model().update();
//     if (shared_data().verbose_print) std::cout << "Update robot model done!" << std::endl;
//     
//     // Get starting hand pose
//     if (shared_data().current_hand == shared_data().rh_id)
//     {
// 	shared_data()._robot->model().getPose("RSoftHand", shared_data().start_hand_pose);
// 	if (shared_data().verbose_print) std::cout << "Get starting pose from RIGHT HAND ok!" << std::endl;
//     }
//     else
//     {
// 	shared_data()._robot->model().getPose("LSoftHand", shared_data().start_hand_pose);
// 	if (shared_data().verbose_print) std::cout << "Get starting pose from LEFT HAND ok!" << std::endl;
//     }
//     
//     
//     // Convert start hand pose (eigen) to geometry msg (pose)
//     geometry_msgs::Pose geo_pose_start_hand_pose;
//     tf::poseEigenToMsg (shared_data().start_hand_pose, geo_pose_start_hand_pose);
//     
//     // Convert geometry msg (pose) to geometry msg (pose stamped)
//     geometry_msgs::PoseStamped geo_posestamped_start_hand_pose;
//     geo_posestamped_start_hand_pose.pose = geo_pose_start_hand_pose;

    
// //     //// define the end frame - HARD CODE
// //     geometry_msgs::PoseStamped r_end_hand_pose_stamped;
// //     r_end_hand_pose_stamped.pose.position.x = 0.619;  // position and orientation are in world frame
// //     r_end_hand_pose_stamped.pose.position.y = -0.29;
// //     r_end_hand_pose_stamped.pose.position.z = 0.873;
// //     r_end_hand_pose_stamped.pose.orientation.x = 0;
// //     r_end_hand_pose_stamped.pose.orientation.y = -0.5591931143131625;
// //     r_end_hand_pose_stamped.pose.orientation.z = 0;
// //     r_end_hand_pose_stamped.pose.orientation.w = 0.8290374303399975;
// //     // need to cast the variable as the pointer --> reverse later!!!
// //     shared_data().rh_grasp_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(r_end_hand_pose_stamped));
//     
//     //// define the end frame - from vision module
//     // wait for vision message
//     if (shared_data().current_hand == shared_data().rh_id)
//     {
// 	shared_data ().grasp_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(shared_data ().vs_rh_obj_pose_3D);
// 	//shared_data ().grasp_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(shared_data ().vs_rh_obj_pose_3D_FAKE); // got fake pose - to debug
// 	std::cout << "GOT MESSAGE FROM vs_rh_grasp_topic_3D" << std::endl;
//     }
//     else
// 	shared_data ().grasp_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(shared_data ().vs_lh_obj_pose_3D);
//     
//     // convert object pose_stamped --> pose --> eigen
//     tf::Transform tf_cam_to_obj;
//     geometry_msgs::Pose temp_pose;
//     temp_pose.position = shared_data().grasp_pose->pose.position;
//     temp_pose.orientation = shared_data().grasp_pose->pose.orientation;
//     tf::poseMsgToTF(temp_pose, tf_cam_to_obj);
//     geometry_msgs::Transform geo_trs_cam_to_obj;
//     tf::transformTFToMsg(tf_cam_to_obj, geo_trs_cam_to_obj);
//     Eigen::Affine3d eigen_cam_to_obj;
//     tf::transformMsgToEigen(geo_trs_cam_to_obj, eigen_cam_to_obj);
//     
//     // get transformation from world to cam
//     Eigen::Affine3d eigen_world_to_cam;
//     //tf.getTransformTf("world_odom", "multisense/left_camera_optical_frame", world_to_cam); // wrong order
//     //tf.getTransformTf("multisense/left_camera_optical_frame", "world_odom", world_to_cam); // ok
//     tf.getTransformTf(shared_data().left_camera_frame, shared_data().world_frame, eigen_world_to_cam);
//     
//     // get final world to object transform
//     Eigen::Affine3d eigen_world_to_object;
//     eigen_world_to_object = eigen_world_to_cam * eigen_cam_to_obj;
//     
//     // convert eign to msg pose
//     geometry_msgs::Pose geo_pose_end_grasp_pose;
//     tf::poseEigenToMsg (eigen_world_to_object, geo_pose_end_grasp_pose);      
//     
//     // keep the position only
//     geometry_msgs::PoseStamped geo_posestamped_grasp_pose;
//     geo_posestamped_grasp_pose.pose.position = geo_pose_end_grasp_pose.position;
//     std::cout << "--- POSITION 1: " << geo_posestamped_grasp_pose.pose.position.x << " " 
// 				    << geo_posestamped_grasp_pose.pose.position.y << " " 
// 				    << geo_posestamped_grasp_pose.pose.position.z << std::endl;
// //     geo_posestamped_grasp_pose.pose.orientation.x = 0;
// //     geo_posestamped_grasp_pose.pose.orientation.y = -0.5591931143131625;
// //     geo_posestamped_grasp_pose.pose.orientation.z = 0;
// //     geo_posestamped_grasp_pose.pose.orientation.w = 0.8290374303399975;  // first hard code pose
// 				    
// //     geo_posestamped_grasp_pose.pose.orientation.x = 0;
// //     geo_posestamped_grasp_pose.pose.orientation.y = 0;
// //     geo_posestamped_grasp_pose.pose.orientation.z = 0;
// //     geo_posestamped_grasp_pose.pose.orientation.w = 1; // No rotation - should be the same with world frame
// 				    
//     geo_posestamped_grasp_pose.pose.orientation.x = 0;
//     geo_posestamped_grasp_pose.pose.orientation.y = -0.7071;  // rotate 270 along y axis - "sidegrasp" pose
//     geo_posestamped_grasp_pose.pose.orientation.z = 0;
//     geo_posestamped_grasp_pose.pose.orientation.w = 0.7071; 
//     
//     //temp_pose_stamp.header.frame_id = "world_odom";
//     geo_posestamped_grasp_pose.header.frame_id = shared_data().world_frame;
//     // need to cast the variable as the pointer --> reverse later!!!
//     shared_data().grasp_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(geo_posestamped_grasp_pose));
//      
//     // publish obj pose message in world frame
//     if (shared_data().current_hand == shared_data().rh_id)
// 	shared_data()._pub_rb_rh_grasp_pose.publish(geo_posestamped_grasp_pose);
//     else
// 	shared_data()._pub_rb_lh_grasp_pose.publish(geo_posestamped_grasp_pose);
//     
//     // find pregrasp pose
//     geometry_msgs::PoseStamped geo_posestamped_pregrasp_pose;
//     geo_posestamped_pregrasp_pose = *shared_data().grasp_pose; // same location, same orientation
//     geo_posestamped_pregrasp_pose.pose.position.x -= 0.2;  // (x towards the robot) - close to the robot
//     geo_posestamped_pregrasp_pose.pose.position.y -= 0.2;  // (y from right to left) - far to the right
//     geo_posestamped_pregrasp_pose.pose.position.z += 0.0;  // (z is up) no change --- BASE ON THE ORIGINAL world_frame (in middle of two feet)
//     shared_data().pregrasp_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(geo_posestamped_pregrasp_pose)); // construct the pregrasp pose
//     // publish obj pregrasp_pose message in world frame
//     if (shared_data().current_hand == shared_data().rh_id)
//     {
// 	shared_data()._pub_rb_rh_pregrasp_pose.publish(geo_posestamped_pregrasp_pose);
// 	if (shared_data().verbose_print)
// 	    std::cout << "Publishing rh_pregrasp_pose pose" << std::endl;
//     }
//     else
// 	shared_data()._pub_rb_lh_pregrasp_pose.publish(geo_posestamped_pregrasp_pose);
//     
//     
    // =====================================================================================
    // Create the Cartesian trajectories - starting ...
    trajectory_utils::Cartesian start_traj;
    start_traj.distal_frame = "RSoftHand";
    start_traj.frame = *shared_data().pst_last_rh_pose; // CHECK LATER
    
    
    // Create the Cartesian trajectories - ending ...
    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
    //end.frame = r_end_hand_pose_stamped;
    end.frame = *shared_data().grasp_pose; // to test hardcode pose; rh_grasp_pose is a pointer --> need *

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 5.0;         // traj duration 1 second      
    s1.start = start_traj;   // start pose
    s1.end = end;            // end pose 

    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back (s1);

    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = shared_data ().world_frame;
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;

    // call the service
    shared_data()._client.call(srv);

    // update the last right hand pose to the grasp pose
    shared_data().pst_last_rh_pose = shared_data().grasp_pose;
    shared_data()._pub_rb_last_rh_pose.publish(*shared_data().pst_last_rh_pose);
    
//     // =====================================================================================
//     //  2 SEGMENTS
//     // Create the Cartesian trajectories - starting ...
//     trajectory_utils::Cartesian start_traj;
//     start_traj.distal_frame = "RSoftHand";
//     start_traj.frame = geo_posestamped_start_hand_pose;
// 
//     // Create the Cartesian trajectories - pregrasping ...
//     trajectory_utils::Cartesian pregrasp_traj;
//     pregrasp_traj.distal_frame = "RSoftHand";
//     pregrasp_traj.frame = *shared_data().pregrasp_pose;
//    
//     // Create the Cartesian trajectories - ending ...
//     trajectory_utils::Cartesian end_traj;
//     end_traj.distal_frame = "RSoftHand";
//     //end.frame = r_end_hand_pose_stamped;
//     end_traj.frame = *shared_data().grasp_pose; // to test hardcode pose; rh_grasp_pose is a pointer --> need *
// 
//     // define the first segment (from start pose to pregrasp pose)
//     trajectory_utils::segment s1;
//     s1.type.data = 0;        // min jerk traj
//     s1.T.data = 5.0;         // traj duration 1 second      
//     s1.start = start_traj;   // start pose
//     s1.end = pregrasp_traj;  // pregrasp pose 
// 
//     // define the second segment (from pregrasp pose to final grasp pose)
//     trajectory_utils::segment s2;
//     s2.type.data = 0;        // min jerk traj
//     s2.T.data = 5.0;         // traj duration 1 second      
//     s2.start = pregrasp_traj;   // start pose
//     s2.end = end_traj;  // pregrasp pose 
//     
//     // 2 segments
//     std::vector<trajectory_utils::segment> segments;
//     segments.push_back (s1);
//     segments.push_back (s2);
// 
//     // prapere the advr_segment_control
//     ADVR_ROS::advr_segment_control srv;
//     srv.request.segment_trj.header.frame_id = shared_data ().world_frame;
//     srv.request.segment_trj.header.stamp = ros::Time::now();
//     srv.request.segment_trj.segments = segments;
// 
//     // call the service
//     shared_data()._client.call(srv);
    
}

void myfsm::Reach::run(double time, double period)
{
    std::cout << "State: REACH RUN" << std::endl;

 // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
	std::string str_cmd = shared_data().current_command.str();
	std::cout << "Received command: " << str_cmd << std::endl;
	
	if (!str_cmd.compare("home"))
	    //transit("Idle");
	    transit("Home");
	
	if (!str_cmd.compare("grasp"))
	    transit("Grasp");
	
    }
}

void myfsm::Reach::react (const XBot::FSM::Event& e)
{

}

void myfsm::Reach::exit ()
{

}


////////////////////////////
//Begin Grasp State
void myfsm::Grasp::react (const XBot::FSM::Event& e)
{
    std::cout << "Grasp react" << std::endl;
}

void myfsm::Grasp::entry (const XBot::FSM::Message& msg)
{
    ADVR_ROS::advr_grasp_control_srv srv;
  
    srv.request.right_grasp = 0.9; // use right hand
    srv.request.left_grasp = 0.0;
    
    // call the service
    shared_data()._grasp_client.call(srv);  
    
}

void myfsm::Grasp::run (double time, double period)
{
    std::cout << shared_data().str_seperator << std::endl;
    std::cout << "State: GRASP RUN" << std::endl;
    
    // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
	std::string str_cmd = shared_data().current_command.str();
	std::cout << "Received command: " << str_cmd << std::endl;
	
	if (!str_cmd.compare("ungrasp"))
	    transit("Ungrasp");
	
	if (!str_cmd.compare("home"))
	    transit("Home");
		
	if (!str_cmd.compare("raise"))
	    transit("Raise");
    }
    
}

void myfsm::Grasp::exit ()
{

}
//End Grasp State



////////////////////////////
//Begin Ungrasp State
void myfsm::Ungrasp::react (const XBot::FSM::Event& e)
{
    std::cout << "Ungrasp react" << std::endl;
}

void myfsm::Ungrasp::entry (const XBot::FSM::Message& msg)
{
    
    ADVR_ROS::advr_grasp_control_srv srv;
  
    srv.request.right_grasp = 0.0; // use right hand
    srv.request.left_grasp = 0.0;
    
    // call the service
    shared_data()._grasp_client.call(srv);  
    
    
}

void myfsm::Ungrasp::run (double time, double period)
{
    std::cout << shared_data().str_seperator << std::endl;
    std::cout << "State: UNGRASP RUN" << std::endl;
    
    // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
	std::string str_cmd = shared_data().current_command.str();
	std::cout << "Received command: " << str_cmd << std::endl;
	
	if (!str_cmd.compare("grasp"))
	    transit("Grasp");
	
	if (!str_cmd.compare("home"))
	    transit("Home");
    }
    
}

void myfsm::Ungrasp::exit ()
{

}
//End Ungrasp State


////////////////////////////
//Begin EMPTY State
void myfsm::Raise::react (const XBot::FSM::Event& e)
{
    std::cout << "Raise react" << std::endl;
}

void myfsm::Raise::entry (const XBot::FSM::Message& msg)
{
//     // Update robot model
//     shared_data()._robot->sense();
//     Eigen::Affine3d world_T_bl;
//     std::string fb;
//     shared_data()._robot->model().getFloatingBaseLink(fb);
//     tf.getTransformTf(fb, shared_data ().world_frame, world_T_bl);
//     shared_data()._robot->model().setFloatingBasePose(world_T_bl);
//     shared_data()._robot->model().update();
//     if (shared_data().verbose_print) std::cout << "Update robot model done!" << std::endl;
//     
//     // Get starting hand pose
//     if (shared_data().current_hand == shared_data().rh_id)
//     {
// 	shared_data()._robot->model().getPose("RSoftHand", shared_data().start_hand_pose);
// 	if (shared_data().verbose_print) std::cout << "Get starting pose from RIGHT HAND ok!" << std::endl;
//     }
//     else
//     {
// 	shared_data()._robot->model().getPose("LSoftHand", shared_data().start_hand_pose);
// 	if (shared_data().verbose_print) std::cout << "Get starting pose from LEFT HAND ok!" << std::endl;
//     }
//     
//     // Convert start hand pose (eigen) to geometry msg (pose)
//     geometry_msgs::Pose geo_pose_start_hand_pose;
//     tf::poseEigenToMsg (shared_data().start_hand_pose, geo_pose_start_hand_pose);
//     
//     // Convert geometry msg (pose) to geometry msg (pose stamped)
//     geometry_msgs::PoseStamped geo_posestamped_start_hand_pose;
//     geo_posestamped_start_hand_pose.pose = geo_pose_start_hand_pose;
    
    
    // =====================================================================================
    // Create the Cartesian trajectories - starting ...
    trajectory_utils::Cartesian start_traj;
    start_traj.distal_frame = "RSoftHand";
    start_traj.frame = *shared_data().pst_last_rh_pose;
    
    
    // Create the Cartesian trajectories - ending ...
    trajectory_utils::Cartesian end;
    end.distal_frame = "RSoftHand";
    //end.frame = r_end_hand_pose_stamped;
    end.frame = *shared_data().raise_pose; // to test hardcode pose; _grasp_pose is a pointer --> need *

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 5.0;         // traj duration 1 second      
    s1.start = start_traj;   // start pose
    s1.end = end;            // end pose 

    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back (s1);
   

    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = shared_data ().world_frame;
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;

    // call the service
    shared_data()._client.call(srv);
    
    // update the last right hand pose to the pregrasp pose
    shared_data().pst_last_rh_pose = shared_data().raise_pose;
    shared_data()._pub_rb_last_rh_pose.publish(*shared_data().pst_last_rh_pose);
    
}

void myfsm::Raise::run (double time, double period)
{
    std::cout << shared_data().str_seperator << std::endl;
    std::cout << "State: RAISE RUN" << std::endl;
    
    // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
	std::string str_cmd = shared_data().current_command.str();
	std::cout << "Received command: " << str_cmd << std::endl;
	
	if (!str_cmd.compare("ungrasp"))
	    transit("Ungrasp");
	
	if (!str_cmd.compare("home"))
	    transit("Home");
    }
    
}

void myfsm::Raise::exit ()
{

}
//End Raise State


////////////////////////////
//Begin Move State
void myfsm::Move::react (const XBot::FSM::Event& e)
{
    std::cout << "MOVE react" << std::endl;
}

void myfsm::Move::entry (const XBot::FSM::Message& msg)
{
    std::cout << shared_data().str_seperator << std::endl;
    std::cout << "State: MOVE ENTRY" << std::endl;
    
    //     //  2 SEGMENTS
    // Create the Cartesian trajectories - starting ...
    trajectory_utils::Cartesian start_traj;
    start_traj.distal_frame = "RSoftHand";
    start_traj.frame = *shared_data().pst_last_rh_pose;

    // Create the Cartesian trajectories - pregrasping ...
    trajectory_utils::Cartesian pregrasp_traj;
    pregrasp_traj.distal_frame = "RSoftHand";
    pregrasp_traj.frame = *shared_data().pregrasp_pose;
   
    // Create the Cartesian trajectories - ending ...
    trajectory_utils::Cartesian end_traj;
    end_traj.distal_frame = "RSoftHand";
    //end.frame = r_end_hand_pose_stamped;
    end_traj.frame = *shared_data().grasp_pose; // to test hardcode pose; rh_grasp_pose is a pointer --> need *

    // define the first segment (from start pose to pregrasp pose)
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 5.0;         // traj duration 1 second      
    s1.start = start_traj;   // start pose
    s1.end = pregrasp_traj;  // pregrasp pose 

    // define the second segment (from pregrasp pose to final grasp pose)
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = 5.0;         // traj duration 1 second      
    s2.start = pregrasp_traj;   // start pose
    s2.end = end_traj;  // pregrasp pose 
    
    // 2 segments
    std::vector<trajectory_utils::segment> segments;
    segments.push_back (s1);
    segments.push_back (s2);

    // prapere the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = shared_data ().world_frame;
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;

    // call the service
    shared_data()._client.call(srv);
    
    // update the last right hand pose to the grasp_pose
    shared_data().pst_last_rh_pose = shared_data().grasp_pose;
    shared_data()._pub_rb_last_rh_pose.publish(*shared_data().pst_last_rh_pose);
    
}

void myfsm::Move::run (double time, double period)
{
    std::cout << shared_data().str_seperator << std::endl;
    std::cout << "State: MOVE RUN" << std::endl;
    
    // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
	std::string str_cmd = shared_data().current_command.str();
	std::cout << "Received command: " << str_cmd << std::endl;
	
	if (!str_cmd.compare("grasp"))
	    transit("Grasp");
	
	if (!str_cmd.compare("ungrasp"))
	    transit("Ungrasp");
	
	if (!str_cmd.compare("raise"))
	    transit("Raise");
	
	if (!str_cmd.compare("home"))
	    transit("Home");
    }
    
}

void myfsm::Move::exit ()
{

}
//End Move State




// ////////////////////////////
// //Begin EMPTY State
// void myfsm::EMPTY::react (const XBot::FSM::Event& e)
// {
//     std::cout << "EMPTY react" << std::endl;
// }
// 
// void myfsm::EMPTY::entry (const XBot::FSM::Message& msg)
// {
//     
//     
// }
// 
// void myfsm::EMPTY::run (double time, double period)
// {
//     std::cout << shared_data().str_seperator << std::endl;
//     std::cout << "State: DETECT RUN" << std::endl;
//     
//     // blocking reading: wait for a command
//     if(shared_data().command.read(shared_data().current_command))
//     {
// 	std::string str_cmd = shared_data().current_command.str();
// 	std::cout << "Received command: " << str_cmd << std::endl;
// 	
// 	if (!str_cmd.compare("reach"))
// 	    transit("Reach");
// 	
// 	if (!str_cmd.compare("home"))
// 	    transit("Home");
//     }
//     
// }
// 
// void myfsm::EMPTY::exit ()
// {
// 
// }
// //End EMPTY State