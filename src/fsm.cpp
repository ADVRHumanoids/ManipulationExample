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
	if (!shared_data().current_command.str().compare("reach"))
	    //transit("Idle");
	    transit("Reach");
	
	if (!shared_data().current_command.str().compare("idle"))
	    transit("Idle");
    }
    
}

void myfsm::Home::exit ()
{

}
//End Home State






void myfsm::Reach::entry(const XBot::FSM::Message& msg)
{
    std::cout << "State: REACH ENTRY" << std::endl;
//     // Update robot model
//     shared_data()._robot->sense();
//     Eigen::Affine3d world_T_bl;
//     std::string fb;
//     shared_data()._robot->model().getFloatingBaseLink(fb);
//     tf.getTransformTf(fb, shared_data ().world_frame, world_T_bl);
//     shared_data()._robot->model().setFloatingBasePose(world_T_bl);
//     shared_data()._robot->model().update();
// 
//     
//     // Get starting hand pose
//     if (shared_data().current_hand == shared_data().rh_id)
// 	shared_data()._robot->model().getPose("RSoftHand", shared_data().start_hand_pose);
//     else
// 	shared_data()._robot->model().getPose("LSoftHand", shared_data().start_hand_pose);
//     
//     // Convert start hand pose (eigen) to geometry msg (pose)
//     geometry_msgs::Pose geo_pose_start_hand_pose;
//     tf::poseEigenToMsg (shared_data().start_hand_pose, geo_pose_start_hand_pose);
//     
//     // Convert geometry msg (pose) to geometry msg (pose stamped)
//     geometry_msgs::PoseStamped geo_posestamped_start_hand_pose;
//     geo_posestamped_start_hand_pose.pose = geo_pose_start_hand_pose;
// 
//     
//     
// 
//    
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
// 	shared_data ().grasp_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(shared_data ().vs_rh_grasp_topic_3D);
//     else
// 	shared_data ().grasp_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(shared_data ().vs_lh_grasp_topic_3D);
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
//     geometry_msgs::PoseStamped geo_posestamped_end_grasp_pose;
//     geo_posestamped_end_grasp_pose.pose.position = geo_pose_end_grasp_pose.position;
//     std::cout << "--- POSITION 1: " << geo_posestamped_end_grasp_pose.pose.position.x << " " 
// 				    << geo_posestamped_end_grasp_pose.pose.position.y << " " 
// 				    << geo_posestamped_end_grasp_pose.pose.position.z << std::endl;
// //     temp_pose_stamp.pose.orientation.x = 0;
// //     temp_pose_stamp.pose.orientation.y = -0.5591931143131625;
// //     temp_pose_stamp.pose.orientation.z = 0;
// //     temp_pose_stamp.pose.orientation.w = 0.8290374303399975;
//     geo_posestamped_end_grasp_pose.pose.orientation.x = 0;
//     geo_posestamped_end_grasp_pose.pose.orientation.y = 0;
//     geo_posestamped_end_grasp_pose.pose.orientation.z = 0;
//     geo_posestamped_end_grasp_pose.pose.orientation.w = 1;
//     //temp_pose_stamp.header.frame_id = "world_odom";
//     geo_posestamped_end_grasp_pose.header.frame_id = shared_data().world_frame;
//     // need to cast the variable as the pointer --> reverse later!!!
//     shared_data().grasp_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(geo_posestamped_end_grasp_pose));
//      
//     if (shared_data().current_hand == shared_data().rh_id)
// 	shared_data()._pub_rh_grasp_pose.publish(geo_posestamped_end_grasp_pose);
//     else
// 	shared_data()._pub_lh_grasp_pose.publish(geo_posestamped_end_grasp_pose);
//     
//     
//     // Create the Cartesian trajectories, set starting frame
//     trajectory_utils::Cartesian start_traj;
//     if (shared_data().current_hand == shared_data().rh_id)
// 	start_traj.distal_frame = "RSoftHand";
//     else
// 	start_traj.distal_frame = "LSoftHand";
//     start_traj.frame = geo_posestamped_start_hand_pose; // must be gsg pose stamped?
//     
//         
//     // CALL trajectory_utils
//     trajectory_utils::Cartesian end;
//     if (shared_data().current_hand == shared_data().rh_id)
// 	end.distal_frame = "RSoftHand";
//     else
// 	end.distal_frame = "LSoftHand";
//     end.frame = *shared_data().grasp_pose; // to test hardcode pose; rh_grasp_pose is a pointer --> need *
// 
//     // define the first segment
//     trajectory_utils::segment s1;
//     s1.type.data = 0;        // min jerk traj
//     s1.T.data = 5.0;        // traj duration 1 second      
//     s1.start = start_traj;   // start pose
//     s1.end = end;            // end pose 
// 
//     
// //     start_traj.frame = end_hand_pose_stamped;
// // 
// //     end_hand_pose_stamped.pose.position.z =
// //     shared_data()._hose_grasp_pose->pose.position.z;
// //     end.frame = end_hand_pose_stamped;
// //     
// //     end_hand_pose_stamped.pose.position.z = 0.1;
// //     end.frame = end_hand_pose_stamped;
// 
// //     // define the second segment
// //     trajectory_utils::segment s2;
// //     s2.type.data = 0;        // min jerk traj
// //     s2.T.data = 10.0;        // traj duration 1 second      
// //     s2.start = start_traj;   // start pose
// //     s2.end = end;            // end pose 
// 
//     // only one segment in this example
//     std::vector<trajectory_utils::segment> segments;
//     segments.push_back (s1);
//     //segments.push_back (s2);
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
	
	// TBD
	if (!shared_data().current_command.str().compare("home"))
	    //transit("Idle");
	    transit("Home");
    }
}

void myfsm::Reach::react (const XBot::FSM::Event& e)
{

}

void myfsm::Reach::exit ()
{

}


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
    std::cout << "State: Idle run ------------------------------------" << std::endl;

//     // blocking reading: wait for a command
//     //if(!shared_data().command.read(shared_data().current_command))
//     //  std::cout << shared_data().current_command.str() << std::endl;
// 
    
    
    // blocking reading: wait for a command
    if(shared_data().command.read(shared_data().current_command))
    {
	std::cout << "Command: " << shared_data().current_command.str() << std::endl;

	// RH Move failed
	if (!shared_data().current_command.str().compare("home"))
	    transit("Home");
    }
    
    
//     // Test transform
//     shared_data()._robot->sense();
//     Eigen::Affine3d world_T_bl;
//     std::string fb;
//     shared_data()._robot->model().getFloatingBaseLink(fb);
//     std::cout << "--- floating base link: " << fb << std::endl;
//     
//     tf.getTransformTf(fb, shared_data ().frame_id_, world_T_bl);
//     std::cout << "--- Affine3d world_T_bl: " << world_T_bl.matrix() << std::endl;
//     
//     shared_data()._robot->model().setFloatingBasePose(world_T_bl);
//     shared_data()._robot->model().update();
// 
//     // Get current hand
//     KDL::Frame hand_pose_KDL;
//     Eigen::Affine3d hand_pose, r_hand_pose, camera_to_world;
//     geometry_msgs::Pose start_hand_pose, r_start_hand_pose, camera_pose_msgs;
// 
//     // Get hands poses
//     //shared_data()._robot->model().getPose("LSoftHand", hand_pose);
//     //shared_data().sl_hand_pose = hand_pose;
//     shared_data()._robot->model().getPose("RSoftHand", r_hand_pose);
//     shared_data().sr_hand_pose = r_hand_pose;
//     
//     //// Get camera pose: will transform a point from camera_frame to world frame ???
//     //shared_data()._robot->model().getPose("multisense/left_camera_optical_frame", camera_to_world);
//     
// 
// 
//     // Transform from Eigen::Affine3d to geometry_msgs::Pose
//     //tf::poseEigenToMsg (hand_pose, start_hand_pose);
//     tf::poseEigenToMsg (r_hand_pose, r_start_hand_pose);
// 
//     // Define the start frame as geometry_msgs::PoseStamped
//     //geometry_msgs::PoseStamped start_hand_pose_stamped;
//     //start_hand_pose_stamped.pose = start_hand_pose;
//     geometry_msgs::PoseStamped r_start_hand_pose_stamped;
//     r_start_hand_pose_stamped.pose = r_start_hand_pose;
// 
//     // Create the Cartesian trajectories
//     trajectory_utils::Cartesian start_traj;
//     //start_traj.distal_frame = "LSoftHand";
//     //start_traj.frame = start_hand_pose_stamped;
//     start_traj.distal_frame = "RSoftHand";
//     start_traj.frame = r_start_hand_pose_stamped;
    
    

//     //// define the end frame - HARD CODE
//     geometry_msgs::PoseStamped r_end_hand_pose_stamped;
//     r_end_hand_pose_stamped.pose.position.x = 0.619;  // position and orientation are in world frame
//     r_end_hand_pose_stamped.pose.position.y = -0.29;
//     r_end_hand_pose_stamped.pose.position.z = 0.873;
//     r_end_hand_pose_stamped.pose.orientation.x = 0;
//     r_end_hand_pose_stamped.pose.orientation.y = -0.5591931143131625;
//     r_end_hand_pose_stamped.pose.orientation.z = 0;
//     r_end_hand_pose_stamped.pose.orientation.w = 0.8290374303399975;
//     // need to cast the variable as the pointer --> reverse later!!!
//     shared_data().rh_grasp_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(r_end_hand_pose_stamped));
    
//     //// define the end frame - from vision module
//     // wait for vision message
//     shared_data ().rh_grasp_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(shared_data ().rh_grasp_pose_topic);
//     
//     // convert object pose message to eigen
//     tf::Transform tf_cam_to_target;
//     geometry_msgs::Pose temp_pose;
//     temp_pose.position = shared_data().rh_grasp_pose->pose.position;
//     temp_pose.orientation = shared_data().rh_grasp_pose->pose.orientation;
//     tf::poseMsgToTF(temp_pose, tf_cam_to_target);
//     geometry_msgs::Transform gt_cam_to_target;
//     tf::transformTFToMsg(tf_cam_to_target, gt_cam_to_target);
//     Eigen::Affine3d cam_to_object;
//     tf::transformMsgToEigen(gt_cam_to_target, cam_to_object);
//     
//   
//     // get transformation from world to cam
//     Eigen::Affine3d world_to_cam;
//     //tf.getTransformTf("world_odom", "multisense/left_camera_optical_frame", world_to_cam);
//     tf.getTransformTf("multisense/left_camera_optical_frame", "world_odom", world_to_cam);
//     // get final world to object transform
//     Eigen::Affine3d world_to_object;
//     world_to_object = world_to_cam * cam_to_object;
//     // Transform from Eigen::Affine3d to geometry_msgs::Pose
//     geometry_msgs::Pose rh_grasp_pose_final;
//     tf::poseEigenToMsg (world_to_object, rh_grasp_pose_final);      
//     
//     // keep the position only
//     geometry_msgs::PoseStamped temp_pose_stamp;
//     temp_pose_stamp.pose.position = rh_grasp_pose_final.position;
//     std::cout << "--- POSITION 1: " << temp_pose_stamp.pose.position.x << " " 
// 				    << temp_pose_stamp.pose.position.y << " " 
// 				    << temp_pose_stamp.pose.position.z << std::endl;
// //     temp_pose_stamp.pose.orientation.x = 0;
// //     temp_pose_stamp.pose.orientation.y = -0.5591931143131625;
// //     temp_pose_stamp.pose.orientation.z = 0;
// //     temp_pose_stamp.pose.orientation.w = 0.8290374303399975;
//     temp_pose_stamp.pose.orientation.x = 0;
//     temp_pose_stamp.pose.orientation.y = 0;
//     temp_pose_stamp.pose.orientation.z = 0;
//     temp_pose_stamp.pose.orientation.w = 1;
//     temp_pose_stamp.header.frame_id = "world_odom";
//     // need to cast the variable as the pointer --> reverse later!!!
//     shared_data().rh_grasp_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(temp_pose_stamp));
//      
//     shared_data()._pub_obj_in_world.publish (temp_pose_stamp);
//     
// 
//     // 2ND TRY
//     tf::StampedTransform tf_world_to_cam;
//     tf.getTransformTf_direct("world_odom", "multisense/left_camera_optical_frame", tf_world_to_cam);
//     
//     tf::Transform tf_world_to_obj;
//     tf_world_to_obj = tf_world_to_cam * tf_cam_to_target;
//     geometry_msgs::Pose temp_pose2;
//     tf::poseTFToMsg(tf_world_to_obj, temp_pose2);
//     geometry_msgs::PoseStamped temp_pose_stamp2;
//     temp_pose_stamp2.pose.position = temp_pose2.position;
//     temp_pose_stamp2.pose.orientation.x = 0;
//     temp_pose_stamp2.pose.orientation.y = 0;
//     temp_pose_stamp2.pose.orientation.z = 0;
//     temp_pose_stamp2.pose.orientation.w = 1;
//     temp_pose_stamp2.header.frame_id = "world_odom";
//     shared_data().rh_grasp_pose2 = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(temp_pose_stamp2));
// 
//     std::cout << "=== POSITION 2: " << temp_pose_stamp2.pose.position.x << " " 
// 				    << temp_pose_stamp2.pose.position.y << " " 
// 				    << temp_pose_stamp2.pose.position.z << std::endl;
// 					    
//     shared_data()._pub_obj_in_world2.publish (temp_pose_stamp2);
    
    
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

    
	

//     // Move LH to LH_Pose (1 mid point in the z-axis fixed dist)
// 
//     // Only for first time sense and sync to the model
//     //TBD: put flag to check if it is first run to retrieve hands pose
//     shared_data()._robot->sense();
//     Eigen::Affine3d world_T_bl;
//     std::string fb;
//     shared_data()._robot->model().getFloatingBaseLink(fb);
//     std::cout << "--- floating base link: " << fb << std::endl;
//     
//     tf.getTransformTf(fb, shared_data ().frame_id_, world_T_bl);
//     std::cout << "--- Affine3d world_T_bl: " << world_T_bl.matrix() << std::endl;
//     
//     shared_data()._robot->model().setFloatingBasePose(world_T_bl);
//     shared_data()._robot->model().update();
// 
//     // Get current hand
//     KDL::Frame hand_pose_KDL;
//     Eigen::Affine3d hand_pose, r_hand_pose, camera_to_world;
//     geometry_msgs::Pose start_hand_pose, r_start_hand_pose, camera_pose_msgs;
// 
//     // Get hands poses
//     //shared_data()._robot->model().getPose("LSoftHand", hand_pose);
//     //shared_data().sl_hand_pose = hand_pose;
//     shared_data()._robot->model().getPose("RSoftHand", r_hand_pose);
//     shared_data().sr_hand_pose = r_hand_pose;
//     
//     //// Get camera pose: will transform a point from camera_frame to world frame ???
//     //shared_data()._robot->model().getPose("multisense/left_camera_optical_frame", camera_to_world);
//     
// 
// 
//     // Transform from Eigen::Affine3d to geometry_msgs::Pose
//     //tf::poseEigenToMsg (hand_pose, start_hand_pose);
//     tf::poseEigenToMsg (r_hand_pose, r_start_hand_pose);
// 
//     // Define the start frame as geometry_msgs::PoseStamped
//     //geometry_msgs::PoseStamped start_hand_pose_stamped;
//     //start_hand_pose_stamped.pose = start_hand_pose;
//     geometry_msgs::PoseStamped r_start_hand_pose_stamped;
//     r_start_hand_pose_stamped.pose = r_start_hand_pose;
// 
//     // Create the Cartesian trajectories
//     trajectory_utils::Cartesian start_traj;
//     //start_traj.distal_frame = "LSoftHand";
//     //start_traj.frame = start_hand_pose_stamped;
//     start_traj.distal_frame = "RSoftHand";
//     start_traj.frame = r_start_hand_pose_stamped;
//     
//     
// 
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
//     shared_data ().rh_grasp_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(shared_data ().rh_grasp_topic);
//     
//     // convert object pose message to eigen
//     tf::Transform tf_cam_to_target;
//     geometry_msgs::Pose temp_pose;
//     temp_pose.position = shared_data().rh_grasp_pose->pose.position;
//     temp_pose.orientation = shared_data().rh_grasp_pose->pose.orientation;
//     tf::poseMsgToTF(temp_pose, tf_cam_to_target);
//     geometry_msgs::Transform gt_cam_to_target;
//     tf::transformTFToMsg(tf_cam_to_target, gt_cam_to_target);
//     Eigen::Affine3d cam_to_object;
//     tf::transformMsgToEigen(gt_cam_to_target, cam_to_object);
//     
//   
//     // get transformation from world to cam
//     Eigen::Affine3d world_to_cam;
//     //tf.getTransformTf("world_odom", "multisense/left_camera_optical_frame", world_to_cam);  // not work
//     tf.getTransformTf("multisense/left_camera_optical_frame", "world_odom", world_to_cam);
//     //tf.getTransformTf("multisense/left_camera_optical_frame", "world_odom", world_to_cam);
//     // get final world to object transform
//     Eigen::Affine3d world_to_object;
//     world_to_object = world_to_cam * cam_to_object;
//     // Transform from Eigen::Affine3d to geometry_msgs::Pose
//     geometry_msgs::Pose rh_grasp_pose_final;
//     tf::poseEigenToMsg (world_to_object, rh_grasp_pose_final);      
//     
//     // keep the position only
//     geometry_msgs::PoseStamped temp_pose_stamp;
//     temp_pose_stamp.pose.position = rh_grasp_pose_final.position;
//     std::cout << "--- FINAL OBJ POSITION: " << temp_pose_stamp.pose.position.x << " " 
// 					    << temp_pose_stamp.pose.position.y << " " 
// 					    << temp_pose_stamp.pose.position.z << std::endl;
//     temp_pose_stamp.pose.orientation.x = 0;
//     temp_pose_stamp.pose.orientation.y = -0.5591931143131625;
//     temp_pose_stamp.pose.orientation.z = 0;
//     temp_pose_stamp.pose.orientation.w = 0.8290374303399975;
// //     temp_pose_stamp.pose.orientation.x = 0;
// //     temp_pose_stamp.pose.orientation.y = 0;
// //     temp_pose_stamp.pose.orientation.z = 0;
// //     temp_pose_stamp.pose.orientation.w = 1;
//     temp_pose_stamp.header.frame_id = "world_odom";
//     // need to cast the variable as the pointer --> reverse later!!!
//     shared_data().rh_grasp_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(temp_pose_stamp));
//      
//     shared_data()._pub_rh_grasp_pose.publish (temp_pose_stamp);
//     
//     
//     
//     // CALL trajectory_utils
//     trajectory_utils::Cartesian end;
//     //end.distal_frame = "LSoftHand";
//     //end.frame = end_hand_pose_stamped;
//     end.distal_frame = "RSoftHand";
//     //end.frame = r_end_hand_pose_stamped;
//     end.frame = *shared_data().rh_grasp_pose; // to test hardcode pose; rh_grasp_pose is a pointer --> need *
// 
//     // define the first segment
//     trajectory_utils::segment s1;
//     s1.type.data = 0;        // min jerk traj
//     s1.T.data = 5.0;        // traj duration 1 second      
//     s1.start = start_traj;   // start pose
//     s1.end = end;            // end pose 
// 
//     
// //     start_traj.frame = end_hand_pose_stamped;
// // 
// //     end_hand_pose_stamped.pose.position.z =
// //     shared_data()._hose_grasp_pose->pose.position.z;
// //     end.frame = end_hand_pose_stamped;
// //     
// //     end_hand_pose_stamped.pose.position.z = 0.1;
// //     end.frame = end_hand_pose_stamped;
// 
// //     // define the second segment
// //     trajectory_utils::segment s2;
// //     s2.type.data = 0;        // min jerk traj
// //     s2.T.data = 10.0;        // traj duration 1 second      
// //     s2.start = start_traj;   // start pose
// //     s2.end = end;            // end pose 
// 
//     // only one segment in this example
//     std::vector<trajectory_utils::segment> segments;
//     segments.push_back (s1);
//     //segments.push_back (s2);
// 
//     // prapere the advr_segment_control
//     ADVR_ROS::advr_segment_control srv;
//     srv.request.segment_trj.header.frame_id = shared_data ().frame_id_;
//     srv.request.segment_trj.header.stamp = ros::Time::now();
//     srv.request.segment_trj.segments = segments;
// 
//     // call the service
//     shared_data()._client.call(srv);
// 
//     
// //     // save last hand pose
// //     shared_data()._last_lh_pose =
// //     boost::shared_ptr<geometry_msgs::PoseStamped>
// // 	(new geometry_msgs::PoseStamped (end_hand_pose_stamped));
// //     shared_data()._last_rh_pose =
// //     boost::shared_ptr<geometry_msgs::PoseStamped>
// // 	(new geometry_msgs::PoseStamped (r_end_hand_pose_stamped));
// 	
//     // Info msg
//     //std::cout << "Send \"lh_move_fail\" or \"success\" msg..." << std::endl;
//     std::cout << "Leaving Move_RH_Entry ..." << std::endl;




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