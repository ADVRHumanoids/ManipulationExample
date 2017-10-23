import numpy as np
import random
import scipy.io as sio
import os, sys, cv2
import argparse

### ROS STUFF
import rospy 
from geometry_msgs.msg import PoseStamped
from time import gmtime, strftime

camera_type = 'multisense'

root_img_folder = '/home/anguyen/workspace/dataset/IMG_FROM_ROBOT'
paper_exp = '2018.icra.sds'
current_time =  strftime("%Y_%m_%d__%H_%M_%S", gmtime())

# global ROS info
rospy.init_node('fake_pose_node')


# publisher
pub_vs_debris_obj_pose_3D = rospy.Publisher("vs_debris_obj_pose_3D", PoseStamped) # FAKE debris pose in camera frame
pub_vs_valve_obj_pose_3D = rospy.Publisher("vs_valve_obj_pose_3D", PoseStamped) 



def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='Faster R-CNN demo')
    parser.add_argument('--gpu', dest='gpu_id', help='GPU device id to use [0]',
                        default=0, type=int)
    parser.add_argument('--cpu', dest='cpu_mode',
                        help='Use CPU mode (overrides --gpu)',
                        action='store_true')
    parser.add_argument('--net', dest='demo_net', help='Network to use [vgg16]',
                        choices=NETS.keys(), default='vgg16')

    args = parser.parse_args()

    return args


def main():

	# publish fake debris pose
	debris_pose_FAKE = PoseStamped()
	if camera_type == 'asus':
		debris_pose_FAKE.header.frame_id = "camera_depth_optical_frame"
	else:
		debris_pose_FAKE.header.frame_id = 'multisense/left_camera_optical_frame'
	debris_pose_FAKE.pose.position.x = -0.38 #hardcode values   ## FOR LEFTHAND
	debris_pose_FAKE.pose.position.y = 0.05
	debris_pose_FAKE.pose.position.z = 0.63
	debris_pose_FAKE.pose.orientation.x = 0
	debris_pose_FAKE.pose.orientation.y = 0
	debris_pose_FAKE.pose.orientation.z = 0
	debris_pose_FAKE.pose.orientation.w = 1 ## no rotation
	pub_vs_debris_obj_pose_3D.publish(debris_pose_FAKE)
	
	# publish fake valve pose
	valve_pose_FAKE = PoseStamped()
	if camera_type == 'asus':
		valve_pose_FAKE.header.frame_id = "camera_depth_optical_frame"
	else:
		valve_pose_FAKE.header.frame_id = 'multisense/left_camera_optical_frame'
	valve_pose_FAKE.pose.position.x = -0.38 #hardcode values
	valve_pose_FAKE.pose.position.y = 0.05
	valve_pose_FAKE.pose.position.z = 0.63
	valve_pose_FAKE.pose.orientation.x = 0
	valve_pose_FAKE.pose.orientation.y = 0
	valve_pose_FAKE.pose.orientation.z = 0
	valve_pose_FAKE.pose.orientation.w = 1 ## no rotation
	pub_vs_valve_obj_pose_3D.publish(valve_pose_FAKE)
	
	
	
	
	

if __name__ == '__main__':
	
	while (1):
		main()
	
	
	
                        
