import cv2
import sys
import argparse
import rospy
from geometry_msgs.msg import PoseStamped
from ros_image_io import ImageIO


BLUE = [255,0,0]        # rectangle color
RED = [0,0,255]         #
GREEN = [0,255,0]       # 
BLACK = [0,0,0]         # 
WHITE = [255,255,255]   # 

DRAW_BG = {'color' : BLACK, 'val' : 0}
DRAW_FG = {'color' : WHITE, 'val' : 1}
DRAW_PR_FG = {'color' : GREEN, 'val' : 3}
DRAW_PR_BG = {'color' : RED, 'val' : 2}

# setting up flags
rect = (0,0,1,1)
drawing = False         # flag for drawing curves
rectangle = False       # flag for drawing rect
rect_over = False       # flag to check if rect drawn
value = DRAW_FG         # drawing initialized to FG
thickness = 3           # brush thickness

center_point = (0,0)       ## center point of the rectangle
grasping_point = (0, 0, 0) ## 3D point 

# start ros node and imageio
rospy.init_node('WalkMan_Vision_Node')
ic = ImageIO()

#camera_type = 'asus'
camera_type = 'multisense'



def onmouse(event,x,y,flags,param):
    global img,img2,drawing,value,mask,rectangle,rect,rect_or_mask,ix,iy,rect_over, center_point

    # Draw Rectangle
    if event == cv2.EVENT_RBUTTONDOWN:
        rectangle = True
        ix,iy = x,y

    elif event == cv2.EVENT_MOUSEMOVE:
        if rectangle == True:
            img = img2.copy()
            cv2.rectangle(img,(ix,iy),(x,y),BLUE,2)
            rect = (min(ix,x),min(iy,y),abs(ix-x),abs(iy-y))
            rect_or_mask = 0

    elif event == cv2.EVENT_RBUTTONUP:
        rectangle = False
        rect_over = True
        cv2.rectangle(img,(ix,iy),(x,y),BLUE,2)
        rect = (min(ix,x),min(iy,y),abs(ix-x), abs(iy-y))
        print '-----------------------------------------------'
        print 'Current rectangle: ', rect
        c_x = int( (x - ix) / 2) + ix
        c_y = int( (y - iy) / 2) + iy
        center_point = (c_x, c_y)
        print 'Current point: ', center_point
        cv2.circle(img, center_point, 7, RED, -1)
        print 'Now press key d if you are happy with this rectangle.'


def project_to_3D_asus(width_x, height_y, depth, ic):
    X = (width_x - ic.asus_cx) * dval / ic.asus_fx      
    Y = (height_y - ic.asus_cy) * dval / ic.asus_fy
    Z = depth
    p3D = [X, Y, Z]
    
    return p3D

def project_to_3D_multisense(width_x, height_y, depth, ic):
    X = (width_x - ic.mult_cx) * dval / ic.mult_fx      
    Y = (height_y - ic.mult_cy) * dval / ic.mult_fy
    Z = depth
    p3D = [X, Y, Z]
    
    return p3D



def parse_args():
    parser = argparse.ArgumentParser(description='Walk-man vision demo')
    parser.add_argument('--task', dest='task_name', help='Name of the current task (debris or valve). Defaul is debris',
                        default='debris', type=str)
    
    args = parser.parse_args()

    return args


if __name__ == '__main__':

#     filename = 'test_img.jpg'
#     img = cv2.imread(filename)
    
    args = parse_args()
    
    if args.task_name == 'debris':
        pub_obj_pose_3D = rospy.Publisher("vs_debris_obj_pose_3D", PoseStamped) # pose of object in camera frame
    elif args.task_name == 'valve':
        pub_obj_pose_3D = rospy.Publisher("vs_valve_obj_pose_3D", PoseStamped) # pose of object in camera frame
    else:
        print 'ERROR: wrong task name!'
        sys.exit(0)
    

    process_flag = True 
    
    


    
    print 'Instructions:'
    print 'Draw a rectangle around the area you want to grasp using the right mouse button.\n'

    
    while(1):
        if camera_type == 'asus':
    	    rgb = ic.asus_rgb_img
    	    dep = ic.asus_dep_img
	else:
            rgb = ic.mult_rgb_img
    	    dep = ic.mult_dep_img

        if (rgb != None and dep != None):
            #print 'rgb shape: ', rgb.shape
            
            img = rgb
            img2 = rgb ## keep a copy to refresh
            
            while(process_flag):
                cv2.namedWindow('input')
                cv2.setMouseCallback('input',onmouse)
                cv2.moveWindow('input',img.shape[1]+10,90)
    
                cv2.imshow('input', img)
                k = cv2.waitKey(1)
                #print 'key value: ', k
        
                # key bindings
                if k == 27:         # esc to exit
                    process_flag = True ## to get new image
                    break
                
                elif k == 100: ## d key ## DO NOT ACTIVE caplock key
                    print 'PROCESSING ..............'
                    print 'Selected rectangle: ', rect
                    print 'Selected center point: ', center_point
                    process_flag = False
                    
                    # get depth value from depth map                        
                    dval = dep[center_point[1], center_point[0]]
                    if dval != 'nan':
                    
                        if camera_type == 'asus':
                            p3Dc = project_to_3D_asus(center_point[0], center_point[1], dval, ic)
                            print '3D point: ', p3Dc
                        else:
			    p3Dc = project_to_3D_multisense(center_point[0], center_point[1], dval, ic)
                            print '3D point: ', p3Dc

                        obj_pose_3D = PoseStamped()
                        #obj_pose_3D.header.frame_id = "camera_depth_optical_frame"
                        if camera_type == 'asus':
                            obj_pose_3D.header.frame_id = "camera_depth_optical_frame"
                        else:
                            obj_pose_3D.header.frame_id = 'multisense/left_camera_optical_frame'
                       
                        obj_pose_3D.pose.position.x = p3Dc[0]
                        obj_pose_3D.pose.position.y = p3Dc[1]
                        obj_pose_3D.pose.position.z = p3Dc[2]
                        obj_pose_3D.pose.orientation.x = 0
                        obj_pose_3D.pose.orientation.y = 0
                        obj_pose_3D.pose.orientation.z = 0
                        obj_pose_3D.pose.orientation.w = 1 ## no rotation
                        # publish pose
                        
                        #while (1):
	                #    pub_obj_pose_3D.publish(obj_pose_3D)
			
			pub_obj_pose_3D.publish(obj_pose_3D)
                    
                    
                    break
                    
            cv2.destroyAllWindows()         
            
#             # hook again
#             k = cv2.waitKey(1)
#             if k == 114: ## r key
#                 process_flag = True

           
    #print 'ALL DONE!'
