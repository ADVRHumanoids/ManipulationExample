import rospy 
import cv2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2


class ImageIO:
    def __init__(self):
        self.bridge = CvBridge()
        self.asus_rgb_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.asus_rgb_callback)
        self.asus_dep_sub = rospy.Subscriber("/camera/depth/image_rect", Image, self.asus_dep_callback)
        self.asus_cam_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.asus_cam_callback) 
        #self.asus_pcl_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.asus_pcl_callback)
        self.asus_rgb_img = None
        self.asus_dep_img = None
        
        self.asus_K  = None
        self.asus_fx = None ###     -----> x
        self.asus_fy = None ###     -
        self.asus_cx = None ###     -
        self.asus_cy = None ###     y  
        self.asus_width = None
        self.asus_height = None
        
        
        self.asus_pcl_dat = None
        self.asus_temp = []          
        
        ###########3
        self.mult_K  = None
        self.mult_fx = None ###     -----> x
        self.mult_fy = None ###     -
        self.mult_cx = None ###     -
        self.mult_cy = None ###     y  
        self.mult_width = None
        self.mult_height = None
        
        # for multisense
        #self.mult_rgb_sub = rospy.Subscriber("/multisense/left/image_color", Image, self.mult_rgb_callback)
        #self.mult_dep_sub = rospy.Subscriber("/multisense/depth", Image, self.mult_dep_callback)
        self.mult_rgb_sub = rospy.Subscriber("/multisense/left/image_color", Image, self.mult_rgb_callback)
        self.mult_dep_sub = rospy.Subscriber("/multisense/depth", Image, self.mult_dep_callback)
        self.mult_cam_sub = rospy.Subscriber("/multisense/depth/camera_info", CameraInfo, self.mult_cam_callback)  ## CHECK LATER
        
        self.mult_rgb_img = None
        self.mult_dep_img = None
        
        
    def mult_rgb_callback(self, data):
        try:
            self.mult_rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8") #careful with rgb - bgr
#             h, w, c = self.mult_rgb_img.shape
#             if h > 60 and w > 60 :
#                 cv2.imshow("Detection_Input", self.mult_rgb_img)
#                 cv2.waitKey(3)
#                 print "got 1 image ..."
        
        except CvBridgeError as e:
            print(e)
        

    def mult_dep_callback(self, dep_msg):
        try:
            self.mult_dep_img = self.bridge.imgmsg_to_cv2(dep_msg, 'passthrough')
        
        except CvBridgeError as e:
            print (e)
            
            
            
    def asus_rgb_callback(self, data):
        try:
            self.asus_rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8") #careful with rgb - bgr
#             h, w, c = self.asus_rgb_img.shape
#             if h > 60 and w > 60 :
#                 cv2.imshow("Detection_Input", in_asus_rgb)
#                 cv2.waitKey(3)
#                 print "got 1 image ..."
            
        except CvBridgeError as e:
            print(e)
    
    def asus_dep_callback(self, dep_msg):
        try:
            self.asus_dep_img = self.bridge.imgmsg_to_cv2(dep_msg, 'passthrough')
        
        except CvBridgeError as e:
            print (e)
            
    def asus_cam_callback(self, cam_msg):
        if (self.asus_K == None):
#             print 'K is none!'

#             self.asus_K = cam_msg.K
#             self.asus_K = [550.6944711701044, 0.0, 325.91239272527577, 0.0, 553.678494137733, 233.01009570210894, 0.0, 0.0, 1.0]
            self.asus_K = [537.719521812601, 0.0, 317.7514539076608, 0.0, 532.663175739311, 226.92788210884436, 0.0, 0.0, 1.0]

            self.asus_fx = self.asus_K[0]
            self.asus_cx = self.asus_K[2]
            self.asus_fy = self.asus_K[4]
            self.asus_cy = self.asus_K[5]
            self.asus_width = cam_msg.width
            self.asus_height = cam_msg.height
            
#             print 'fx = ', self.asus_fx
#             print 'fy = ', self.asus_fy
#             print 'cx = ', self.asus_cx
#             print 'cy = ', self.asus_cy
            
        else:
#             Unsubscribe from any more messages 
#             print 'Unsubscribe from camera info ...!'        
            self.asus_cam_sub.unregister()
            
            
    def mult_cam_callback(self, cam_msg):
        if (self.mult_K == None):
        #             print 'K is none!'
        
            self.mult_K = cam_msg.K
        #             self.asus_K = [550.6944711701044, 0.0, 325.91239272527577, 0.0, 553.678494137733, 233.01009570210894, 0.0, 0.0, 1.0]
            #self.asus_K = [537.719521812601, 0.0, 317.7514539076608, 0.0, 532.663175739311, 226.92788210884436, 0.0, 0.0, 1.0]
            
            self.mult_fx = self.mult_K[0]
            self.mult_cx = self.mult_K[2]
            self.mult_fy = self.mult_K[4]
            self.mult_cy = self.mult_K[5]
            self.mult_width = cam_msg.width
            self.mult_height = cam_msg.height
        
        #             print 'fx = ', self.asus_fx
        #             print 'fy = ', self.asus_fy
        #             print 'cx = ', self.asus_cx
        #             print 'cy = ', self.asus_cy
        
        else:
        #             Unsubscribe from any more messages 
        #             print 'Unsubscribe from camera info ...!'        
            self.mult_cam_sub.unregister()
               
               
    def asus_pcl_callback(self, pcl_msgs):
        #self.asus_pcl_dat = pcl_msgs
        self.asus_pcl_dat = pc2.read_points(pcl_msgs)
        
#         for p in self.asus_pcl_dat:
#             print 'px, py, pz: ', p[0], p[1], p[2]
#             print 'done!'
#             #break
     
#         for p in self.asus_pcl_dat:
#             self.asus_temp.append(p)
        #print self.asus_temp
        #print 'POINTS: ', self.asus_pcl_dat(10, 0)
