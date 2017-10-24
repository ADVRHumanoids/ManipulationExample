import numpy as np
import sys
import time
import os
  
import roslib  
import rospy 
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def padzero(x):
    if x>=0 and x<10:
        return '0000000' + str(x)
    elif x>=10 and x<100:
        return '000000' + str(x)
    elif x>=100 and x<1000:
        return '00000' + str(x)
    elif x>=1000 and x<10000:
        return '0000' + str(x)
    elif x>=10000 and x<100000:
        return '000' + str(x)
    elif x>=100000 and x<1000000:
        return '00' + str(x)
    elif x>=1000000 and x<10000000:
        return '0' + str(x)
    else:
        return '___error___'
          
class image_converter:
  
    
    def __init__(self):
        #self.image_pub = rospy.Publisher("image_topic_2", Image)
      
        counter = '40'
        self.img_path = "/home/anguyen/workspace/dataset/IIT_Multisense/img" + counter + '/' 
        self.dep_path = "/home/anguyen/workspace/dataset/IIT_Multisense/dep" + counter + '/'
        
        if not os.path.exists(self.img_path):
            os.makedirs(self.img_path)   
            
        if not os.path.exists(self.dep_path):
            os.makedirs(self.dep_path) 

        self.img_path = self.img_path + counter + "_"
        self.dep_path = self.dep_path + counter + "_"
        
        self.count = 0;
        self.bridge = CvBridge()
        
        ## ASUS 
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        #self.deep_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.deep_callback)
  
        # MULTISENSE
        self.image_sub = rospy.Subscriber("/multisense/left/image_color", Image, self.callback)
        self.deep_sub = rospy.Subscriber("/multisense/depth", Image, self.deep_callback)

        
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
              
              
        except CvBridgeError as e:
            print(e)
      
        # init frame 
        frame_result = cv_image
          
        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            rgb_name = self.img_path + padzero(self.count) + ".jpg"
            #cv2.circle(cv_image, (50, 50), 10, 255)
              
            # execute affordances detection
            #frame_result = acnn.execute(cv_image)
            print "ok"
            #cv2.imshow("Detection_Input", cv_image)
            cv2.imwrite(rgb_name, cv_image)
        #cv2.imshow("Detection_Aff", frame_result)
            cv2.waitKey(3)
        else:
            print "error"
              
    def deep_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
            
            #print depth_image.shape
            dep_name = self.dep_path + padzero(self.count) + ".txt"
            np.savetxt(dep_name, depth_image, "%f")
            self.count = self.count + 1;
                       
        except CvBridgeError, e:
            print e
  
def main(args):
    ic = image_converter()
    print "AAAA"
    rospy.init_node('image_converter', anonymous=True)
    print "BBB"
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
  
  
if  __name__ == '__main__':
    main(sys.argv)
    print "ALL DONE!"

# 
#  
# 
# import roslib;
# import sys
# import rospy
# import cv2
# import cv2.cv
# import message_filters
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# from datetime import datetime
#  
# class follow_person:
#     def __init__(self):
#         self.bridge = CvBridge()
#  
#         image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
#         depth_sub = message_filters.Subscriber("/camera/depth_registered/image_raw", Image)
#  
#         self.ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
#         self.ts.registerCallback(self.callback)
#  
#  
#     def callback(self, rgb_data, depth_data):
#         try:
#             image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
#             depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
#         except CvBridgeError, e:
#             print e
#  
#         cv2.imshow('RGB', image)
#         cv2.imshow('Depth', depth_image)
#         cv2.waitKey(3)
#  
# def main(args):
#     fp = follow_person()
#     rospy.init_node('follow_person', anonymous=True)
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print "Shutting down"
#     cv2.destroyAllWindows()
#  
# if __name__ == '__main__':
#     main(sys.argv)