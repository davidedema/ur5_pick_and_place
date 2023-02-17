#!/usr/bin/env python
#import libraries
from pathlib import Path
import sys
import os
import rospy as ros
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32
from motion.msg import pos
from LegoDetect import LegoDetect

# -----------------------------------------------------------------------------------------
# Global variables and paths
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
IMG_ZED = os.path.abspath(os.path.join(ROOT, "log/img_ZED_cam.png"))

# Camera parameters
w_R_c = np.matrix([[0, -0.499, 0.866], [-1, 0, 0], [0, -0.866, -0.499]])
x_c = np.array([-0.9, 0.24, -0.35])
base_offset = np.array([0.5, 0.35, 1.75])

#height offset from the ground
OFF_SET = 0.86 + 0.1

# -----------------------------------------------------------------------------------------

class Vision:

    def __init__(self):

        ros.init_node('vision', anonymous=True)

        self.lego_list = []
        self.bridge = CvBridge() # Convert ROS image to OpenCV image

        # Flags
        self.allow_receive_image = True
        self.allow_receive_pointcloud = False
        self.vision_ready = 0
        
        # Publish the position of the lego
        self.pos_pub = ros.Publisher("/vision/pos", pos, queue_size=1) 
        self.ack_sub = ros.Subscriber('/vision/ack', Int32, self.ackCallbak) # Subscribe to the ack topic get from motion planner

        # Subscribe to the image topic
        self.image_sub = ros.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, self.receive_image)
        #subscribe to the point cloud topic to get distance and x,y,z coordinates from the camera
        self.sub_pointcloud = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, self.receive_pointcloud, queue_size=1)

        self.ack_pub = ros.Publisher('/taskManager/stop', Int32, queue_size=1)
    
    # -----------------------------------------------------------------------------------------
    # Callback functions for ROS subscribers
    # -----------------------------------------------------------------------------------------
    def receive_image(self, data):

        # Flag for temporizing
        if not self.allow_receive_image:
            return
        self.allow_receive_image = False

        # Convert ROS image to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Save image and detect lego
        cv.imwrite(IMG_ZED, cv_image)
        legoDetect = LegoDetect(IMG_ZED)
        self.lego_list = legoDetect.lego_list
        
        self.allow_receive_pointcloud = True
    
    def receive_pointcloud(self, msg):

        # Flag for temporizing
        if not self.allow_receive_pointcloud:
            return
        self.allow_receive_pointcloud = False
        
        self.pos_msg_list = []

        for lego in self.lego_list:

            # Get point cloud from camera
            for data in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=True, uvs=[lego.center_point]):
                lego.point_cloud = (data[0], data[1], data[2])

            # Transform point cloud to world
            lego.point_world = w_R_c.dot(lego.point_cloud) + x_c + base_offset

            # Show details
            lego.show()

            # Create msg for pos_pub
            pos_msg = pos()
            pos_msg.class_id = lego.class_id
            pos_msg.x = lego.point_world[0, 0]
            pos_msg.y = lego.point_world[0, 1]
            pos_msg.z = lego.point_world[0, 2]
            pos_msg.pitch = 0
            pos_msg.roll = 0
            pos_msg.yaw = 0

            #check if the lego is on the ground and remove all detection not wanted
            if pos_msg.z < OFF_SET:
                self.pos_msg_list.append(pos_msg)
            
        print('\nVISION DONE DETECTING LEGO!\nREADY FOR MOTION!')
        self.vision_ready = 1
        self.send_pos_msg() # Send the position of the lego to motion planner

    # definition of the function to check if the motion planner is ready to receive the position of the lego
    def ackCallbak(self, ack_ready):
        
        if self.vision_ready == 1 and ack_ready.data == 1:
            self.send_pos_msg()
    
    # definition of the function to send the position of the lego to motion planner
    def send_pos_msg(self): 
        try:
            pos_msg = self.pos_msg_list.pop()
            self.pos_pub.publish(pos_msg)
            print('\nPosition published:\n', pos_msg)
        except IndexError:
            print('\nFINISH ALL LEGO\n')
            
            

# -----------------------------------------------------------------------------------------
# Main function

if __name__ == '__main__':

    vision = Vision()

    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
