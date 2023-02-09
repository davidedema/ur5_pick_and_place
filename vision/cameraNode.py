#!/usr/bin/env python
import rospy as ros
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import params as conf
import ros_numpy
import torch
import math
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2


#path to the weights of the model
weights_path = "../weights/best.pt"
model = torch.hub.load('ultralytics/yolov5', 'custom', weights_path)

class image_converter:


    x_pixel = 0;
    y_pixel = 0;

    def __init__(self):

        # self.image_pub = ros.Publisher("/z_base_camera/camera/rgb/image_raw",Image, queue_size=1)

        self.bridge = CvBridge()
        #self.image_sub = ros.Subscriber("/ur5/zed_node/left_raw-image_raw_color",Image,self.callback)
        self.image_sub = ros.Subscriber("/ur5/zed_node/left_raw/image_raw_color",Image,self.callback)
        self.sub_pointcloud = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, self.receive_pointcloud, queue_size=1)

    def receive_pointcloud(self, msg):
    # read all the points
        array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)

        # get the lowest distance
        distance_array = np.zeros(shape=(array.shape[0], 1))
        for i in range(array.shape[0]):
            distance = math.sqrt(array[i, 0] ** 2 + array[i, 1] ** 2 + array[i, 2] ** 2)
            #print(distance)
            distance_array[i, 0] = distance
        print("Minimum distance  is: " + str(np.min(distance_array)))
        min_index = np.argmin(distance_array)
        print("Minimum distance index is: " + str(np.argmin(distance_array)))
        print("position  of min distance point is: " + str(array[min_index, :]))

        #get one (or more) points from the pointcloud (unfortunately you need an iterator)
        # These X,Y,Z values are in the zed2_left_camera_optical_frame (X is seen as going to left (looking the camera)
        #  Y is top to bottom and Z pointing out the camera). You can select the pixel in the image frame with the u,v variable,
        #  u = 0 , v = 0 is the top right corner, u = 640 , v = 360 is the middle of the image plane which corresponds to the origin of the zed2_left_camera_optical_frame
        points_list = []
        for data in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=False, uvs=[(x_pixel, y_pixel)]):
            points_list.append([data[0], data[1], data[2]])
        print("Data: ", points_list)




    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #pass the image to opencv
        results=model(cv_image) 

        #get the bounding box 4 coordinates (x1,y1,x2,y2)
        bb = results.xyxy[0][0]
        x1 = bb[0]
        y1 = bb[1]
        x2 = bb[2]
        y2 = bb[3]
        
        #compute the center of the bounding box
        x = int((x1+x2)/2)
        y = int((y1+y2)/2)
        
        x_pixel = x
        y_pixel = y
        
        #print a circle in the middle of the image
        cv.circle(cv_image, (x,y), 5, (0,0,255), -1)

        #(rows,cols,channels) = cv_image.shape

        cv.imshow("Image window", cv_image)
        cv.waitKey(3)

        '''
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
        '''
if __name__ == '__main__':
    ic = image_converter()
    ros.init_node('image_converter', anonymous=True)
    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()
