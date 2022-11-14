#!/usr/bin/env python
import rospy as ros
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import params as conf

class image_converter:
    def __init__(self):

        self.image_pub = ros.Publisher("/z_base_camera/camera/rgb/image_raw",Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = ros.Subscriber("/z_base_camera/camera/rgb/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv.circle(cv_image, (50,50), 10, 255)

        cv.imshow("Image window", cv_image)
        cv.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    ic = image_converter()
    ros.init_node('image_converter', anonymous=True)
    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()
