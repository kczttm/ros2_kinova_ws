# Publish real-time camera image with cv2
# Auther: Chuizheng Kong
# Created on: 03/05/2024

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import numpy as np
import sys
import cv2
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self, cam_id = 0):
        super().__init__('cv_img_pub')
        self.pub = self.create_publisher(Image, 'cv_frame'+str(cam_id), 10)

        self.pub_period = 0.1

        self.timer = self.create_timer(self.pub_period, self.timer_callback)

        self.cap = cv2.VideoCapture(cam_id) # check which number in dev/video#

        self.cb = CvBridge()

    def timer_callback(self):

        # true/false received frame, frame
        ret, frame = self.cap.read()

        if ret:
            self.pub.publish(self.cb.cv2_to_imgmsg(frame))

        # Display the message on the console
        #self.get_logger().info('A frame published in %d Hz' % (1/self.pub_period))

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) > 1:
        image_pub = ImagePublisher(int(sys.argv[1]))
    else:
        image_pub = ImagePublisher()
    rclpy.spin(image_pub)

    # when done
    image_pub.cap.release()
    image_pub.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    cap = cv2.VideoCapture(2,cv2.CAP_V4L2)
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    # width = 1920
    # height = 1080
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        w, h = frame.shape[1], frame.shape[0]
        cv2.imshow('frame {} x {}'.format(w,h), frame)
        # termination choices
        keyPressed = cv2.waitKey(1)
        if keyPressed == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
