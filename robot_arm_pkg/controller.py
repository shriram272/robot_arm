#!/usr/bin/env python

import cv2 as cv
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image

from src.robot_arm_pkg.brain import movement

class Controller:
    def __init__(self):
        rospy.init_node('controller')
        
        self.cap = cv.VideoCapture(1)
        self.controller = rospy.Publisher('/servo_node/delta_twist_cmds', TwistStamped, queue_size=10)
        self.image_publisher = rospy.Publisher("frames", Image, queue_size=10)
        self.bridge = CvBridge()
        
    def timer_callback(self, event):
        success, image = self.cap.read()
        
        if success:      
            position = movement(image)
            x = position[0]
            y = position[1]
            
            if x != 0 and y != 0:
                velocity = TwistStamped()
                velocity.header.frame_id = "panda_link0"
                velocity.header.stamp = rospy.Time.now()
                
                if x > 0.65:
                    velocity.twist.linear.y = -0.8
                elif x < 0.55:
                    velocity.twist.linear.y = 0.8
                    
                if y > 0.65:
                    velocity.twist.linear.x = -0.8
                elif y < 0.55:
                    velocity.twist.linear.x = 0.8
                
                self.controller.publish(velocity)
                
            image = cv.flip(image, 1)
            self.image_publisher.publish(self.bridge.cv2_to_imgmsg(image))
            
    def show_image(self, data):
        frame = self.bridge.imgmsg_to_cv2(data)
        cv.imshow('Controller', frame)
        cv.waitKey(1)
        
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.timer_callback(None)
            rate.sleep()
        
def main():
    controller = Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
