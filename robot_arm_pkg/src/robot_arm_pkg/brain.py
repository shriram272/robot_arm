#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import mediapipe as mp

""" Importing the hands module """
mp_hands = mp.solutions.hands

""" Creating an object of the hands class """
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)


def __init__(self):
        rospy.init_node('hand_movement_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
def movement(self, image):
        """ Flipping the image for a selfie-view display """
        image = cv.flip(image, 1) 
        
        """ To improve performance, mark the image as not writeable. """
        image.flags.writeable = False
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        results = hands.process(image)
        
        image.flags.writeable = True
        image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
        
        if results.multi_hand_landmarks not in (None, []):
            for hand_landmark in results.multi_hand_landmarks:
                """ Storing the coordinates of the wrist """
                index_tip = hand_landmark.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                return [index_tip.x, index_tip.y]
        
        return [0, 0]

def image_callback(self, data):
        """ Convert ROS Image message to OpenCV image """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        position = self.movement(cv_image)
        x, y = position[0], position[1]

        rospy.loginfo(f"Detected finger tip position: x={x}, y={y}")

def main():
   
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
