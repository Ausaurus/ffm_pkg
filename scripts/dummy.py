#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

class HelloPublisher:
    def __init__(self):
        self.activate_detection_sub = rospy.Subscriber('/activate_model', Bool, self.activate_detection_callback)
        self.resume_waypoint_pub = rospy.Publisher('/resume_detection', Bool, queue_size=10)
        
        self.activated = False

    def activate_detection_callback(self, msg):
        if msg.data:
            rospy.loginfo("Hello")
            rospy.sleep(5)  # Stop for 5 seconds
            self.resume_waypoint_pub.publish(True)
            rospy.loginfo("Published True to /resume_waypoint")
            self.activated = False  # Reset activation

    def run(self):
        rospy.loginfo("Waiting for activation...")
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('hello_publisher')
    
    hello_publisher = HelloPublisher()
    hello_publisher.run()
