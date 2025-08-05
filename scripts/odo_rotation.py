#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as tf_trans

class MainGreeting:
    def __init__(self):  #Method/instance method
        rospy.init_node('main_greeting', anonymous=True)
        self.person_centered_sub = rospy.Subscriber('/person_centered', Bool, self.person_centered_callback)  #initialize the subscriber, data is obtained every time it's called
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
   
        self.person_centered = False  # Assume initially no person is centered
        self.initial_check_done = False  #Instance Attributes: These are the variables defined with self
        self.start_yaw = None
        self.current_yaw = None
        self.angle_turned = 0
        #Each instance of the class has its own set of these attributes shared among its diff method.
        
        #on the other hand, Class Attributes: If defined at the class level (not inside any method and without self), they are shared across all instances of the class.
        
        self.rate = rospy.Rate(10)  

    def person_centered_callback(self, msg):  #method to be called by run
        self.person_centered = msg.data
        self.initial_check_done = True  # Indicate that the initial check is complete

    def odom_callback(self, msg):
        # Extract orientation in quaternion and convert to Euler angles
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_trans.euler_from_quaternion(orientation_list)
        
        self.current_yaw = yaw  # Update the current yaw

        if self.start_yaw is None:  # Set the initial yaw
            self.start_yaw = yaw

        # Calculate the angle turned in degrees
        #self.angle_turned = self.calculate_angle_turned(self.start_yaw, self.current_yaw)
    
    def calculate_angle_turned(self, start_yaw, current_yaw):
        angle = (current_yaw - start_yaw) * (180 / 3.14159)  # Convert radians to degrees
        if angle < -2:
            angle += 360  # Normalize angle to [0, 360)
        return angle
    

    def run(self):
        twist = Twist()
        # Wait for the first message to determine the initial state
        timeout = rospy.Time.now() + rospy.Duration(2)  # 2-second timeout
        while not self.initial_check_done and rospy.Time.now() < timeout:
            rospy.loginfo("Waiting for the initial person detection status...")
            rospy.sleep(0.5)
        if not self.initial_check_done:
            rospy.logwarn("No person detection data received. Exiting...")
            return

        rospy.loginfo("Start pub ang vel")
        acc_start_yaw = self.current_yaw
        total_acc_yaw = 0
        while not rospy.is_shutdown() and total_acc_yaw < 350:
            twist.angular.z = 0.3
            self.vel_pub.publish(twist)   #keep echoing the ang vel to the robot if no detection
            acc_current_yaw = self.current_yaw
            total_acc_yaw = self.calculate_angle_turned(acc_current_yaw, acc_start_yaw)
            if self.person_centered:      
                twist.angular.z = 0       #stop echoing if there's detection
                self.vel_pub.publish(twist)
                self.start_yaw = self.current_yaw  #find the current orientation
                rospy.loginfo("Hello")
                rospy.sleep(3)
                angle = self.calculate_angle_turned(self.start_yaw, self.current_yaw)   ########add time out feature  
                while angle < 15:
                    rospy.loginfo(f"start forced rotation, {angle}")
                    twist.angular.z = 0.31
                    self.vel_pub.publish(twist)
                    angle = self.calculate_angle_turned(self.start_yaw, self.current_yaw)                      
                    self.rate.sleep() 
            self.rate.sleep()  

    def rotate_robot(self, angular_velocity):  #method to be called by run
        twist = Twist()
        twist.angular.z = angular_velocity
        self.vel_pub.publish(twist)

 
if __name__ == '__main__':
    main_greeting = MainGreeting()
    main_greeting.run()
