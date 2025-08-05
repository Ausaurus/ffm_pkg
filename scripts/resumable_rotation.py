#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as tf_trans
import subprocess
import os

class MainGreeting:
    def __init__(self):
        rospy.init_node('main_greeting', anonymous=True)
        self.person_centered_sub = rospy.Subscriber('/person_centered', Bool, self.person_centered_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.activate_pub = rospy.Publisher('/activate_model', Bool, queue_size=10)
        self.resume_rotation_sub = rospy.Subscriber('/resume_rotation', Bool, self.resume_rotation_callback)

        self.person_centered = False
        self.initial_check_done = False
        self.start_yaw = None
        self.current_yaw = None
        self.angle_turned = 0
        self.resume_rotation = False
        self.waypoint_paused = False  # Flag to pause waypoint navigation

        self.rate = rospy.Rate(10)

    def person_centered_callback(self, msg):
        self.person_centered = msg.data
        self.initial_check_done = True

    def resume_rotation_callback(self, msg):
        self.resume_rotation = msg.data

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = tf_trans.euler_from_quaternion(orientation_list)

        self.current_yaw = yaw

        if self.start_yaw is None:
            self.start_yaw = yaw

    def calculate_angle_turned(self, start_yaw, current_yaw):
        angle = (current_yaw - start_yaw) * (180 / 3.14159)
        if angle < -2:
            angle += 360
        return angle

    def trigger_gemini_photo(self):
        gemini_photo_path = os.path.join('/home/i_h8_ros/catkin_ws/src/fz_gemini/scripts', 'gemini_photo.py')
        env = os.environ.copy()
        env['GUEST_ID'] = str(1)  # Assuming a guest ID of 1 for this example

        rospy.loginfo("Triggering gemini_photo.py...")
        subprocess.run([gemini_photo_path], env=env)
        rospy.loginfo("gemini_photo.py process completed.")

    def run(self):
        twist = Twist()
        timeout = rospy.Time.now() + rospy.Duration(2)
        while not self.initial_check_done and rospy.Time.now() < timeout:
            rospy.loginfo("Waiting for the initial person detection status...")
            rospy.sleep(0.5)
        if not self.initial_check_done:
            rospy.logwarn("No person detection data received. Exiting...")
            return

        rospy.loginfo("Start publishing angular velocity")
        acc_start_yaw = self.current_yaw
        total_acc_yaw = 0
        while not rospy.is_shutdown() and total_acc_yaw < 350:
            twist.angular.z = 0.3
            self.vel_pub.publish(twist)
            acc_current_yaw = self.current_yaw
            total_acc_yaw = self.calculate_angle_turned(acc_current_yaw, acc_start_yaw)
            if self.person_centered:
                twist.angular.z = 0
                self.vel_pub.publish(twist)
                self.start_yaw = self.current_yaw

                self.activate_pub.publish(True)
                rospy.loginfo("Published True to /activate_model")

                rospy.loginfo("Waiting for gemini_photo.py to complete...")
                self.trigger_gemini_photo()  # Call the gemini_photo.py script

                # Wait until the resume rotation signal is received
                rospy.loginfo("Waiting for /resume_rotation to be True...")
                while not self.resume_rotation and not rospy.is_shutdown():
                    self.rate.sleep()

                rospy.loginfo("Resuming rotation after receiving /resume_rotation signal.")

                angle = self.calculate_angle_turned(self.start_yaw, self.current_yaw)
                while angle < 15:
                    rospy.loginfo(f"Start forced rotation, {angle}")
                    twist.angular.z = 0.31
                    self.vel_pub.publish(twist)
                    angle = self.calculate_angle_turned(self.start_yaw, self.current_yaw)
                    self.rate.sleep()
            self.rate.sleep()

    def rotate_robot(self, angular_velocity):
        twist = Twist()
        twist.angular.z = angular_velocity
        self.vel_pub.publish(twist)

if __name__ == '__main__':
    main_greeting = MainGreeting()
    main_greeting.run()
