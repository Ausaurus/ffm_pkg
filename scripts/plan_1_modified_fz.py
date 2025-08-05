#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
from tf.transformations import euler_from_quaternion
import subprocess
import os
import time

class TurtleBotController:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.activate_model_pub = rospy.Publisher('/activate_model', Bool, queue_size=10)
        self.resume_rotation_pub = rospy.Publisher('/resume_rotation', Bool, queue_size=10)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.person_center_sub = rospy.Subscriber('/person_centered', Bool, self.person_center_callback)
        self.resume_detection_sub = rospy.Subscriber('/resume_rotation', Bool, self.resume_detection_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.current_pose = None
        self.interrupt = False
        self.waiting_for_resume = False
        self.photo_taken = False  # To ensure gemini_photo is triggered only once per detection
        self.movement_paused = False  # Indicates if the robot's movement is paused
        self.last_photo_time = 0  # Timestamp for the last time photo was taken to debounce

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def person_center_callback(self, msg):
        if msg.data and not self.photo_taken:
            # Ensure that gemini_photo is not triggered twice in quick succession
            current_time = time.time()
            if current_time - self.last_photo_time > 4.0:  # 2-second debounce window
                rospy.loginfo("Person centered detected, stopping movement and triggering gemini_photo.py.")
                self.handle_interrupt()  # Call a unified interrupt handler
                self.last_photo_time = current_time
            else:
                rospy.loginfo("Person detection ignored due to debounce.")

    def resume_detection_callback(self, msg):
        if msg.data:
            rospy.loginfo("Resuming movement after gemini_photo process.")
            self.movement_paused = False  # Allow the movement to resume
            self.photo_taken = False  # Reset the photo flag after movement resumes

    def trigger_gemini_photo(self):
        """Trigger gemini_photo.py script."""
        gemini_photo_path = os.path.join('/home/i_h8_ros/catkin_ws/src/fz_gemini/scripts', 'gemini_photo.py')
        rospy.loginfo("Triggering gemini_photo.py process.")

        # Set environment variable to track guest
        env = os.environ.copy()
        env['GUEST_ID'] = str(1)  # Hardcoded to 1 for simplicity. Modify as needed.

        # Run gemini_photo.py
        subprocess.run([gemini_photo_path], env=env)
        rospy.loginfo("gemini_photo.py process completed.")

        self.photo_taken = True  # Mark that the photo has been taken

        # Signal to resume rotation/movement
        self.resume_rotation_pub.publish(True)

    def handle_interrupt(self):
        """Unified interrupt handling for both linear movement and rotation."""
        if not self.interrupt:
            rospy.loginfo("Handling interrupt to stop both movement and rotation.")
            self.cmd_vel_pub.publish(Twist())  # Stop both linear movement and rotation
            self.movement_paused = True  # Set the flag to pause movement
            self.interrupt = True  # Prevent further interrupts
            self.trigger_gemini_photo()  # Trigger Gemini photo process

    def linear_x(self, speed, max_acc_distance):
        while not self.current_pose:
            rospy.logwarn("Waiting for odometry data...")
            self.rate.sleep()

        acc_start_x = self.current_pose.position.x
        acc_start_y = self.current_pose.position.y
        move_cmd = Twist()
        move_cmd.linear.x = speed

        while not rospy.is_shutdown():
            if self.movement_paused:
                self.cmd_vel_pub.publish(Twist())  # Ensure the robot is stopped while paused
                rospy.loginfo("Movement paused, waiting for resume signal...")
                self.rate.sleep()
                continue

            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            acc_distance_moved = math.sqrt((current_x - acc_start_x)**2 + (current_y - acc_start_y)**2)

            if acc_distance_moved >= max_acc_distance:
                break

            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

        move_cmd.linear.x = 0
        self.cmd_vel_pub.publish(move_cmd)

    def rotation(self, ang_speed, max_acc_angle):
        while not self.current_pose:
            rospy.logwarn("Waiting for odometry data...")
            self.rate.sleep()

        move_cmd = Twist()
        move_cmd.angular.z = ang_speed if max_acc_angle > 0 else -ang_speed

        acc_start_angle = self.get_yaw()
        max_acc_rad = math.radians(max_acc_angle)

        while not rospy.is_shutdown():
            if self.movement_paused:
                self.cmd_vel_pub.publish(Twist())  # Ensure the robot is stopped while paused
                rospy.loginfo("Rotation paused, waiting for resume signal...")
                self.rate.sleep()
                continue

            current_angle = self.get_yaw()
            acc_angle_turned = self.normalize_angle(current_angle - acc_start_angle)

            rospy.loginfo(f"Current angle: {math.degrees(current_angle):.2f}, "
                        f"Accumulated angle turned: {math.degrees(acc_angle_turned):.2f}, "
                        f"Target angle: {max_acc_angle}")

            if abs(acc_angle_turned) >= abs(max_acc_rad):
                break

            print(f"the paused variable is {self.movement_paused}")
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

        move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(move_cmd)

    def get_yaw(self):
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def execute_movement_plan(self):
        while not self.current_pose:
            rospy.logwarn("Waiting for odometry data...")
            self.rate.sleep()

        #self.linear_x(0.15, 0.7)
        #self.rotation(0.4, -87)
        #rospy.sleep(1)
        #self.linear_x(0.15, 1.0)
        #self.rotation(0.4, 81)
        #rospy.sleep(1)
        #self.linear_x(0.15, 1.0)
        #self.rotation(0.4, -9)
        #rospy.sleep(1)
        #self.linear_x(0.15, 1.6)
        #rospy.loginfo("Movement plan finished.")

        self.linear_x(0.15,2)
        rospy.sleep(0.5)
        self.rotation(0.4,-90)
        rospy.sleep(0.5)
        self.linear_x(0.15,1)
        rospy.sleep(0.5)
        self.linear_x(0.15,0.8)
        rospy.sleep(0.5)
        self.rotation(0.4,-90)
        rospy.sleep(0.5)
        self.linear_x(0.15,0.8)
        rospy.sleep(0.5)
        self.linear_x(0.15,0.7)
        rospy.sleep(0.5)
        self.rotation(0.4,-90)
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        controller = TurtleBotController()
        controller.execute_movement_plan()
    except rospy.ROSInterruptException:
        pass

