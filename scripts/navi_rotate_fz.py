#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
import tf.transformations as tf_trans
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion

class MainGreeting:     
    def __init__(self):
        rospy.init_node('main_greeting', anonymous=True)

        # Subscribers
        #self.person_centered_sub = rospy.Subscriber('/person_centered', Bool, self.person_centered_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Publishers
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.activate_pub = rospy.Publisher('/activate_model', Bool, queue_size=10)
        
        # Class attributes
        self.person_centered = False
        self.initial_check_done = False
        self.start_yaw = None
        self.current_yaw = None
        self.angle_turned = 0
        self.current_room = None  # Store current room or location here
        
        self.rate = rospy.Rate(10)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()


    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    """
    def person_centered_callback(self, msg):
        self.person_centered = msg.data
        self.initial_check_done = True
    """
    def odom_callback(self, msg):
        # Extract orientation in quaternion and convert to Euler angles
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_trans.euler_from_quaternion(orientation_list)
        
        self.current_yaw = yaw  # Update the current yaw

        if self.start_yaw is None:
            self.start_yaw = yaw

    def calculate_angle_turned(self, start_yaw, current_yaw):
        angle = (current_yaw - start_yaw) * (180 / 3.14159)  # Convert radians to degrees
        if angle < -2:
            angle += 360  # Normalize angle to [0, 360)
        return angle
    """
    def rotate_robot(self):
        twist = Twist()
        
        # Wait for the first message to determine the initial state
        timeout = rospy.Time.now() + rospy.Duration(2)  # 2-second timeout
        while not self.initial_check_done and rospy.Time.now() < timeout:
            rospy.loginfo("Waiting for the initial person detection status...")
            rospy.sleep(0.5)
        if not self.initial_check_done:
            rospy.logwarn("No person detection data received. Exiting...")
            return
        
        
        rospy.loginfo("Start rotating")
        acc_start_yaw = self.current_yaw
        total_acc_yaw = 0
        while (not rospy.is_shutdown()) and total_acc_yaw <= 86:
            twist.angular.z = 0.3
            self.vel_pub.publish(twist)
            print(f"the angle turned is {total_acc_yaw}")
            acc_current_yaw = self.current_yaw
            total_acc_yaw = self.calculate_angle_turned(acc_current_yaw, acc_start_yaw)
            if self.person_centered:
                twist.angular.z = 0
                self.vel_pub.publish(twist)
                self.start_yaw = self.current_yaw
                rospy.loginfo("Hello")
                rospy.sleep(3)
                angle = self.calculate_angle_turned(self.start_yaw, self.current_yaw)
                while angle < 15:
                    rospy.loginfo(f"start forced rotation, {angle}")
                    twist.angular.z = 0.31
                    self.vel_pub.publish(twist)
                    angle = self.calculate_angle_turned(self.start_yaw, self.current_yaw)
                    self.rate.sleep()
            self.rate.sleep()
        """
    def rotation_angle(self, ang_speed, max_acc_angle):
        move_cmd = Twist()
        move_cmd.angular.z = ang_speed if max_acc_angle > 0 else -ang_speed  # Adjust direction based on angle sign

        acc_start_angle = self.current_yaw  # Use current_yaw directly
        max_acc_rad = math.radians(max_acc_angle)

        while not rospy.is_shutdown():
            current_angle = self.current_yaw  # Use current_yaw directly
            acc_angle_turned = self.normalize_angle(current_angle - acc_start_angle)
            
            rospy.loginfo(f"Current angle: {math.degrees(current_angle):.2f}, "
                          f"Accumulated angle turned: {math.degrees(acc_angle_turned):.2f}, "
                          f"Target angle: {max_acc_angle}")

            if abs(acc_angle_turned) >= abs(max_acc_rad):  # Use absolute values for comparison
                break

            self.vel_pub.publish(move_cmd)  # Make sure you're publishing to the correct topic
            self.rate.sleep()

        move_cmd.angular.z = 0
        self.vel_pub.publish(move_cmd)

    def send_waypoint(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = waypoint

        rospy.loginfo(f"Sending waypoint {waypoint}")
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(80.0))  # Wait for a max of 40 seconds
        rospy.loginfo(f"Waypoint {waypoint} reached or timeout reached")



    def trigger_resumable_rotation(self):
        """Use subprocess to call the resumable rotation script"""
        rospy.loginfo("Triggering resumable rotation script via subprocess...")
        
        try:
            # Call resumable_rotation_fz.py using subprocess
            subprocess.run(["rosrun", "ffm_pkg", "resumable_rotation_fz.py"])
            rospy.loginfo("Resumable rotation script completed.")
        except Exception as e:
            rospy.logerr(f"Failed to trigger resumable rotation script: {e}")

    def run(self):
        self.rotation_angle(0.5,90)
        waypoint = Pose(position=Point(-2.95,3.5,0.00035), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))  #study room
        self.send_waypoint(waypoint)
        waypoint = Pose(position=Point(-1.28,4.31,0.00203), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)) #house center
        self.send_waypoint(waypoint)
        waypoint = Pose(position=Point(-0.576,5.35,0.00314), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))  #kitchen entrance
        self.send_waypoint(waypoint)
        waypoint = Pose(position=Point(0.846,6.78,0.00218), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))  #kitchen
        self.send_waypoint(waypoint)  # Go to the waypoint

        self.current_room = "Kitchen"  # Manually set the room name
        rospy.loginfo(f"Robot is now in {self.current_room}")

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Waypoint reached successfully.")
            self.trigger_resumable_rotation()  # Trigger resumable rotation after reaching waypoint
        else:
            rospy.logwarn("Failed to reach waypoint within the time limit.")

        rospy.loginfo("Reached waypoint, starting rotation")

        self.rotation_angle(0.5,90)
        waypoint = Pose(position=Point(-0.576,5.35,0.00314), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))  #kitchen entrance
        self.send_waypoint(waypoint)
        waypoint = Pose(position=Point(-0.571,3.56,0.00136), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)) #living room entrance
        self.send_waypoint(waypoint)
        waypoint = Pose(position=Point(0.659,3.62,0.0052), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)) #living room 
        self.send_waypoint(waypoint)

        self.current_room = "Living Room"  # Manually set the room name
        rospy.loginfo(f"Robot is now in {self.current_room}")

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Waypoint reached successfully.")
            self.trigger_resumable_rotation()  # Trigger resumable rotation after reaching waypoint
        else:
            rospy.logwarn("Failed to reach waypoint within the time limit.")

        rospy.loginfo("Reached waypoint, starting rotation")

        waypoint = Pose(position=Point(-0.571,3.56,0.00136), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)) #living room entrance
        self.send_waypoint(waypoint)
        waypoint = Pose(position=Point(-1.28,4.31,0.00203), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)) #house center
        self.send_waypoint(waypoint)
        waypoint = Pose(position=Point(-2.95,3.5,0.00035), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))  #study room
        self.send_waypoint(waypoint) 
        waypoint = Pose(position=Point(-2.45,6.45,0.00482), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)) #bedroom
        self.send_waypoint(waypoint) 

        self.current_room = "Bedroom"  # Manually set the room name
        rospy.loginfo(f"Robot is now in {self.current_room}")

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Waypoint reached successfully.")
            self.trigger_resumable_rotation()  # Trigger resumable rotation after reaching waypoint
        else:
            rospy.logwarn("Failed to reach waypoint within the time limit.")

        rospy.loginfo("Reached waypoint, starting rotation")

        waypoint = Pose(position=Point(-3.92,5.34,0.000201), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))  #bedroom entrance
        self.send_waypoint(waypoint)
        waypoint = Pose(position=Point(-2.95,3.5,0.00035), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))  #study room
        self.send_waypoint(waypoint)
        waypoint = Pose(position=Point(-4.35,3,0.000388), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))  #bedroom entrance
        self.send_waypoint(waypoint) 

        self.current_room = "Bedroom"  # Manually set the room name
        rospy.loginfo(f"Robot is now in {self.current_room}")

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Waypoint reached successfully.")
            self.trigger_resumable_rotation()  # Trigger resumable rotation after reaching waypoint
        else:
            rospy.logwarn("Failed to reach waypoint within the time limit.")

        rospy.loginfo("Reached waypoint, starting rotation")




if __name__ == '__main__':
    main_greeting = MainGreeting()
    

    # Run the process (go to waypoint and then rotate)
    main_greeting.run()
