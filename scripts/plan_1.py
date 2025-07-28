#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
from tf.transformations import euler_from_quaternion

class TurtleBotController:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.person_center_sub = rospy.Subscriber('/person_centered', Bool, self.person_center_callback)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        self.current_pose = None
        self.interrupt = False

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def person_center_callback(self, msg):
        if msg.data:
            self.interrupt = True

    def linear_x(self, speed, max_acc_distance):
        while not self.current_pose:
            rospy.logwarn("linear: odometry not received yet!")
            self.rate.sleep()
        
        acc_start_x = self.current_pose.position.x  #acc stands for accumuluated distance calculation
        acc_start_y = self.current_pose.position.y
        move_cmd = Twist()
        move_cmd.linear.x = speed

        while not rospy.is_shutdown():
            if self.interrupt:
                self.handle_interrupt()
                self.force_moving(acc_start_x,acc_start_y, max_acc_distance)
                self.interrupt = False

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
            rospy.logwarn("rotation: odometry not received yet!")
            self.rate.sleep()
        
        move_cmd = Twist()
        move_cmd.angular.z = ang_speed if max_acc_angle > 0 else -ang_speed  # Adjust direction based on angle sign

        acc_start_angle = self.get_yaw()
        max_acc_rad = math.radians(max_acc_angle)

        while not rospy.is_shutdown():
            if self.interrupt:
                self.handle_interrupt()
                self.force_rotating(acc_start_angle, max_acc_rad)
                self.interrupt = False

            current_angle = self.get_yaw()
            acc_angle_turned = self.normalize_angle(current_angle - acc_start_angle)
            
            rospy.loginfo(f"Current angle: {math.degrees(current_angle):.2f}, "
                        f"Accumulated angle turned: {math.degrees(acc_angle_turned):.2f}, "
                        f"Target angle: {max_acc_angle}")

            if abs(acc_angle_turned) >= abs(max_acc_rad):  # Use absolute values for comparison
                break

            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

        move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(move_cmd)


    def force_moving(self, acc_start_x,acc_start_y, max_acc_distance, max_distance=0.40):
        # Define the movement command
        move_cmd = Twist()
        move_cmd.linear.x = 0.1 

        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        rospy.loginfo("start forced moving")
        
        # Move for a short distance to ensure the guest is away from the center
        while True:
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            distance_moved = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            acc_distance_moved =  math.sqrt((current_x - acc_start_x)**2 + (current_y - acc_start_y)**2)
            rospy.loginfo(f"distance moved: {distance_moved:.2f}")

            if (distance_moved >= max_distance) or (acc_distance_moved >= max_acc_distance):
                break

        # Stop the robot after reaching the target distance or after the loop completes
        move_cmd.linear.x = 0
        self.cmd_vel_pub.publish(move_cmd)
        rospy.loginfo("Forced moving completed")


    def force_rotating(self,acc_start_angle, max_acc_rad, max_angle=math.radians(10)):
        # Define the movement command
        move_cmd = Twist()
        move_cmd.angular.z = 0.1  # Set a small angular speed

        start_angle = self.get_yaw()

        # Rotate for a short angle to ensure the guest is away from the center
        #while not rospy.is_shutdown():
        while True:
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

            current_angle = self.get_yaw()
            angle_turned = self.normalize_angle(current_angle - start_angle)
            acc_angle_turned = self.normalize_angle(current_angle - acc_start_angle)
            if (abs(angle_turned) >= max_angle) or (abs(acc_angle_turned) >= max_acc_rad):    
                break
        #    break
        # Stop the robot
        move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(move_cmd)
        rospy.loginfo("Forced moving completed")

    def get_yaw(self):
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def handle_interrupt(self):
        rospy.loginfo("Hello")
        self.cmd_vel_pub.publish(Twist())  # Stop the robot
        rospy.sleep(5)
        self.interrupt = False  # Reset interrupt flag

    def execute_movement_plan(self):
        while not self.current_pose:
            rospy.logwarn("Odometry not received yet!!!")
            self.rate.sleep()

        self.rotation(0.5, 90)
        self.linear_x(0.2, 1.2)
        rospy.sleep(1)
        self.rotation(0.5, 90)
        self.linear_x(0.2, 1.2)
        rospy.sleep(1)
        self.rotation(0.5, -90)
        self.linear_x(0.2, 1.2)
        #self.rotation(0.5, -45)
        #self.rotation(0.5, -45)
        #self.linear_x(0.2, 270)


        """      
        self.rotation(0.5, -45)
        self.linear_x(0.2, 1.2)
        rospy.sleep(3)  
        self.rotation(0.5, -5)
        self.linear_x(0.2, 2.2)  
        self.rotation(0.5, 35)
        rospy.sleep(1)
        self.linear_x(0.2, 1.8)
        self.rotation(0.5, -5)
        rospy.sleep(1)
        self.linear_x(0.2, 0.6)
        self.linear_x(0.2, 0.6)
        self.rotation(0.5, 160)
        self.linear_x(0.2, 1.2)
        self.linear_x(0.2, 0.8)    
        self.linear_x(0.2,1.5)
        self.rotation(0.4, 90)
        rospy.sleep(2)
        self.linear_x(0.15, 0.8)
        #rospy.loginfo("Finish")
        """

if __name__ == '__main__':
    try:
        controller = TurtleBotController()
        controller.execute_movement_plan()
    except rospy.ROSInterruptException:
        pass