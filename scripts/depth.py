#!/usr/bin/env python3

import pyrealsense2 as rs
import mediapipe as mp
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from scipy.ndimage import median_filter
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import *
rospy.init_node("zed_filter")
bridge = CvBridge()
context = rs.context()

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Get the list of connected devices
devices = context.query_devices()

if len(devices) == 0:
    print("No device connected")
else:
    print("Connected device:")
    for device in devices:
        print(device.get_info(rs.camera_info.name), ":", device.get_info(rs.camera_info.serial_number))

# Create a pipeline
pipeline = rs.pipeline()
align_to = rs.stream.color
align = rs.align(align_to)

# Create a config and configure the pipeline to stream different formats
config = rs.config()

# Enable depth and color streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
pub_cv2image = rospy.Publisher("/zed2/zed_node/rgb_raw/image_raw_color", Image, queue_size=0)
pub_depthimage = rospy.Publisher("/zed2/zed_node/depth/depth_registered", Image, queue_size=0)

def display(input, title):
    normalized_input = cv2.normalize(input, None, 0, 255, cv2.NORM_MINMAX)

    # Convert the normalized depth image to an 8-bit image
    input_8bit = np.uint8(normalized_input)

    # Display the normalized depth image
    cv2.imshow(title, input_8bit)

def shutdown():
    rospy.sleep(1)
    rospy.signal_shutdown("shutdown so that characteristics_rs.py can open realsense")
    return TriggerResponse(success=True, message="depth closed")
service = rospy.Service("shut_depth", Trigger, shutdown)
#---------------------------------------------------------------------------------------------------------------------------------

while True:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not depth_frame or not color_frame:
        print("There's no depth/color frame")
        continue

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    bgr_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
    ros_image = bridge.cv2_to_imgmsg(color_image, "bgr8")
    depth_msg = bridge.cv2_to_imgmsg(depth_image.astype(np.float32), "32FC1")
    # display(depth_image,"depth_ori")
    # display(color_image,"color_ori")


    # Create a mask based on the depth data where objects within 1 meter are shown
    depth_mask = np.where((depth_image > 0) & (depth_image <= 10000), depth_image, 0).astype(np.uint16)
    #depth_image_filtered = cv2.guidedFilter(depth_image, depth_image, radius=5, eps=1e-2)
    # display(depth_mask,"depth_filtered")

    # Apply the depth mask to the RGB image
    #masked_color_image = cv2.bitwise_and(color_image, color_image, mask=depth_mask)
    pub_depthimage.publish(depth_msg)
    pub_cv2image.publish(ros_image)


    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

