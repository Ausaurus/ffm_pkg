#!/usr/bin/env python3

import pyrealsense2 as rs
import mediapipe as mp
import numpy as np
import cv2
from scipy.ndimage import median_filter

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

# Create a config and configure the pipeline to stream different formats
config = rs.config()

# Enable depth and color streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

def display(input, title):
    normalized_input = cv2.normalize(input, None, 0, 255, cv2.NORM_MINMAX)

    # Convert the normalized depth image to an 8-bit image
    input_8bit = np.uint8(normalized_input)

    # Display the normalized depth image
    cv2.imshow(title, input_8bit)
#---------------------------------------------------------------------------------------------------------------------------------

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            print("There's no depth/color frame")
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        display(depth_image,"depth_ori")
        display(color_image,"color_ori")


        # Create a mask based on the depth data where objects within 1 meter are shown
        depth_mask = np.where((depth_image > 0) & (depth_image <= 10000), depth_image, 0).astype(np.uint16)
        #depth_image_filtered = cv2.guidedFilter(depth_image, depth_image, radius=5, eps=1e-2)
        display(depth_mask,"depth_filtered")

        # Apply the depth mask to the RGB image
        #masked_color_image = cv2.bitwise_and(color_image, color_image, mask=depth_mask)


        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
