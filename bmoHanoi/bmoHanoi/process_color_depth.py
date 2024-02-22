import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image, JointState


def depth_process(node, msg):
    # Confirm the encoding and report.
    assert(msg.encoding == "16UC1")

    # Extract the depth image information (distance in mm as uint16).
    width  = msg.width
    height = msg.height
    depth  = np.frombuffer(msg.data, np.uint16).reshape(height, width)

    # Report.
    col = width//2
    row = height//2
    node.centerDist   = depth[row][col]

    # Process the image (detect the ball).
def rgb_process(node, msg):
    # Confirm the encoding and report.
    assert(msg.encoding == "rgb8")

    # Convert into OpenCV image, using RGB 8-bit (pass-through).
    frame = node.bridge.imgmsg_to_cv2(msg, "passthrough")

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Threshold in Hmin/max, Smin/max, Vmin/max
    binary = cv2.inRange(hsv, node.hsvlimits[:,0], node.hsvlimits[:,1])

    # Grab the image shape, determine the center pixel.
    (H, W, D) = frame.shape
    uc = W//2
    vc = H//2

    # Draw the center lines.  Note the row is the first dimension.
    frame = cv2.line(frame, (uc,0), (uc,H-1), (255, 255, 255), 1)
    frame = cv2.line(frame, (0,vc), (W-1,vc), (255, 255, 255), 1)

    node.centerColor = hsv[vc, uc]

    # Convert the frame back into a ROS image and republish.
    node.pubrgb.publish(node.bridge.cv2_to_imgmsg(frame, "rgb8"))