#!/usr/bin/env python3
#
#   mapping.py
#
#   Demostrate how to map pixel coordinates into world coordinates.
#
#   Node:           /mapper
#   Subscribers:    /usb_cam/image_raw          Source image
#   Publishers:     /mapper/image_raw           Debug (marked up) image
#
import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image
from geometry_msgs.msg  import Point


#
#  Demo Node Class
#
class DemoNode(Node):
    # Pick some colors, assuming RGB8 encoding.
    red    = (255,   0,   0)
    green  = (  0, 255,   0)
    blue   = (  0,   0, 255)
    yellow = (255, 255,   0)
    white  = (255, 255, 255)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a publisher for the processed (debugging) images.
        # Store up to three images, just in case.
        self.pubrgb = self.create_publisher(Image, name+'/image_raw', 3)
        self.hsvlimits = np.array([[10, 40], [60, 230], [103, 179]])

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process, 1)
        
        self.pubpoint = self.create_publisher(Point, "/point", 10)
#ros2 topic pub -1 /point geometry_msgs/msg/Point "{x: 0.35, y: 0.5, z: 0.2}"
        # Report.
        self.get_logger().info("Mapper running...")

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    def dist(self, point1, point2):
        return np.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
    
    def isRect(self, box):
        ratio = (self.dist(box[0], box[1]) / self.dist(box[2], box[1]))
        
        print("ratio", ratio)
        return ratio, ratio > 3 or ratio < 1/3

    def get_contours(self, frame):
         # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Convert into OpenCV image, using RGB 8-bit (pass-through).

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Cheat: swap red/blue

        # Grab the image shape, determine the center pixel.
        (H, W, D) = frame.shape
        uc = W//2
        vc = H//2

        # Help to determine the HSV range...
        if True:
            # Draw the center lines.  Note the row is the first dimension.
            frame = cv2.line(frame, (uc,0), (uc,H-1), self.white, 1)
            frame = cv2.line(frame, (0,vc), (W-1,vc), self.white, 1)

            # Report the center HSV values.  Note the row comes first.
            self.get_logger().info(
                "HSV = (%3d, %3d, %3d)" % tuple(hsv[vc, uc]))

        
        # Threshold in Hmin/max, Smin/max, Vmin/max
        binary = cv2.inRange(hsv, self.hsvlimits[:,0], self.hsvlimits[:,1])

        # Erode and Dilate. Definitely adjust the iterations!
        iter = 4
        binary = cv2.erode( binary, None, iterations=iter)
        binary = cv2.dilate(binary, None, iterations=2*iter)
        binary = cv2.erode( binary, None, iterations=iter)


        # Find contours in the mask and initialize the current
        # (x, y) center of the ball
        (contours, hierarchy) = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw all contours on the original image for debugging.
        #cv2.drawContours(frame, contours, -1, self.blue, 2)

        # Only proceed if at least one contour was found.  You may
        # also want to loop over the contours...
        if len(contours) > 0:
            # Pick the largest contour.
            for contour in contours:
                #contour = max(contours, key=cv2.contourArea)
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame,[box],0,(0,0,255),2)
# 
                center_x = int(np.average([x[0] for x in box]))
                center_y = int(np.average([x[1] for x in box]))

                
                ratio, isbox = self.isRect(box) 
                
                box_world = self.pixelToWorldbox(frame, box, 0.0, 0.5)
                found_none = False

                new_point = None
                for i in box_world:
                    if i is None:
                        found_none = True
                if isbox and not found_none:
                    if ratio < 1:
                        dx = box_world[1][0] - box_world[2][0]
                        dy = box_world[1][1] - box_world[2][1]
                    else:
                        dx = box_world[0][0] - box_world[1][0]
                        dy = box_world[0][1] - box_world[1][1]
                    angle = np.degrees(np.arctan2(dy, dx))
                elif not isbox and not found_none:
                    ((ur, vr), radius) = cv2.minEnclosingCircle(contour)
                    new_point = Point()
                    new_point.x = float(ur)
                    new_point.y = float(vr)
                    new_point.z = float(0.0)

                    self.pubpoint.publish(new_point)
                    
                    #print("angle", np.degrees(np.arctan2(dy, dx)))


                # # Find the enclosing circle (convert to pixel values)
                # ((ur, vr), radius) = cv2.minEnclosingCircle(contour)
                # ur     = int(ur)
                # vr     = int(vr)
                # radius = int(radius)

                # Draw the circle (yellow) and centroid (red) on the
                # original image.
                # cv2.circle(frame, (ur, vr), int(radius), self.yellow,  2)
                # cv2.circle(frame, (ur, vr), 5,           self.red,    -1)

            # Report.
            #self.get_logger().info(
                # "Found Ball enclosed by radius %d about (%d,%d)" %
                # (radius, ur, vr))

    # Pixel Conversion
    def pixelToWorldbox(self, image, box, x0, y0, annotateImage=False):
        '''
        Convert the (u,v) pixel position into (x,y) world coordinates
        Inputs:
          image: The image as seen by the camera
          u:     The horizontal (column) pixel coordinate
          v:     The vertical (row) pixel coordinate
          x0:    The x world coordinate in the center of the marker paper
          y0:    The y world coordinate in the center of the marker paper
          annotateImage: Annotate the image with the marker information

        Outputs:
          point: The (x,y) world coordinates matching (u,v), or None

        Return None for the point if not all the Aruco markers are detected
        '''

        # Detect the Aruco markers (using the 4X4 dictionary).
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(
            image, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        if annotateImage:
            cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)

        # Abort if not all markers are detected.
        if (markerIds is None or len(markerIds) != 4 or
            set(markerIds.flatten()) != set([1,2,3,4])):

            return None


        # Determine the center of the marker pixel coordinates.
        uvMarkers = np.zeros((4,2), dtype='float32')
        for i in range(4):
            uvMarkers[markerIds[i]-1,:] = np.mean(markerCorners[i], axis=1)

        # Calculate the matching World coordinates of the 4 Aruco markers.
        DX = 0.1016
        DY = 0.06985
        xyMarkers = np.float32([[x0+dx, y0+dy] for (dx, dy) in
                                [(-DX, DY), (DX, DY), (-DX, -DY), (DX, -DY)]])


        # Create the perspective transform.
        M = cv2.getPerspectiveTransform(uvMarkers, xyMarkers)

        # Map the object in question.
        box_world = []
        for u, v in box:
            uvObj = np.float32([u, v])
            xyObj = cv2.perspectiveTransform(uvObj.reshape(1,1,2), M).reshape(2)


            # Mark the detected coordinates.
            if annotateImage:
                # cv2.circle(image, (u, v), 5, (0, 0, 0), -1)
                s = "(%7.4f, %7.4f)" % (xyObj[0], xyObj[1])
                
                cv2.putText(image, s, (u-80, v-8), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 0, 0), 2, cv2.LINE_AA)
            box_world.append(xyObj)

        return box_world


    # Process the image (detect the ball).
    def process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.get_contours(image)

        # Grab the image shape, determine the center pixel.
        (H, W, D) = image.shape
        uc = W//2
        vc = H//2

        # Assume the center of marker sheet is at the world origin.
        x0 = 0.0
        y0 = 0.50

        # Convert the center of the image into world coordinates.
        # xyCenter = self.pixelToWorld(image, uc, vc, x0, y0)

        
        # Mark the center of the image.
        cv2.circle(image, (uc, vc), 5, self.red, -1)

        # Report the mapping.
        # if xyCenter is None:
        #     self.get_logger().info("Unable to execute mapping")
        # else:
        #     (xc, yc) = xyCenter
        #     self.get_logger().info("Camera pointed at (%f,%f)" % (xc, yc))

        # Convert the image back into a ROS image and republish.
        self.pubrgb.publish(self.bridge.cv2_to_imgmsg(image, "rgb8"))


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = DemoNode('mapper')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
