
import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image, JointState, CameraInfo
from geometry_msgs.msg  import Point

COLOR_HSV_MAP = {'blue': [(85, 118), (175,255), (59, 178)],
                 'green': [(40, 80), (55, 220), (35, 175)],
                  'yellow': [(15, 55), (65, 255), (150, 255)],
                   'orange': [(0, 15), (80, 255), (146, 255)],
                    'red': [(0,3), (151, 255), (90, 255)],
                    'black': [(74, 119), (11, 156), (0, 95)]}

class CameraNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        def cb(msg):
            self.camD = np.array(msg.d).reshape(5)
            self.camK = np.array(msg.k).reshape((3,3))
            self.camw = msg.width
            self.camh = msg.height
            self.caminfoready = True
        # Temporarily subscribe to get just one message.
        self.get_logger().info("Waiting for camera info...")
        sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', cb, 1)
        self.caminfoready = False
        while not self.caminfoready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub) 

        self.bridge = cv_bridge.CvBridge()
        self.pubrgb = self.create_publisher(Image, '/camera/color/display_image', 3)
        # Create Subscribers for image
        self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', 
                                                self.rgb_process, 1)
        self.sub_depth = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw',
                                                self.depth_process, 1)

        self.color_publishers = {}
        for color in COLOR_HSV_MAP:
            self.color_publishers[color] = self.create_publisher(Image,name + f'/{color}binary',    3)
        

        self.hsvImageMap = {}
        
    def depth_process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "16UC1")

        # Extract the depth image information (distance in mm as uint16).
        width  = msg.width
        height = msg.height
        depth  = np.frombuffer(msg.data, np.uint16).reshape(height, width)

        self.depthImage = depth

    def rgb_process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")


        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)


        for color in COLOR_HSV_MAP.keys():
            binary = cv2.inRange(hsv, np.array(COLOR_HSV_MAP[color])[:,0], 
                                    np.array(COLOR_HSV_MAP[color])[:,1])

            self.color_publishers[color].publish(self.bridge.cv2_to_imgmsg(binary))

            self.hsvImageMap[color] = binary
            

        # Grab the image shape, determine the center pixel.
        (H, W, D) = frame.shape
        uc = W//2
        vc = H//2

        # Draw the center lines.  Note the row is the first dimension.
        frame = cv2.line(frame, (W//3,0), (W//3,H-1), (255, 255, 255), 1)
        frame = cv2.line(frame, (2*W//3,0), (2*W//3,H-1), (255, 255, 255), 1)

        # Convert the frame back into a ROS image and republish.
        self.pubrgb.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))


    
def main(args=None):

    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = CameraNode('bmoHanoi')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()