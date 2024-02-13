import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image, JointState


from bmoHanoi.TrajectoryUtils import *
from bmoHanoi.TransformHelpers import *

RATE = 100
GO_TO_SHOULDER = 10
GO_TO_IDLE     = 10
TIME_TEST = 20
GO_TO_POINT = 10
class BmoHanoi(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)
        self.jointPos0 = self.grabfbk()
        self.starting = True
        # Thresholds in Hmin/max, Smin/max, Vmin/max
        self.hsvlimits = np.array([[20, 30], [90, 170], [60, 255]])


        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        self.pubrgb = self.create_publisher(Image, '/camera/color/display_image', 3)

        # Create Subscribers for image
        self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', 
                                                self.rgb_process, 1)
        self.sub_depth = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw',
                                                self.depth_process, 1)
        
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while(not self.count_subscribers('/joint_commands')):
            pass

        # Create a subscriber to continually receive joint state messages.
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)


        # Create a timer to keep calculating/sending commands.
        rate       = RATE
        self.t = 0.0
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        self.start_time = sec + nano*10**(-9)



        # Create a timer to keep calculating/sending commands.
        self.qshape = (5, 1)

        self.zeros = np.array([0.0, 0.0, 0.0, 0.0, 0.0]).reshape(self.qshape)
        self.jointVel0 = self.zeros
        print(self.jointVel0)
        self.init_joints = False

        self.cmdmsg = JointState()

        # Report.
        self.centerColor = [0,0,0]
        self.centerDist = 0
        self.timer = self.create_timer(1/rate, self.sendCmd)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))

        self.testArr = np.array([
            [0.0, -90],
            [0.0, -60],
            [0.0, -30],
            [0.0,   0.0],
            [30,   0.0],
            [30, -30],
            [30, -60],
            [30, -90],
            [30, -60],
            [30, -30],
            [30,   0.0],
            [60, -30],
            [60, -45],
        ])
        self.testArr = np.array([np.radians(x) for x in self.testArr])
        self.test_idx = -1
        self.positionCmds = []
        self.actualPos = []
        self.actEff = []

    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['base', 'shoulder', 'elbow', 'wrist', 'head']

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    def recvfbk(self, fbkmsg):
        self.actualJointPos   = list(fbkmsg.position)
        self.actualJointVel   = list(fbkmsg.velocity)
        self.actualJointEff   = list(fbkmsg.effort)
    # Process the depth image (detect the face).
        
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        def cb(fbkmsg):
            self.grabpos   = list(fbkmsg.position)
            self.grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)


        # Return the values.
        return self.grabpos
    
    def depth_process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "16UC1")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Extract the depth image information (distance in mm as uint16).
        width  = msg.width
        height = msg.height
        depth  = np.frombuffer(msg.data, np.uint16).reshape(height, width)

        # Report.
        col = width//2
        row = height//2
        self.centerDist   = depth[row][col]

    # Process the image (detect the ball).
    def rgb_process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Threshold in Hmin/max, Smin/max, Vmin/max
        binary = cv2.inRange(hsv, self.hsvlimits[:,0], self.hsvlimits[:,1])

        # Grab the image shape, determine the center pixel.
        (H, W, D) = frame.shape
        uc = W//2
        vc = H//2

        # Draw the center lines.  Note the row is the first dimension.
        frame = cv2.line(frame, (uc,0), (uc,H-1), (255, 255, 255), 1)
        frame = cv2.line(frame, (0,vc), (W-1,vc), (255, 255, 255), 1)

        self.centerColor = hsv[vc, uc]

        # Convert the frame back into a ROS image and republish.
        self.pubrgb.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))


    def gravityCalc(self):
        pass

    def goto_idle(self, t):
        if t < GO_TO_SHOULDER:
            desired = self.jointPos0[:]
            desired[1] = 0.0

            q, qdot = spline(t, GO_TO_SHOULDER, np.array(self.jointPos0).reshape(self.qshape), 
                    np.array(desired).reshape(self.qshape), 
                    np.zeros(self.qshape), 
                    np.zeros(self.qshape))
            self.newJoint0 =  q
        else:
            desired = np.zeros(self.qshape)
            desired[2] = np.radians(-90.0)
            q, qdot = spline(t - GO_TO_SHOULDER, GO_TO_IDLE, 
                    np.array(self.newJoint0).reshape(self.qshape), 
                    np.array(desired).reshape(self.qshape), 
                    np.zeros(self.qshape), 
                    np.zeros(self.qshape)) 
            self.endIdlePos = self.actualJointPos
            if len(self.testArr) < 14:
                self.testArr = np.vstack([[desired[1, 0], desired[2, 0]], self.testArr])
        return list(q[:, 0]), list(qdot[:, 0])


    def performTest(self, t):

        q, qDot = spline(t, TIME_TEST, np.array(self.jointPos0).reshape(self.qshape), 
                self.idleJointPos.reshape(self.qshape), 
                np.array(self.jointVel0, dtype=float).reshape(self.qshape), 
                np.zeros(self.qshape, dtype=float))
        return q, qDot
    
    def saveFdbck(self,pd):
        self.positionCmds.append(pd)
        self.actualPos.append(self.actualJointPos)
        self.actEff.append(self.actualJointEff)


    def start_test(self, t):
        if self.test_idx < len(self.testArr)-1:

            start = self.endIdlePos[:]
            start[1] = self.testArr[self.test_idx][0]
            start[2] = self.testArr[self.test_idx][1]

            end = self.endIdlePos[:]
            end[1] = self.testArr[self.test_idx + 1][0]
            end[2] = self.testArr[self.test_idx + 1][1]
            q, qDot = spline(t, GO_TO_POINT, np.array(start).reshape(self.qshape), 
                    np.array(end).reshape(self.qshape), 
                    np.zeros(self.qshape, dtype=float), 
                    np.zeros(self.qshape, dtype=float))
            self.saveFdbck(q)

            return list(q[:, 0]), list(qDot[:, 0])
        return None, None
        

    def sendCmd(self):
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        now = sec + nano*10**(-9)
        dt = (now - self.start_time) -self.t
        self.t = now - self.start_time

        if (self.t < GO_TO_SHOULDER + GO_TO_IDLE) and self.starting:
            pd, vd = self.goto_idle(self.t)
        else:
            if self.starting or self.t > GO_TO_POINT:
                self.starting = False
                sec, nano = self.get_clock().now().seconds_nanoseconds()
                self.start_time = sec + nano*10**(-9)
                self.t = 0.0
                self.test_idx += 1
            pd, vd = self.start_test(self.t)
            if pd == None:
                self.get_logger().info(f"Shapes {np.array(self.actEff).shape}, {np.array(self.positionCmds).shape}, "
                                     f"{np.array(self.actualPos).shape}")
                np.save('pos_cmds', np.array(self.positionCmds))
                np.save('act_pos', np.array(self.actualPos))
                np.save('act_eff', np.array(self.actEff))
                return None
        

        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = self.jointnames()
        self.cmdmsg.position     = pd
        self.cmdmsg.velocity     = vd
        self.cmdmsg.effort       = [np.nan, np.nan, np.nan, np.nan, np.nan]
        self.cmdpub.publish(self.cmdmsg)

#   Main Code
#
def main(args=None):

    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = BmoHanoi('bmoHanoi')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
