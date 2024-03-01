import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image, JointState, CameraInfo

from bmoHanoi.process_color_depth import rgb_process, depth_process
from bmoHanoi.TrajectoryUtils import *
from bmoHanoi.TransformHelpers import *

RATE = 100
GOTO_REC_T = 10
REC_T = 3
GOTO_PRI_T = 10
GRAB_T = 10
READY_T = 10
COLOR_HSV_MAP = {'blue': [(85, 118), (175,255), (59, 178)],
                 'green': [(40, 80), (55, 220), (35, 175)],
                  'yellow': [(15, 55), (65, 255), (150, 255)],
                   'orange': [(0, 15), (80, 255), (146, 255)],
                    'red': [(0,5), (160, 255), (70, 230)] 
                    }

TIME_DICT = {'GOTO_REC' : GOTO_REC_T, 
                          'REC' : REC_T, 
                          'GOTO_PRI' : GOTO_PRI_T,
                          'GRAB' : GRAB_T, 
                          'GOTO_READY': READY_T}
import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image, JointState


from bmoHanoi.TrajectoryUtils import *
from bmoHanoi.TransformHelpers import *
from bmoHanoi.KinematicChain import *

class BmoHanoi(Node):
    # Initialization.
    def __init__(self, name):
   
        # Initialize the node, naming it as specified
        super().__init__(name)
        def cb(msg):
            self.camD = np.array(msg.d).reshape(5)
            self.camK = np.array(msg.k).reshape((3,3))
            self.camw = msg.width
            self.camh = msg.height
            self.get_logger().info(f"bruhhhh{self.camw}")
            self.caminfoready = True
        # Temporarily subscribe to get just one message.
        self.get_logger().info("Waiting for camera info...")
        sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', cb, 1)
        self.caminfoready = False
        while not self.caminfoready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)  


        self.chain = KinematicChain('world', 'tip', self.jointnames()[:5])
        self.camChain = KinematicChain('world', 'camera', self.jointnames()[:5])


        self.T = 0
        # Create a timer to keep calculating/sending commands.
        rate       = RATE
        self.t = 0.0
        self.dt = 0.0
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        self.start_time = sec + nano*10**(-9)

        self.prSt = 'GOTO_REC'
        self.nxSt = 'GRAB'
        self.priorityDonut = None

        self.actualJointPos   = self.grabfbk()
        self.actualJointVel   = None
        self.actualJointEff   = None

        self.initJointPos     = self.actualJointPos[:5]
        self.taskShape = (5, 1)
        self.jointShape = (5, 1)

        self.q = np.array(self.actualJointPos[:5]).reshape(self.jointShape)


        self.recon_joint = np.array([0.0, -.27, -1.47, -.15 + .58, 0.0]).reshape(self.taskShape)
        self.taskPosition0, self.taskOrientation0, _, _ = self.chain.fkin(self.q)
        self.Rd = self.taskOrientation0

        self.pd = self.taskPosition0[:]
        self.vd = np.zeros((3, 1))
        self.wd = np.zeros((2, 1))

        self.hsvlimits = np.array(COLOR_HSV_MAP['orange'])

        self.readyJointState = np.radians(np.array([0.0, 0.0, -90.0, -90, 0.0]))
        self.ready_Rd = Rotz(np.radians(180))

        self.gam = 0.0
        self.lam = 0.0

        start = np.zeros(self.taskShape)
        start[2] = np.radians(45.0)

        self.reconPos, self.reconOr, _, _ = self.chain.fkin(self.recon_joint)
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
        self.pubbin = self.create_publisher(Image, name+'/binary',    3)

        self.cmdmsg = JointState()

        self.timer = self.create_timer(1/rate, self.sendCmd)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))
        
        self.hsvImage = None                   
        self.depthImage = None  

         #Create a temporary handler to grab the info.
    

    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['base', 'shoulder', 'elbow', 'wrist', 'head', 'gripper']

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

        # Extract the depth image information (distance in mm as uint16).
        width  = msg.width
        height = msg.height
        depth  = np.frombuffer(msg.data, np.uint16).reshape(height, width)

        self.depthImage = depth
        # Report.
        col = width//2
        row = height//2
        self.centerDist   = depth[row][col]

    # # Process the image (detect the ball).
    # def rgb_process(self, msg):
    #     # Confirm the encoding and report.
    #     assert(msg.encoding == "rgb8")
    #     # self.get_logger().info(
    #     #     "Image %dx%d, bytes/pixel %d, encoding %s" %
    #     #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

    #     # Convert into OpenCV image, using RGB 8-bit (pass-through).
    #     frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    #     # Update the HSV limits (updating the control window).

    #     # Convert to HSV
    #     hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    #     # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Cheat: swap red/blue

    #     # Threshold in Hmin/max, Smin/max, Vmin/max
    #     binary = cv2.inRange(hsv, self.hsvlimits[:,0], self.hsvlimits[:,1])

    #     self.hsvImage = binary

    #     # Grab the image shape, determine the center pixel.
    #     (H, W, D) = frame.shape
    #     uc = W//2
    #     vc = H//2

    #     # Draw the center lines.  Note the row is the first dimension.
    #     frame = cv2.line(frame, (uc,0), (uc,H-1), (255, 255, 255), 1)
    #     frame = cv2.line(frame, (0,vc), (W-1,vc), (255, 255, 255), 1)

    #     # Report the center HSV values.  Note the row comes first.
    #     # self.get_logger().info(
    #     #     "Center pixel HSV = (%3d, %3d, %3d)" % tuple(hsv[vc, uc]))

    #     # Convert the frame back into a ROS image and republish.
    #     self.pubrgb.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

    #     # Also publish the thresholded binary (black/white) image.
    #     self.pubbin.publish(self.bridge.cv2_to_imgmsg(binary))
    
    def saveFdbck(self,pd):
        self.positionCmds.append(pd)
        self.actualPos.append(self.actualJointPos)
        self.actEff.append(self.actualJointEff)   
    
    def setT(self):
        match self.prSt:
            case 'GOTO_REC':
                self.T = GOTO_REC_T
            case 'REC':
                self.T = REC_T
            case 'GOTO_PRI':
                self.T = GOTO_PRI_T
            case 'GRAB':
                self.T = GRAB_T  
            case 'GOTO_READY':
                self.T = READY_T
    def updateTime(self):
        if self.prSt != self.nxSt:
            sec, nano = self.get_clock().now().seconds_nanoseconds()
            self.t = 0.0
            self.dt = 0.0
            self.start_time = sec + nano*10**(-9)
        elif self.t < self.T:
            sec, nano = self.get_clock().now().seconds_nanoseconds()
            now = sec + nano*10**(-9)
            self.dt = (now - self.start_time) - self.t
            self.t = now - self.start_time
    
    def updateNextState(self):
        if (self.t < self.T):
            self.nxSt = self.prSt
        else:
          match self.prSt:
            case 'GOTO_REC':
                self.nxSt = 'REC'
            case 'REC':
                self.get_logger().info(f"priority {(self.priorityDonut)}")
                if self.priorityDonut is not None:
                  self.nxSt = 'GOTO_READY'
                else:
                  self.nxSt = 'REC'
                # self.get_logger().info(f"nx {(self.nxSt)}")
            case 'GOTO_READY':
                self.nxSt = 'GOTO_PRI'
            case 'GOTO_PRI':
                self.nxSt = 'GOTO_PRI'
            case 'GRAB':
                self.nxSt = 'GOTO_REC'
          self.taskPosition0 = self.pd[:]
          self.initJointPos  = self.q[:]


    def updateState(self):
        self.prSt = self.nxSt
        # self.get_logger().info(f"state {(self.prSt)}")

    def gotoRec(self):
        
        q, qdot = spline(self.t, self.T, np.array(self.initJointPos).reshape(self.jointShape),
                          self.recon_joint, 
                       np.zeros(self.jointShape, dtype=float), 
                    np.zeros(self.jointShape, dtype=float))
        
        if self.t < self.T / 2:
            elbow, elbowdot = self.initJointPos[2], 0.0
        else:
            elbow,elbowdot = spline(self.t - self.T/2, self.T/2, self.initJointPos[2], self.recon_joint[2, 0], 
                    0.0, 
                    0.0)
        q[2, 0] = elbow
        qdot[2, 0] = elbowdot 

        return q, qdot

    def average_index_of_ones(self, arr):
        # Get the indices of 1's
        indices = np.argwhere(arr)

        # Calculate the average index for x and y separately
        avg_index_x = int(np.average(indices[:, 0]))
        avg_index_y = int(np.average(indices[:, 1]))
        return avg_index_x, avg_index_y
    
    
    def ir_depth(self, x, y):
        distance = (self.depthImage[x, y])
        # self.get_logger().info(f"ir distance {distance}")
        return distance

    # must be cyclic
    def recon(self):
        self.priorityDonut = np.array([.18, .50, .06])
        q, qdot = spline(1, 1, self.recon_joint, self.recon_joint, np.zeros(self.jointShape, dtype=float), 
                    np.zeros(self.jointShape, dtype=float))
        


        if len(np.flatnonzero(self.hsvImage)) > 50:
            camera_scale = 1000
            v, u = self.average_index_of_ones(self.hsvImage)
            zc = self.ir_depth(v, u)

            fx = self.camK[0, 0]
            fy = self.camK[1, 1]
            cx = self.camK[0, 2]
            cy = self.camK[1, 2]
            x_bar = (u - cx) / fx
            y_bar = (v - cy) / fy



            # self.get_logger().info(f"k, {self.camK}, shape {np.shape(self.camK)}")
            # bar = cv2.undistortPoints(np.array([u, v]).reshape((1,1,2)), self.camK, self.camD)
            # x_bar, y_bar = bar[0,0,0], bar[0,0,1]
            xc, yc = x_bar * zc / camera_scale, y_bar * zc / camera_scale

            zc = zc / camera_scale

            # theta = self.get_theta()
            # d = self.get_table_distance(self.camK @ np.array([u, v]))
            # x, y, z = self.pd[:, 0]
            # # self.get_logger().info(f"d {d}")
            self.get_logger().info(f"cam x, y, z {[xc, yc, zc]}")
            # (p, R, _, _) = self.chain.fkin(self.q)
            (p, R, _, _) = self.camChain.fkin(np.array(self.actualJointPos[:5]))
            # p[:, 0] = p[:, 0] - np.array([.035, .095, -0.08])
            self.get_logger().info(f"test cam x, y, z {p}")



            self.priorityDonut = np.array(p + R @ 
                                          np.array([xc, zc, -yc]).reshape((3,1))).reshape((3,1))
        
        return q, qdot
    
    def ikin(self, pd, vd):

        wd = np.zeros((3, 1))
        Rd = self.ready_Rd

        qlast  = np.array(self.q).reshape(self.taskShape)
        pdlast = self.pd

        # # Compute the old forward kinematics.
        (p, R, Jv, Jw) = self.chain.fkin(qlast)
        self.get_logger().info(f"p actual{p}")
        # # Set up the inverse kinematics.
        vr    = vd + self.lam * ep(pdlast, p)
        wr    = wd + self.lam * eR(Rd, R)
        J     = np.vstack((Jv, Jw))
        xrdot = np.vstack((vr, wr))
        self.get_logger().info(f"wr {wr.shape}")
        self.get_logger().info(f"vr {vr.shape}")

        
        Jinv = J.T @ np.linalg.pinv(J @ J.T + self.gam**2 * np.eye(6))
        qdot = Jinv @ xrdot
        q = qlast + self.dt * qdot
        # Save the joint value and desired values for next cycle.
        self.q  = q
        self.pd = pd
        # self.Rd = Rd

        return q, qdot
    

    def gotoReady(self):
        q, qdot = spline(self.t, self.T, np.array(self.recon_joint).reshape(self.jointShape),
                          self.readyJointState.reshape(self.jointShape), 
                       np.zeros(self.jointShape, dtype=float), 
                    np.zeros(self.jointShape, dtype=float))        
        return q, qdot
    
    def gotoPri(self):
        task_shape =(3, 1)
        
        pd, vd = spline(self.t, self.T, self.taskPosition0, self.priorityDonut, np.zeros(task_shape, dtype=float), 
                    np.zeros(task_shape, dtype=float))
        self.get_logger().info(f"pd {(self.pd)}")

        return self.ikin(pd, vd)
    
    def grab(self):
       pass

    def executeState(self):
       match self.prSt:
            case 'GOTO_REC':
              return self.gotoRec()
            case 'REC':
              return self.recon()
            case 'GOTO_READY':
              return self.gotoReady()
            case 'GOTO_PRI':
              return self.gotoPri()
            case 'GRAB':
              return self.grab()

    
    def sendCmd(self):
    #     self.get_logger().info(f"T {self.T}")
    #     self.get_logger().info(f"t {self.t}")

        self.setT()
        q, qdot = self.executeState()
        # self.get_logger().info(f"nxt {self.nxSt}")
        self.updateNextState()
        self.updateTime()
        self.updateState()

        q, qdot = list(q[:, 0]), list(qdot[:, 0])


        # self.pd = pd
        self.q = q[:]
   
        motor35eff = 7.1 * np.sin(self.actualJointPos[1] - self.actualJointPos[2])
        motor17offset = -0.07823752 * np.sin(self.actualJointPos[1] - self.actualJointPos[2])
        q[1] = q[1] + motor17offset


        q.append(0.0)
        qdot.append(0.0)

        # self.get_logger().info(f"q: {q}")
        # self.get_logger().info(f"p: {pd}")
        # self.get_logger().info(f"t: {self.t}")
        # self.get_logger().info(f"dt: {self.dt}")
        # self.get_logger().info(f"T: {self.T}")

        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = self.jointnames()
        self.cmdmsg.position     = q #[np.nan, np.nan, np.nan, np.nan, np.nan] 
        self.cmdmsg.velocity     = qdot #[np.nan, np.nan, np.nan, np.nan, np.nan]#
        self.cmdmsg.effort       = [np.nan, np.nan, motor35eff, np.nan, np.nan, np.nan]
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
