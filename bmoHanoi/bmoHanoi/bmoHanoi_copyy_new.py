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
GOTO_REC_T = 5
REC_T = 1
GOTO_PRI_T = 10
GRAB_T = 3
READY_T = 10
HONE_T = 5

TIME_DICT = {'GOTO_REC' : GOTO_REC_T, 
                          'REC' : REC_T, 
                          'GOTO_PRI' : GOTO_PRI_T,
                          'GRAB' : GRAB_T, 
                          'GOTO_READY': READY_T, 
                          'HONE' : HONE_T}


COLOR_HSV_MAP = {'blue': [(85, 118), (175,255), (59, 178)],
                 'green': [(40, 80), (55, 220), (35, 175)],
                  'yellow': [(15, 55), (65, 255), (150, 255)],
                   'orange': [(0, 15), (80, 255), (146, 255)],
                    'red': [(0,5), (160, 255), (70, 230)] 
                    }
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

# from bmoHanoi.CameraProcess import CameraProcess

class CameraProcess():
    def __init__(self, msg):
        self.hsvImageMap = {}
        self.camD = np.array(msg.d).reshape(5)
        self.camK = np.array(msg.k).reshape((3,3))
        self.camw = msg.width
        self.camh = msg.height
        self.depthImage = None

    def average_index_of_ones(self, arr):
        indices = np.argwhere(arr)

        # Calculate the average index for x and y separately
        avg_index_x = int(np.average(indices[:, 0]))
        avg_index_y = int(np.average(indices[:, 1]))
        return avg_index_x, avg_index_y

    
    def ir_depth(self, color):

        dists = self.depthImage[self.hsvImageMap[color] != 0.0]

        return np.nanmean(dists)
    
    def getPriorityDonut(self, color):
        camera_scale = 1000
        v, u = self.average_index_of_ones(self.hsvImageMap[color])
        zc = self.ir_depth(color)
        fx = self.camK[0, 0]
        fy = self.camK[1, 1]
        cx = self.camK[0, 2]
        cy = self.camK[1, 2]
        x_bar = (u - cx) / fx
        y_bar = (v - cy) / fy

        xc, yc = x_bar * zc / camera_scale, y_bar * zc / camera_scale

        zc = zc / camera_scale
        return xc, yc, zc
    
    def get_xy_bar(self, color):
        v, u = self.average_index_of_ones(self.hsvImageMap[color])
        fx = self.camK[0, 0]
        fy = self.camK[1, 1]
        cx = self.camK[0, 2]
        cy = self.camK[1, 2]
        x_bar = (v - cx) / fx
        y_bar = (u - cy) / fy
        return x_bar, y_bar

class Spline():
    def __init__(self, t, T, p0, pd, v0, vd) -> None:
        self.t = t
        self.T = T
        self.p0 = p0
        self.pd = pd
        self.v0 = v0
        self.vd = vd
    
    def excecute_spline():
        pass
        # return q, qdot

class StateMachine():
    def __init__(self, sec, nano):
        self.prSt = 'GOTO_REC'
        self.nxSt = 'GOTO_REC'
        self.priorityDonut = None
        self.T = GOTO_REC_T
        self.t = 0.0
        self.dt = 0.0
        self.start_time = sec + nano*10**(-9)
        self.initJointPos = None
        
    def setT(self):
        self.T = TIME_DICT[self.prSt]
    
    def get_state(self):
        return self.prSt
    
    def updateTime(self, sec, nano):
        if self.t < self.T:
            now = sec + nano*10**(-9)
            self.dt = (now - self.start_time) - self.t
            self.t = now - self.start_time

    def updateState(self):
        self.prSt = self.nxSt

    def updateNextState(self, sec, nano, priorityDonut):

        if (self.t < self.T):
            self.nxSt = self.prSt
            return False
        else:
          match self.prSt:
            case 'GOTO_REC':
                self.nxSt = 'HONE'
            case 'HONE': 
                self.nxSt = 'REC'
            case 'REC':
                # self.get_logger().info(f"priority {(self.priorityDonut)}")
                if priorityDonut is not None:
                  self.nxSt = 'GOTO_PRI'
                else:
                  self.nxSt = 'HONE'
            case 'GOTO_READY':
                self.nxSt = 'GOTO_PRI'
            case 'GOTO_PRI':
                self.nxSt = 'GOTO_PRI'
            case 'GRAB':
                self.nxSt = 'GOTO_REC'
          self.t = 0.0
          self.dt = 0.0
          self.start_time = sec + nano*10**(-9)
          return True
        
    def get_curr_state(self):
        return self.prSt 

class BmoHanoi(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)
        self.camera = None
        def cb(msg):
            self.camera = CameraProcess(msg)
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

        # self.T = 0
        # Create a timer to keep calculating/sending commands.
        rate       = RATE
        # self.t = 0.0
        # self.dt = 0.0
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        self.start_time = sec + nano*10**(-9)

        # self.prSt = 'GOTO_REC'
        # self.nxSt = 'GRAB'
        # self.priorityDonut = None

        self.actualJointPos   = self.grabfbk()
        self.actualJointVel   = None
        self.actualJointEff   = None

        self.initJointPos     = self.actualJointPos[:5]
        self.jointShape = (5, 1)

        self.q = np.array(self.actualJointPos[:5]).reshape(self.jointShape)

        self.recon_joint = np.radians(np.array([0.0, 0.0, -90, 0.0, 0.0])).reshape(self.jointShape)
        self.taskPosition0, self.taskOrientation0, _, _ = self.chain.fkin(self.q)

        self.pd = self.taskPosition0[:]
        self.vd = np.zeros((3, 1))
        self.Rd = self.taskOrientation0


        self.readyJointState = np.radians(np.array([0.0, -45.0, -135.0, -90, 0.0]))
        self.ready_Rd = Rotz(np.radians(180))

        self.gam = 0.3
        self.lam = .1

        start = np.zeros(self.jointShape)
        start[2] = np.radians(45.0)

        self.reconPos, self.reconOr, _, _ = self.chain.fkin(self.recon_joint)
        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()
        # Create Subscribers for image
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
        self.rec_orange = self.create_subscription(Image, name+'/binary', self.orangeImage,    3)

        self.cmdmsg = JointState()

        self.timer = self.create_timer(1/rate, self.sendCmd)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))
        
        self.depthImage = None              

        self.state_machine = StateMachine(sec, nano)
        self.state_dict = {'GOTO_REC': self.gotoRec, 
                    'REC' : self.recon, 
                    'GOTO_READY': self.gotoReady,
                    'GOTO_PRI': self.gotoPri,
                    'GRAB': self.grab,
                    'HONE': self.hone}
        self.priorityDonut = None
        self.grabpos, self.grabvel = 0.0, 0.0

    def orangeImage(self, msg):
        self.camera.hsvImageMap['orange'] = self.bridge.imgmsg_to_cv2(msg, "passthrough")


    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['base', 'shoulder', 'elbow', 'wrist', 'head', 'gripper']

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut dowfn the node.
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
        self.camera.depthImage = depth
        # Report.
    
    # def setT(self):
    #     self.T = TIME_DICT[self.prSt]

    # def updateState(self):
    #     self.prSt = self.nxSt
        
    # TODO: update all of our states to use generic spline class

    def gotoRec(self):
        q, qdot = spline(self.state_machine.t, self.state_machine.T, np.array(self.initJointPos).reshape(self.jointShape),
                          self.recon_joint, 
                       np.zeros(self.jointShape, dtype=float), 
                    np.zeros(self.jointShape, dtype=float))
        
        if self.state_machine.t < self.state_machine.T / 2:
            elbow, elbowdot = self.initJointPos[2], 0.0
        else:
            elbow,elbowdot = spline(self.state_machine.t - self.state_machine.T/2, self.state_machine.T/2, 
                    self.initJointPos[2], self.recon_joint[2, 0], 
                    0.0, 
                    0.0)
        q[2, 0] = elbow
        qdot[2, 0] = elbowdot 

        return q, qdot

    def recon(self):
        self.priorityDonut = None
        q, qdot = spline(1, 1, np.array(self.initJointPos).reshape(self.jointShape),
                                           np.array(self.initJointPos).reshape(self.jointShape),
                                             np.zeros(self.jointShape, dtype=float), 
                    np.zeros(self.jointShape, dtype=float))
        
        if len(np.flatnonzero(self.camera.hsvImageMap['orange'])) > 50:
            xc, yc, zc = self.camera.getPriorityDonut('orange')
            # TODO: should this be self.q instead?
            (p, R, _, _) = self.camChain.fkin(np.array(self.actualJointPos[:5]))
            self.priorityDonut = np.array(p + R @ 
                        np.array([xc, zc, -yc]).reshape((3,1))).reshape((3,1))
        
        return q, qdot
    
    def ikin_test(self, pd, vd):
        wd = np.zeros((3, 1))
        Rd = self.taskOrientation0

        qlast  = np.array(self.q).reshape(self.jointShape)
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
        q = qlast + self.state_machine.dt * qdot
        # Save the joint value and desired values for next cycle.
        self.q  = q
        self.pd = pd
        # self.Rd = Rd

        return q, qdot
    
    def ikin(self, pd, vd, wd, Rd, stage=None):

        qlast  = np.array(self.q).reshape(self.jointShape)
        pdlast = self.pd

        # # Compute the old forward kinematics.
        (p, R, Jv, Jw) = self.chain.fkin(qlast)
        self.get_logger().info(f"p actual : {p}")

        vr    = vd + 0.0 * ep(pdlast, p)
        wr    = wd + 0.0 * eR(Rd, R)

        if stage == 'hone':
            wr =  (np.eye(3) - (R[0:3,1:2] @ R[0:3,1:2].T)) @ \
         (wd + self.lam * .5 * cross(R[0:3,1:2], Rd[0:3,1:2]))  # eR(Rd, R)
            Jw = (np.eye(3) - (R[0:3,1:2] @ R[0:3,1:2].T)) @ Jw
        
        J     = np.vstack((Jv, Jw))
        xrdot = np.vstack((vr, wr))

        
        Jinv = J.T @ np.linalg.pinv(J @ J.T + self.gam**2 * np.eye(6))
        qdot = Jinv @ xrdot
        q = qlast + self.state_machine.dt * qdot
        
        # Save the joint value and desired values for next cycle.
        self.q  = q
        self.pd = pd
        self.Rd = Rd

        return q, qdot
    
    def gotoReady(self):
        q, qdot = spline(self.state_machine.t, self.state_machine.T, np.array(self.recon_joint).reshape(self.jointShape),
                          self.readyJointState.reshape(self.jointShape), 
                       np.zeros(self.jointShape, dtype=float), 
                    np.zeros(self.jointShape, dtype=float))        
        return q, qdot
    
    def centerColor(self, color):
        # Gives w for realsense
        if len(np.flatnonzero(self.camera.hsvImageMap[color])) > 50:

            self.camera : CameraProcess
            x_bar, y_bar = self.camera.get_xy_bar(color)


            kp = .75
            (p, R, jv, jr) = self.camChain.fkin(self.q)

            return R @ (np.array([-1 * x_bar , 0, -y_bar]).reshape((3, 1)) * kp)
        return np.zeros((3, 1))

    def hone(self):
        (p, R, jv, jr) = self.chain.fkin(self.q)
        self.pd = self.taskPosition0[:]
        pd = self.pd

        vd = np.zeros((3, 1))
        wd = self.centerColor('orange')

        theta_x = np.arctan2(R[2, 1], R[2, 2])
        theta_y = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        theta_z = np.arctan2(R[1, 0], R[0, 0])

        Rd = Rotz(theta_z + wd[2, 0]*1/RATE) @ Roty(theta_y + wd[1, 0]*1/RATE)\
        @ Rotx(theta_x + wd[0, 0]*1/RATE) 

        q, qdot = self.ikin(pd, vd, wd, Rd, stage='hone')
        self.get_logger().info(f"wd: {[wd]}")

        

        return q, qdot
    
    def gotoPri(self):
        task_shape =(3, 1)
        
        pd, vd = spline(self.state_machine.t, self.state_machine.T, self.taskPosition0, self.priorityDonut, np.zeros(task_shape, dtype=float), 
                    np.zeros(task_shape, dtype=float))
        self.get_logger().info(f"pd {(self.pd)}")
        self.get_logger().info(f"donut: {self.priorityDonut}")

        wd = np.zeros((3, 1))
        Rd = self.taskOrientation0 #self.ready_Rd
        return self.ikin_test(pd, vd)
    
    def grab(self):
        task_shape =(3, 1)
        pd, vd = spline(1.0, 1.0, self.priorityDonut, self.priorityDonut, np.zeros(task_shape, dtype=float), 
            np.zeros(task_shape, dtype=float))
        self.grabpos, self.grabvel = spline(self.state_machine.t, self.state_machine.T, 0.0, -.5, 0.0, 0.0)
        self.get_logger().info(f"grab pos: {self.grabpos}")

        wd = np.zeros((3, 1))
        Rd = self.ready_Rd
        return self.ikin(pd, vd, wd, Rd)
    
    def executeState(self):
        return self.state_dict[self.state_machine.get_curr_state()]()
    
    def sendCmd(self):
        # self.state_machine.updateTime()
        self.state_machine : StateMachine
        self.state_machine.setT()

        q, qdot = self.executeState()

        sec, nano = self.get_clock().now().seconds_nanoseconds()

        if self.state_machine.updateNextState(sec, nano, self.priorityDonut):
            self.taskPosition0, self.taskOrientation0 , _, _ = self.chain.fkin(self.q) 
            self.initJointPos  = self.q
        self.state_machine.updateTime(sec, nano)
        self.get_logger().info(f"t, T: {self.state_machine.t, self.state_machine.T}")
        self.state_machine.updateState()
        self.get_logger().info(f"state: {self.state_machine.prSt}")

        
        # self.setT()
        # q, qdot = self.executeState()
        # self.updateNextState()
        # self.updateTime()
        # self.updateState()

        q, qdot = list(q[:, 0]), list(qdot[:, 0])

        self.q = q[:]
   
        motor35eff = 6.81 * np.sin(self.actualJointPos[1] - self.actualJointPos[2])
        motor17offset = -0.07823752 * np.sin(self.actualJointPos[1] - self.actualJointPos[2])
        q[1] = q[1] + motor17offset

        if self.state_machine.prSt == "GRAB":
            q.append(self.grabpos)
            qdot.append(self.grabvel)
            # self.get_logger().info(f"q: {q}")
        else:
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
