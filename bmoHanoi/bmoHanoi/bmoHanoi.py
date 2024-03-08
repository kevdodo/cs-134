

RATE = 100

COLOR_HSV_MAP = {'blue': [(85, 118), (175,255), (59, 178)],
                 'green': [(40, 80), (55, 220), (35, 175)],
                  'yellow': [(15, 55), (65, 255), (150, 255)],
                   'orange': [(0, 15), (80, 255), (146, 255)],
                    'red': [(0,5), (160, 255), (70, 230)] 
                    }

SPLINE_EFFORT_T = 7.0

import cv2
import numpy as np
import copy
# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image, JointState, CameraInfo

from bmoHanoi.process_color_depth import rgb_process, depth_process
from bmoHanoi.TrajectoryUtils import *
from bmoHanoi.TransformHelpers import *
from bmoHanoi.KinematicChain import *

from bmoHanoi.StateMachine import StateMachine
from bmoHanoi.CameraProcess import CameraProcess







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

        # Create a timer to keep calculating/sending commands.
        rate       = RATE
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


        self.recon_joint = np.radians(np.array([0.0, -15, -60, 30, 0.0])).reshape(self.taskShape)
        self.taskPosition0, self.taskOrientation0, _, _ = self.chain.fkin(self.q)

        self.grabpos = self.actualJointPos[-1]
        self.grabpos0 = self.grabpos

        self.pd = self.taskPosition0[:]
        self.vd = np.zeros((3, 1))
        self.Rd = self.taskOrientation0
        self.wd = np.zeros((2, 1))

        self.readyJointState = np.radians(np.array([0.0, 0.0, -90, -0.0, 0.0])) # np.radians(np.array([0.0, -45.0, -135.0, -90, 0.0]))
        self.ready_Rd = Rotz(np.radians(180))

        self.gam = 0.00
        self.lam = 10.0

        start = np.zeros(self.taskShape)
        start[2] = np.radians(45.0)

        self.reconPos, self.reconOr, _, _ = self.chain.fkin(self.recon_joint)
        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()
        self.pubrgb = self.create_publisher(Image, '/camera/color/display_image', 3)
        # Create Subscribers for image
        # self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', 
        #                                         self.rgb_process, 1)
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
        # self.rec_orange = self.create_subscription(Image, name + '/orangebinary', self.orangeImage,    3)
        self.rec_orange = self.create_subscription(Image, name+'/orangebinary', lambda msg: self.procImage(msg, 'orange'), 3)
        self.rec_blue = self.create_subscription(Image, name+'/bluebinary', lambda msg: self.procImage(msg, 'blue'), 3)
        self.rec_green = self.create_subscription(Image, name+'/greenbinary', lambda msg: self.procImage(msg, 'green'), 3)
        self.rec_red = self.create_subscription(Image, name+'/redbinary', lambda msg: self.procImage(msg, 'red'), 3)
        self.rec_yellow = self.create_subscription(Image, name+'/yellowbinary', lambda msg: self.procImage(msg, 'yellow'), 3)
        self.rec_yellow = self.create_subscription(Image, name+'/blackbinary', lambda msg: self.procImage(msg, 'black'), 3)

        self.cmdmsg = JointState()

        self.state_machine = StateMachine(sec, nano)

        self.timer = self.create_timer(1/rate, self.sendCmd)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))
        
        self.hsvImage = None                   
        self.depthImage = None  
        
        self.spline_effort = True

        self.state_dict = {'GOTO_REC': self.gotoRec, 
                    'REC' : self.recon, 
                    'GOTO_PRI': self.gotoPriPick,
                    'GOTO_PRI_HALF': self.gotoPriPickHalf,
                    'HOLD': self.hold_close,
                    'HONE': self.hone,
                    'HONE_HALF': self.honeHalf,
                    'GOTO_READY': self.gotoReady,
                    'GO_UP' :self.goUp,
                    'REC_HALF': self.recon,
                    'MOVE_DOWN': self.move_down,
                    'SPECIAL_REC': self.special_recon}
        self.grabpos, self.grabvel = 0.0, 0.0


         #Create a temporary handler to grab the info.
    def procImage(self, msg, color):
        self.camera.hsvImageMap[color] = self.bridge.imgmsg_to_cv2(msg, "passthrough")

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
        col = width//2
        row = height//2
        self.centerDist   = depth[row][col]
        self.get_logger().info(f"bruh {self.centerDist}")
    
    def gotoRec(self):
        

        q, qdot = spline(self.state_machine.t, self.state_machine.T, np.array(self.initJointPos).reshape(self.jointShape),
                          self.recon_joint, 
                       np.zeros(self.jointShape, dtype=float), 
                    np.zeros(self.jointShape, dtype=float))
        
        if self.state_machine.t < self.state_machine.T / 2:
            elbow, elbowdot = self.initJointPos[2], 0.0
        else:
            elbow,elbowdot = spline(self.state_machine.t - self.state_machine.T/2, self.state_machine.T/2, self.initJointPos[2], self.recon_joint[2, 0], 
                    0.0, 
                    0.0)
        q[2, 0] = elbow
        qdot[2, 0] = elbowdot 
        self.grab()

        return q, qdot
    
    def special_recon(self):
        self.priorityDonut = None
        q, qdot = spline(1, 1, np.array(self.initJointPos).reshape(self.jointShape),
                                           np.array(self.initJointPos).reshape(self.jointShape),
                                             np.zeros(self.jointShape, dtype=float), 
                    np.zeros(self.jointShape, dtype=float))
        ir = None
        if len(np.flatnonzero(self.camera.hsvImageMap['orange'])) > 50:
            xc, yc, zc = self.camera.getPriorityDonut('orange')
            # TODO: should this be self.q instead?
            (p, R, _, _) = self.camChain.fkin(np.array(self.actualJointPos[:5]))
            self.priorityDonut = np.array(p + R @ 
                        np.array([xc, zc, -yc]).reshape((3,1))).reshape((3,1))
            
            ir = self.camera.ir_depth('orange')
        self.get_logger().info(f"qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq {self.q}")
        self.get_logger().info(f"ir {ir}")

        return q, qdot


    def recon(self):
        self.priorityDonut = None
        q, qdot = spline(1, 1, np.array(self.initJointPos).reshape(self.jointShape),
                                           np.array(self.initJointPos).reshape(self.jointShape),
                                             np.zeros(self.jointShape, dtype=float), 
                    np.zeros(self.jointShape, dtype=float))
        ir = None
        if len(np.flatnonzero(self.camera.hsvImageMap['orange'])) > 50:
            xc, yc, zc = self.camera.getPriorityDonut('orange')
            # TODO: should this be self.q instead?
            (p, R, _, _) = self.camChain.fkin(np.array(self.actualJointPos[:5]))
            self.priorityDonut = np.array(p + R @ 
                        np.array([xc, zc, -yc]).reshape((3,1))).reshape((3,1))
            
            ir = self.camera.ir_depth('orange')
        self.get_logger().info(f"priority donut {self.priorityDonut}")
        self.get_logger().info(f"ir {ir}")

        return q, qdot

    
    def centerColor(self, color, kp):
        # Gives w for realsense
        if len(np.flatnonzero(self.camera.hsvImageMap[color])) > 50:

            self.camera : CameraProcess
            x_bar, y_bar = self.camera.get_xy_bar(color)

            # kp = 1.25
            (p, R, jv, jr) = self.camChain.fkin(np.array(self.actualJointPos[:5]).reshape(5,1))

            self.get_logger().info(f"v stuffffffffffffffffff {np.array([-1 * x_bar , 0, -y_bar]).reshape((3, 1)) * kp}")

            return R @ (np.array([-1 * x_bar , 0, -y_bar]).reshape((3, 1)) * kp)
        
        return np.zeros((3, 1))
    
    def honeHalf(self):
        (p, R, jv, jr) = self.chain.fkin(self.q)
        # (p, R, jv, jr) = self.chain.fkin(np.array(self.actualJointPos[:5]).reshape(5, 1))
        # self.get_logger().info(f"q {self.q}")
        # self.get_logger().info(f"actual q{self.actualJointPos}")
        # self.pd = self.taskPosition0[:]

        task_shape = (3,1)

        pd, vd = spline(self.state_machine.t, self.state_machine.T, self.taskPosition0, (self.taskPosition0 + self.priorityDonut) / 2, np.zeros(task_shape, dtype=float), 
                    np.zeros(task_shape, dtype=float))

        kp, _ = spline5(self.state_machine.t, self.state_machine.T, 0.0, 1.3, 0.0, 0.0, 0.0, 0.0)

        wd = self.centerColor('orange', kp)

        theta_x = np.arctan2(R[2, 1], R[2, 2])
        theta_y = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        theta_z = np.arctan2(R[1, 0], R[0, 0])

        Rd = Rotz(theta_z + wd[2, 0]*1/RATE) @ Roty(theta_y + wd[1, 0]*1/RATE)\
        @ Rotx(theta_x + wd[0, 0]*1/RATE) 

        # self.get_logger().info(f"wd, R shape{[wd.shape, R[0:3, 1:2].shape]}")


        # wd = exyz(wd[0, 0], wd[1, 0], wd[2, 0])

        # y_desired = np.cross(wd[:, 0], R[0:3, 2:3][:, 0])  #np.cross(wd[:, 0], R[0:3, 1:2][:, 0])
        
        # a = angle(wd, R[0:3, 2:3][:, 0])


        # self.get_logger().info(f"angle {a}")

        # Rd = Rote(wd, a_d) @ R
        

        q, qdot = self.ikin(pd, vd, wd, Rd, stage='hone')

        return q, qdot
    
    def hone(self):
        (p, R, jv, jr) = self.chain.fkin(self.q)
        # (p, R, jv, jr) = self.chain.fkin(np.array(self.actualJointPos[:5]).reshape(5, 1))
        self.get_logger().info(f"q {self.q}")
        self.get_logger().info(f"actual q{self.actualJointPos}")
        # self.pd = self.taskPosition0[:]

        task_shape = (3,1)
        
        # pd, vd = spline(self.state_machine.t, 2*self.state_machine.T, self.taskPosition0, self.priorityDonut, np.zeros(task_shape, dtype=float), 
        #             np.zeros(task_shape, dtype=float))

        pd = self.taskPosition0
        vd = np.zeros((3, 1))


        kp, _ = spline5(self.state_machine.t, self.state_machine.T, 0.0, .75, 0.0, 0.0, 0.0, 0.0)
        wd = self.centerColor('orange', kp)

        theta_x = np.arctan2(R[2, 1], R[2, 2])
        theta_y = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        theta_z = np.arctan2(R[1, 0], R[0, 0])

        Rd = Rotz(theta_z + wd[2, 0]*1/RATE) @ Roty(theta_y + wd[1, 0]*1/RATE)\
        @ Rotx(theta_x + wd[0, 0]*1/RATE) 

        q, qdot = self.ikin(pd, vd, wd, Rd, stage='hone')
        
        if len(np.flatnonzero(self.camera.hsvImageMap['orange'])) > 50:
            xc, yc, zc = self.camera.getPriorityDonut('orange')
            # TODO: should this be self.q instead?
            (p, R, _, _) = self.camChain.fkin(np.array(self.actualJointPos[:5]))
            self.priorityDonut = np.array(p + R @ 
                        np.array([xc, zc, -yc]).reshape((3,1))).reshape((3,1))
        return q, qdot
    
    def ikin(self, pd, vd, wd, Rd, stage=None):

        qlast  = np.array(self.q).reshape(self.jointShape)
        pdlast = self.pd
        Rdlast = self.Rd

        # # Compute the old forward kinematics.
        (p, R, Jv, Jw) = self.chain.fkin(qlast)


        vr    = vd + self.lam * ep(pdlast, p)
        # wr    = wd + self.lam * eR(Rd, R)

        # if stage == 'hone':
        wr =  (np.eye(3) - (R[0:3,1:2] @ R[0:3,1:2].T)) @ \
        (wd + self.lam * .5 * cross(R[0:3,1:2], Rdlast[0:3,1:2]))  # eR(Rd, R)
        Jw = (np.eye(3) - (R[0:3,1:2] @ R[0:3,1:2].T)) @ Jw
        
        J     = np.vstack((Jv, Jw))     
        xrdot = np.vstack((vr, wr))


        
        Jinv = J.T @ np.linalg.pinv(J @ J.T + self.gam**2 * np.eye(6))
        qsdot = np.array(self.recon_joint).reshape((5,1))
        qsdot[3, 0] += np.radians(-60)

        qdot = Jinv @ xrdot + (np.eye(5) - Jinv @ J) @ (self.lam * qsdot)
        # qdot = Jinv @ xrdot
        q = qlast + self.state_machine.dt * qdot
        
        # Save the joint value and desired values for next cycle.
        self.q  = q
        self.pd = pd
        self.Rd = Rd

        return q, qdot
    
    def goUp(self):
        task_shape =(3, 1)
        new_dest = copy.deepcopy(self.priorityDonut[:])
        new_dest[2, 0] = self.priorityDonut[2,0] + .25

        pd, vd = spline(self.state_machine.t, self.state_machine.T, self.priorityDonut, new_dest, np.zeros(task_shape, dtype=float), 
            np.zeros(task_shape, dtype=float))
        # self.grabpos, self.grabvel = spline(self.state_machine.t, self.state_machine.T, 0.0, -.5, 0.0, 0.0)
        wd = np.zeros((3, 1))
        Rd = self.taskOrientation0
        return self.ikin(pd, vd, wd, Rd)
    
    def gotoReady(self):
        q, qdot = spline(self.state_machine.t, self.state_machine.T, np.array(self.initJointPos).reshape(self.jointShape),
                          self.readyJointState.reshape(self.jointShape), 
                       np.zeros(self.jointShape, dtype=float), 
                    np.zeros(self.jointShape, dtype=float))        
        return q, qdot
    
    def gotoPriPick(self):
        task_shape =(3, 1)
        
        pd, vd = spline(self.state_machine.t, self.state_machine.T, self.taskPosition0, self.priorityDonut, np.zeros(task_shape, dtype=float), 
                    np.zeros(task_shape, dtype=float))
        self.get_logger().info(f"pd {pd}")

        R = self.taskOrientation0

        theta_x = np.arctan2(R[2, 1], R[2, 2])
        theta_y = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        theta_z = np.arctan2(R[1, 0], R[0, 0])

        endPick = np.radians([0.0, 0.0, -180.0])

        

        # if abs(theta_x - 2 * np.pi) > abs(theta_x):
        #     endPick[0] = 2 * np.pi
        # if abs(theta_y - 2 * np.pi) > abs(theta_y):
        #     endPick[1] = 2 * np.pi

        if abs(theta_z - np.pi) < abs(theta_z):
            endPick[2] = np.pi

        a, a_dot = spline(self.state_machine.t, self.state_machine.T, 
                                  theta_x, endPick[0], 0.0, 0.0)
        
        b, b_dot = spline(self.state_machine.t, self.state_machine.T, 
                                  theta_y, endPick[1], 0.0, 0.0)
        
        c, c_dot = spline(self.state_machine.t, self.state_machine.T, 
                                  theta_z, endPick[2], 0.0, 0.0)


        wdx = np.array([[0, -c_dot, b_dot], [c_dot, 0, -a_dot], [-b_dot, a_dot, 0]])
        
        Rd = Rotz(c) @ Roty(b) @ Rotx(a) 

        Rdot = wdx @ Rd
        #        wd = np.zeros((3,1))#ez()*a_dot + Rotz(a) @ (ey()*b_dot + Roty(b) @ ex()*c_dot) 

        # wd = np.zeros((3,1))#ez()*a_dot + Rotz(a) @ (ey()*b_dot + Roty(b) @ ex()*c_dot) 

        wd = R @ np.array([Rd[:, 2].T @ Rdot[:, 1], Rd[:, 0].T @ Rdot[:, 2], Rd[:, 1].T @ Rdot[:, 0]]).reshape((3, 1))

        self.release()
        return self.ikin(pd, vd, wd, Rd, stage="reach")
    
    def gotoPriPickHalf(self):
        task_shape =(3, 1)
        
        pd, vd = spline(self.state_machine.t, 5*self.state_machine.T, self.taskPosition0, self.priorityDonut, np.zeros(task_shape, dtype=float), 
                    np.zeros(task_shape, dtype=float))

        Rd = self.taskOrientation0

        wd = np.zeros(task_shape)

        return self.ikin(pd, vd, wd, Rd, stage="reach")

    def move_down(self):
        task_shape =(3, 1)
        place = copy.deepcopy(self.priorityDonut)
        place[0:2, 0] = self.taskPosition0[0:2, 0]
        #self.taskPosition0 * 1/1.45 + (1-1/1.45)* place    * 1/5 + (1-1/5)* place
        pd, vd = spline(self.state_machine.t, self.state_machine.T, self.taskPosition0, self.taskPosition0 * 1/5 + (1-1/5)* place, np.zeros(task_shape, dtype=float), 
                    np.zeros(task_shape, dtype=float))

        Rd = self.taskOrientation0

        wd = np.zeros(task_shape)

        return self.ikin(pd, vd, wd, Rd, stage="reach")

    
    def hold_close(self):
        task_shape =(3, 1)
        pd, vd = spline(self.state_machine.T, self.state_machine.T, self.taskPosition0, self.taskPosition0, np.zeros(task_shape, dtype=float), 
            np.zeros(task_shape, dtype=float))
        wd = np.zeros((3, 1))
        Rd = self.taskOrientation0
        self.grab()
        return self.ikin(pd, vd, wd, Rd)
    
        
    def hold_open(self):
        task_shape =(3, 1)
        pd, vd = spline(self.state_machine.T, self.state_machine.T, self.taskPosition0, self.taskPosition0, np.zeros(task_shape, dtype=float), 
            np.zeros(task_shape, dtype=float))
        wd = np.zeros((3, 1))
        Rd = self.taskOrientation0
        self.release()
        return self.ikin(pd, vd, wd, Rd)
    
    def grab(self):
        self.grabpos, self.grabvel = spline(self.state_machine.t, self.state_machine.T, self.grabpos0, -.6, 0.0, 0.0)

    def release(self):
        self.grabpos, self.grabvel = spline(self.state_machine.t, self.state_machine.T, self.grabpos0, 0.3, 0.0, 0.0)

    def executeState(self):
        return self.state_dict[self.state_machine.get_curr_state()]()

    
    def sendCmd(self):
        
        self.state_machine : StateMachine
        self.state_machine.setT()

        q, qdot = self.executeState()

        sec, nano = self.get_clock().now().seconds_nanoseconds()

        if self.state_machine.updateNextState(sec, nano, self.priorityDonut):
            # self.get_logger().info("donut")
            self.spline_effort = False

            self.taskPosition0, self.taskOrientation0 , _, _ = self.chain.fkin(self.q) 
            self.grabpos0 = self.grabpos
            self.initJointPos  = self.q
        self.state_machine.updateTime(sec, nano)
        self.state_machine.updateState()


        q, qdot = list(q[:, 0]), list(qdot[:, 0])



        self.q = q[:]
        
        # self.pd = pd
        if self.spline_effort:    
            p, _ = spline(self.state_machine.t, SPLINE_EFFORT_T, 0.0, 1.0, 0.0, 0.0)
            motor35eff = 7.81 *p * np.sin(self.actualJointPos[1] - self.actualJointPos[2])
            motor17offset = -0.07823752 * p * np.sin(self.actualJointPos[1] - self.actualJointPos[2])
        else:
            motor35eff = 7.81 * np.sin(self.actualJointPos[1] - self.actualJointPos[2])
            motor17offset = -0.07823752 * np.sin(self.actualJointPos[1] - self.actualJointPos[2])
        q[1] = q[1] + motor17offset

        # ir = None
        # if not self.spline_effort and len(np.flatnonzero(self.camera.hsvImageMap['orange'])) > 50:
        #     xc, yc, zc = self.camera.getPriorityDonut('orange')
        #     ir = self.camera.ir_depth('orange')
        #     # TODO: should this be self.q instead?
        #     (p, R, _, _) = self.camChain.fkin(np.array(self.actualJointPos[:5]))
        #     self.priorityDonut = np.array(p + R @ 
        #                 np.array([xc, zc, -yc]).reshape((3,1))).reshape((3,1))
            
        # self.get_logger().info(f"ir depth: {ir}")
        # if len(np.flatnonzero(self.camera.hsvImageMap['orange'])) > 50:
        #     xc, yc, zc = self.camera.getPriorityDonut('orange')
        #     # TODO: should this be self.q instead?
        #     (p, R, _, _) = self.camChain.fkin(np.array(self.actualJointPos[:5]))
        #     self.priorityDonut = np.array(p + R @ 
        #                 np.array([xc, zc, -yc]).reshape((3,1))).reshape((3,1))
            
        #         # self.get_logger().info(f"q: {q}")

        # if self.state_machine.grab:
        q.append(self.grabpos)
        qdot.append(self.grabvel)


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