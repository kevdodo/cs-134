#!/usr/bin/env python3
#
#   toughAndGo.py
#
#   Goal 2 touch and go task code.
#
import numpy as np
import rclpy

from rclpy.node         import Node
from sensor_msgs.msg    import JointState

from basic134.KinematicChain import KinematicChain
from geometry_msgs.msg          import Point, Quaternion, Pose


import time
from demo134.TrajectoryUtils import *
from demo134.TransformHelpers import *

import matplotlib.pyplot as plt

#
#   Definitions
#
RATE = 100.0            # Hertz

GO_TO_START_T = 3.0    # Seconds
GO_TO_POINT_T = 3.0      # Seconds
STRIP_GO_T = 9.0
GO_TO_T = 10.0    # Seconds
HOLD_TIME = 1.0
#
#   DEMO Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a temporary subscriber to grab the initial position.
        self.jointPos0 = self.grabfbk()
        self.get_logger().info("Initial positions: %r" % self.jointPos0)
        self.actualJointPos = self.jointPos0
        self.q = self.actualJointPos
        self.actualJointVel = np.zeros((3, 1))
        self.actualJointEff = np.zeros((3, 1))
        self.qdot = np.zeros((3, 1))
        self.effort = np.zeros((3, 1))

        self.joint_to_return_from = np.copy(self.jointPos0)
        self.vel_to_return_from = np.copy(self.actualJointVel)
        self.start_vel_to_point = np.zeros((3, 1))
        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
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

        self.timer = self.create_timer(1/rate, self.sendcmd)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))

        self.hold = False
        self.detected = True
        self.go = False
        self.hold_point = False
        self.strip_detected = False

        self.did_circle = False

        # This is to get point coordinates
        self.pointx = None
        self.pointy = None
        self.pointz = None
        self.gox = None
        self.goy = None
        self.goz = None
        self.posex = None
        self.posey = None
        self.posez = None
        self.posew = None




        self.pointsub = self.create_subscription(Point, '/point', self.recvpoint, 10)
        self.posesub = self.create_subscription(Pose, '/pose', self.recpose, 10)


        self.chain = KinematicChain('world', 'tip', self.jointnames())
        self.idleJointPos = np.radians([0,70,180]).reshape((3,1))
        self.idlePosition, _, _, _ = self.chain.fkin(self.idleJointPos)
        self.coords_from, _, _, _ = self.chain.fkin(self.actualJointPos)

        self.t = 0.0

        sec, nano = self.get_clock().now().seconds_nanoseconds()
        self.start_time = sec + nano*10**(-9)

        self.gam = .1
        self.lam = 10
       
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['base', 'shoulder', 'elbow']
    
    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    # Grab a single feedback - do not call this repeatedly.
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
    
    def recvpoint(self, pointmsg):
        # Extract the data.
        # Only recieve messages when in hold
        if self.hold and not self.go:
            self.pointx = pointmsg.x
            self.pointy = pointmsg.y
            self.pointz = .015

    def recpose(self, posemsg):
        # Extract the data.
        if self.hold and not self.go:
            point = posemsg.position
            quat = posemsg.orientation

            self.posex = point.x
            self.posey = point.y
            self.posez = point.z
            self.posew = np.arcsin(quat.z) * 2
    
    def get_strip_points(self, left):
        if left:
            return self.posex - .025* np.sin(self.posew), self.posey + .025*np.cos(self.posew) , 0.0
        return self.posex + .025* np.sin(self.posew), self.posey - .025*np.cos(self.posew) , 0.0
        
    # Receive feedback - called repeatedly by incoming messages.
    def recvfbk(self, fbkmsg):
        self.actualJointPos   = list(fbkmsg.position)
        self.actualJointVel   = list(fbkmsg.velocity)
        self.actualJointEff        = list(fbkmsg.effort)
        self.grabready = True
    
    def start_to_idle(self, t, T):
        q, qDot = spline(t, T, np.array(self.joint_to_return_from).reshape((3,1)), 
                        self.idleJointPos.reshape((3,1)), 
                        np.array(self.vel_to_return_from).reshape((3,1)), 
                        np.zeros((3, 1))) 
        return q, qDot

    def go_to_coords(self, t, T, x, y, z):
        p, v = spline(t, T, np.array(self.coords_from).reshape((3,1)), 
                        np.array([x, y, z]).reshape(3,1), 
                        np.zeros((3,1)),
                          np.zeros((3,1)))

        return p, v
    
    def strip_found(self, t, T):

        if t < T / 3:
            x, y, z = self.get_strip_points(True)

            p, v = spline(t, T/3, np.array(self.coords_from).reshape((3,1)), 
                        np.array([x, y, z]).reshape(3,1), 
                        np.zeros((3,1)),
                          np.zeros((3,1)))
        elif t < 2*T/3:
            # needs to go up
            x, y, z = self.get_strip_points(True)

            p, v = spline(t-T/3, T/3, np.array([x, y, z]).reshape((3,1)), 
                        np.array([self.posex, self.posey, self.posez + 0.02]).reshape(3,1), 
                        np.zeros((3,1)),
                          np.zeros((3,1)))
        else:
            x, y, z = self.get_strip_points(False)
            p, v = spline(t-(2*T/3), T/3, np.array([self.posex, self.posey, self.posez + 0.02]).reshape((3,1)), 
                np.array([x, y, z]).reshape(3,1), 
                np.zeros((3,1)),
                    np.zeros((3,1)))
        return p, v

    
    def collision_detection(self, q, qdot):
        return (np.abs(np.array(self.actualJointPos).reshape((3, 1)) - np.array(q).reshape((3, 1))) > 1).any() or \
               (np.abs(np.array(self.actualJointVel).reshape((3, 1)) - np.array(qdot).reshape((3, 1))) > 1).any()  or \
               (np.abs(np.array(self.actualJointEff).reshape((3, 1)) - np.array(self.effort).reshape((3, 1))) > 1).any()
    
    # Send a command - called repeatedly by the timer.
    def sendcmd(self):
        # This is the time `now` when the sendcmd is called.
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        now = sec + nano*10**(-9)
        dt = (now - self.start_time) -self.t
        self.t = now - self.start_time

        if not self.hold and not self.go:
            if self.hold_point:   
                if self.t > HOLD_TIME:     
                    if self.detected:
                        self.joint_to_return_from = self.actualJointPos
                        self.vel_to_return_from = self.actualJointVel  
                        self.detected = False  
                    else:  
                        self.joint_to_return_from = self.q
                        self.vel_to_return_from = self.qdot   
                    self.hold_point = False
                    self.start_time = now
                    self.t = now - self.start_time

                q, qdot = self.q, self.qdot
            elif self.t >= GO_TO_START_T:
                # Enter hold
                self.go = False
                self.hold = True
                self.start_time = now
                self.t = now - self.start_time
            else:
                # Go to start
                q, qdot = self.start_to_idle(self.t, GO_TO_START_T)
                self.go = False
                self.hold = False
        if self.hold and not self.go:
            if self.did_circle:
                if self.pointx is not None:
                    # Go to point
                    self.go = True
                    self.hold = False
                    self.gox = self.pointx
                    self.goy = self.pointy
                    self.goz = self.pointz
                    self.joint_to_return_from = self.q 
                    self.vel_to_return_from = self.qdot
                    self.coords_from, _, _, _ = self.chain.fkin(self.q)

                    self.q = np.array(self.q).reshape((3, 1))
                    (self.pd, _, _, _) = self.chain.fkin(self.q)

                    self.pointx = None
                    self.pointy = None
                    self.pointz = None
                elif self.posex is not None:
                    self.strip_detected = True
                    self.go = True
                    self.hold = False
                    self.q = np.array(self.q).reshape((3, 1))
                    self.coords_from, _, _, _ = self.chain.fkin(self.q)

                    (self.pd, _, _, _) = self.chain.fkin(self.q)

                else:
                    # Hold
                    q, qdot = self.start_to_idle(GO_TO_START_T,  GO_TO_START_T)
                    self.hold = True
                    self.go = False
            else:
                if self.posex is not None:
                    self.strip_detected = True
                    self.go = True
                    self.hold = False
                    self.q = np.array(self.q).reshape((3, 1))
                    self.coords_from, _, _, _ = self.chain.fkin(self.q)

                    (self.pd, _, _, _) = self.chain.fkin(self.q)
                elif self.pointx is not None:
                    # Go to point
                    self.go = True
                    self.hold = False
                    self.gox = self.pointx
                    self.goy = self.pointy
                    self.goz = self.pointz
                    self.joint_to_return_from = self.q 
                    self.vel_to_return_from = self.qdot
                    self.coords_from, _, _, _ = self.chain.fkin(self.q)

                    self.q = np.array(self.q).reshape((3, 1))
                    (self.pd, _, _, _) = self.chain.fkin(self.q)

                    self.pointx = None
                    self.pointy = None
                    self.pointz = None
                else:
                    # Hold
                    q, qdot = self.start_to_idle(GO_TO_START_T,  GO_TO_START_T)
                    self.hold = True
                    self.go = False
            self.start_time = now
            self.t = now - self.start_time

        if self.go and not self.hold:
            if self.t >= GO_TO_POINT_T and not self.strip_detected or (self.t >= STRIP_GO_T):
                self.hold = False
                self.go = False
                self.start_time = now
                self.t = now - self.start_time

                self.joint_to_return_from = self.q
                self.vel_to_return_from = self.qdot
                self.hold_point = True

                q = self.q
                qdot = self.qdot
                self.strip_detected = False
                self.posex = None
                self.posey = None
                self.posez = None
            else:
                # Compute desired trajectory
                if self.strip_detected:
                    pd, vd = self.strip_found(self.t, STRIP_GO_T)
                else:
                    pd, vd = self.go_to_coords(self.t, GO_TO_POINT_T, self.gox, self.goy, self.goz)

                # Grab the last joint value and desired orientation.
                qlast  = self.q
                pdlast = self.pd

                # Compute the old forward kinematics.
                (p, R, Jv, Jw) = self.chain.fkin(qlast)

                # Set up the inverse kinematics.
                vr    = vd + self.lam * ep(pdlast, p)
                Jinv = Jv.T @ np.linalg.pinv(Jv @ Jv.T + self.gam**2 * np.eye(3))
                qdot = Jinv @ vr
                q = qlast + dt * qdot

                self.pd = pd

                # (p_actual, r, Jv, Jw) = self.chain.fkin(self.q)
                # error = ep(p_actual, p)
                # print(error)
                # gamma = self.gammaFactor
                # Jinv = Jv.T @ np.linalg.pinv(Jv @ Jv.T + gamma**2 * np.eye(3))

                # qdot = Jinv @ (v + self.lamFactor * error)
                # q = self.q + self.qdot*dt


        # print(np.array(self.actualJointPos).shape,  np.array(q).shape)
        if self.collision_detection(q, qdot) and not self.detected:
            self.detected = True
            self.go = False
            self.hold = False
            self.hold_point = True
            self.joint_to_return_from = q 
            self.vel_to_return_from = qdot

            self.start_time = now
            self.t = now - self.start_time

                # .23646962 * np.cos(q[1, 0])
        e = np.array([0.0, 1.60541142 * np.cos(self.actualJointPos[1]), 0.0])

        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = self.jointnames()
        self.cmdmsg.position     = list(q[:, 0]) #(np.nan, np.nan, np.nan) # #  (np.nan, np.nan, np.nan)
        self.cmdmsg.velocity     = list(qdot[:, 0]) #(np.nan, np.nan, np.nan) # # (np.nan, np.nan, np.nan)
        self.cmdmsg.effort       = list(e)
      
        self.q = q
        self.qdot = qdot
        self.effort = e

        # Save the joint value and desired values for next cycle.
        self.q  = q
        # Order 0-2: 3.3 Base, 3.5 shoulder, 3.4 Wrist

        
        # Build up the message and publish.
        self.cmdpub.publish(self.cmdmsg)



#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = DemoNode('demo1')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()