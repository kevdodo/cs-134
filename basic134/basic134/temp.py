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
from geometry_msgs.msg          import Point


import time
from demo134.TrajectoryUtils import *
from demo134.TransformHelpers import *

#
#   Definitions
#
RATE = 100.0            # Hertz

GO_TO_START_T = 5.0    # Seconds
GO_TO_POINT_T = 5.0      # Seconds
GO_TO_T = 10.0    # Seconds
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
        self.go = False

        # This is to get point coordinates
        self.fbksub = self.create_subscription(
            Point, '/point', self.recvpoint, 10)
        self.pointx = []
        self.pointy = []
        self.pointz = []
        self.gox = 0.0
        self.goy = 0.0
        self.goz = 0.0
        # Report.
        self.get_logger().info("Running %s" % name)
        self.chain = KinematicChain('world', 'tip', self.jointnames())
        self.idleJointPos = np.radians([0,70,180]).reshape((3,1))
        self.idlePosition, _, _, _ = self.chain.fkin(self.idleJointPos)

        self.t = 0.0

        sec, nano = self.get_clock().now().seconds_nanoseconds()
        self.start_time = sec + nano*10**(-9)

        self.gammaFactor = .1
        self.lamFactor = .1
       
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
        self.pointx.append(pointmsg.x)
        self.pointy.append(pointmsg.y)
        self.pointz.append(pointmsg.z)
        
        
    # Receive feedback - called repeatedly by incoming messages.
    def recvfbk(self, fbkmsg):
        self.actualJointPos   = list(fbkmsg.position)
        self.actualJointVel   = list(fbkmsg.velocity)
        self.actualJointEff        = list(fbkmsg.effort)
        self.grabready = True
    
    def start_to_idle(self, t, T):
        q, qDot = spline5(t, T, np.array(self.q).reshape(3,1), 
                        self.idleJointPos.reshape(3,1), np.zeros((3, 1)), 
                        np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1))) 
        return q, qDot

    def go_to_coords(self, t, T, x, y, z):
        p, v = spline(t, T, np.array(self.idlePosition).reshape((3,1)), 
                        np.array([x, y, z]).reshape(3,1), np.zeros((3, 1)), np.zeros((3,1)))

        return p, v
    
    # Send a command - called repeatedly by the timer.
    def sendcmd(self):
        # This is the time `now` when the sendcmd is called.
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        now = sec + nano*10**(-9)
        dt = (now - self.start_time) -self.t
        self.t = now - self.start_time

        if not self.hold and not self.go:
            if self.t >= GO_TO_START_T:
                self.go = False
                self.hold = True
                self.start_time = now
                self.t = now - self.start_time
            else:
                q, qdot = self.start_to_idle(self.t, GO_TO_START_T)
        if self.hold and not self.go:
            if len(self.pointx) > 0:
                self.go = True
                self.hold = False
                self.gox = self.pointx.pop(0)
                self.goy = self.pointy.pop(0)
                self.goz = self.pointz.pop(0)

            else:
                q, qdot = self.start_to_idle(GO_TO_START_T, GO_TO_START_T)

            self.start_time = now
            self.t = now - self.start_time

        if self.go and not self.hold:
            if self.t >= GO_TO_POINT_T:

                self.hold = False
                self.go = False
                self.start_time = now
                self.t = now - self.start_time                
                q = self.q
                qdot = self.qdot
            else:

                p, v = self.go_to_coords(self.t, GO_TO_POINT_T, self.gox, self.goy, self.goz)
                (p_actual, r, Jv, Jw) = self.chain.fkin(self.q)
                e = ep(p_actual, p)
                gamma = self.gammaFactor
                Jinv = Jv.T @ np.linalg.pinv(Jv @ Jv.T + gamma**2 * np.eye(3))

                qdot = Jinv @ (v + self.lamFactor * e)


                q = self.q + self.qdot*dt
        if abs(np.array(self.actualJointPos[1]) - self.q[1]) > 2 or \
               abs(np.array(self.actualJointVel[1]) - self.qdot[1]) > 2 or \
               abs(np.array(self.actualJointEff[1]) - self.effort[1]) > 2:
                self.go = False
                self.hold = False
                print("a;lskdfja;lksnv")
        # x_dot = J qdot
        e = np.array([0.0, 1.23646962 * np.cos(q[1, 0]), 0.0])
        self.q = q

        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = self.jointnames()
        self.cmdmsg.position     = list(self.q[:, 0])
        self.cmdmsg.velocity     = list(qdot[:, 0])
        self.cmdmsg.effort       = list(e)
      
        self.q = q
        self.qdot = qdot
        self.effort = e
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
