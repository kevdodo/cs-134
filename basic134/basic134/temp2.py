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


import time
from demo134.TrajectoryUtils import *
#
#   Definitions
#
RATE = 100.0            # Hertz

GO_TO_START_T = 5.0    # Seconds

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

        # The indicate that at the beginning of the program BMO is both at the 
        # beginning of a task and that that task is to touch as opposed to return
        self.start = True
        self.touch = True
        self.init  = True
        # Additionally the self.t begins at 0.0
        self.t     = 0.0

        self.idlePosition = np.radians([0,45,180])
        self.taskPos0 = None

        self.userPosition = None

        self.chain = KinematicChain(self, 'world', 'tip', self.jointnames())
       
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


    # Receive feedback - called repeatedly by incoming messages.
    def recvfbk(self, fbkmsg):
        self.actualJointPos   = list(fbkmsg.position)
        self.actualJointVel   = list(fbkmsg.velocity)
        self.actualEff        = list(fbkmsg.effort)
        self.grabready = True
        return 
    
    def start_to_idle(self, t, T):
        e, ed = spline(t, T - 1.0, np.zeros((3)), 
                        np.array([0, 1.71, 0]), np.zeros((3)), np.zeros((3)))
        q, qDot = spline(t, T, np.array(self.jointPos0), 
                        self.idlePosition, np.zeros((3)), np.zeros((3))) 
        
        return q, qDot, e

        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = self.jointnames()
        self.cmdmsg.position     = list(q)
        self.cmdmsg.velocity     = list(qDot)
        self.cmdmsg.effort       = list(e)
        self.cmdpub.publish(self.cmdmsg)
    
    # Send a command - called repeatedly by the timer.
    def sendcmd(self):
        # This is the time `now` when the sendcmd is called.
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        now = sec + nano*10**(-9)

        if (self.start and self.init):
          self.start_time = now
          self.start = False
        # First thing's first BMO splines in joint space to the idleJoinPos.
        elif (self.t > GO_TO_START_T) and self.init:
          self.init = False
        if(self.t > GO_TO_START_T):
          self.start = True
          print("end init", self.start, self.touch)
              
        # If the start and touch are both true then the user is prompted to
        # enter the x and y coordinates of the touch point. The start_time is
        # set to the current time and the starting is set to false so the next 
        # time sendcmd is called the if statement will not be entered. 
        if self.start and self.touch:
            print('x:')
            x = x.fromSub
            print('y:')
            y = 0.4
            self.starting = False
            self.start_time = now
            self.t = 0
            self.Start = False
            self.jointPos0 = self.grabfbk()
            self.taskPos0 = self.chain.fkin(self.jointPos0)
            userX = x
            userY = y
            self.userPosition = np.array([userX, userY, 0]) 
        # If the start is false and the touch is true then
        # the function sets self.t to 0 and the jointPos0 to the current 
        # position in order to. 
            
        # this is if you hit something / touched no problem
        elif (not self.touch) and self.start:
            self.t = 0
            self.jointPos0 = self.grabfbk()
            self.taskPos0 = self.chain.fkin(self.jointPos0)
            self.start = False
            self.start_time = now
        
        # This will update the t and dt and save the time as a class variable
        #  to access on the next call.
        elapsedTime = now - self.start_time
        dt = elapsedTime - self.t
        self.t = elapsedTime
        # In this case the robot is moving to the touch point. The position and 
        # velocity are calculated using the spline function.
        if (self.t <= GO_TO_T and self.touch):
            self.start = False
            p, v = spline(self.t, GO_TO_T, np.array(self.taskPos0), 
                          np.array(self.userPosition), np.zeros((3)), np.zeros((3)))
            # If the robot bumps into somthing causing discrepancy between the
            # actual and the desired then the touch is set to false and BMO will
            # return to the idle position.
            if abs(self.actualJointPos - self.q) > P_THRESHOLD or \
               abs(self.actualJointVel - self.qDot) > V_THRESHOLD or \
               abs(self.actualEff - self.effort) > EFFORT_THRESHOLD:
                self.touch = False
                self.start = True
        # If self.touch is false then BMO will be returning to idle position.
        elif (self.t <= GO_TO_T and not self.touch):
            p, v = spline(self.t, GO_TO_T, np.array(self.taskPos0), 
                          np.array(self.idlePosition), np.zeros((3)), np.zeros((3)))
            self.start = False
        # Once we have touched the target self.touch will be set to true and 
        # start will be false so BMo may begin her journey back to idle.
        elif self.touch:
            self.touch = False
            self.start = True
        # If BMO has returned back to idle she will set touch and start to true
        # in order to go to a new touch target.
        elif not self.touch:
            self.touch = True
            self.start = True
          
        (p_actual, r, Jv, Jw) = self.chain1.fkin(self.q)
        # e = np.vstack((ep(self.pr, pr), eR(self.Rr, Rr)))
        e = ep(self.q, self.actualJointPos)
        gamma = self.gammaFactor
        lam = self.lamFactor
        Jinv = Jv.T @ np.linalg.pinv(Jv @ Jv.T + gamma**2 * np.eye(3))

        qdot = Jinv @ v

        # x_dot = J qdot
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = ['one', 'two', 'three']
        self.cmdmsg.position     = list(q)
        self.cmdmsg.velocity     = list(qdot)
        self.cmdmsg.effort       = list([0, 1.8 * np.sin(q[1]), 0])
      
        self.q = q
        self.qDot = qDot
        self.effort = [0, 1.8 * np.sin(q[1]), 0]
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
    node = DemoNode('demo')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
