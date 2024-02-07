#!/usr/bin/env python3
#
#   demo134.py
#
#   Demonstration node to interact with the HEBIs.
#
import numpy as np
import rclpy

from rclpy.node         import Node
from sensor_msgs.msg    import JointState

import time
from demo134.TrajectoryUtils import *
#
#   Definitions
#
RATE = 100.0            # Hertz

GO_TO_START_T = 5.0    # Seconds
#
#   DEMO Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a temporary subscriber to grab the initial position.
        self.position0 = self.grabfbk()
        self.get_logger().info("Initial positions: %r" % self.position0)

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

        self.starting = True
        self.t = None
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
        # Just print the position (for now).
        # print(list(fbkmsg.position))
        pass

    # Send a command - called repeatedly by the timer.
    def sendcmd(self):
        # this is the previous time
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        now = sec + nano*10**(-9)
        
        if self.starting:
            self.start_time = now
            self.starting = False
        self.t = now 
        dt = now - self.t 


        t = self.t - self.start_time

        PHASE = .1

        end_init_pos = np.array([0.0, .05 * np.sin(1/3 * 
            np.pi * PHASE), .3 * np.sin(1.5 * np.pi * (2 * PHASE))])


        if t < GO_TO_START_T - 1.0:
            p, v = spline(t, GO_TO_START_T - 1.0, np.array(self.position0), end_init_pos, np.zeros((3)), np.zeros((3)))
            e, ed = spline(t, GO_TO_START_T - 1.0, np.zeros((3)), np.array([0, 1.71, 0]), np.zeros((3)), np.zeros((3)))
        elif t < GO_TO_START_T:
            p, v = spline(GO_TO_START_T - 1.0, GO_TO_START_T - 1.0, np.array(self.position0), end_init_pos, np.zeros((3)), np.zeros((3)))   
            e = np.array([0, 1.71, 0])          
        else:
            p, v = spline(GO_TO_START_T, GO_TO_START_T, np.array(self.position0), np.zeros((3)), np.zeros((3)), np.zeros((3)))
            e = np.array([0, 1.71, 0])
            p[0] = .2 * np.sin( np.pi * (t - GO_TO_START_T))
            p[1] = .05 * np.sin(1/3 * np.pi * (t - GO_TO_START_T + PHASE))
            p[2] = .3 * np.sin(1.5 * np.pi * (t - GO_TO_START_T + 2 * PHASE))

            v[0] = .2 * np.pi * np.cos(np.pi * (t - GO_TO_START_T))
            v[1] = .05 * 1/3 * np.pi * np.cos(1/3 * np.pi * (t - GO_TO_START_T + PHASE))
            v[2] = .3 * 1.5 * np.pi * np.cos(1.5 * np.pi * (t - GO_TO_START_T + 2 * PHASE))

        # Order 0-2: 3.3 Base, 3.5 shoulder, 3.4 Wrist

        
        # Build up the message and publish.
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = ['base', 'shoulder', 'elbow']
        self.cmdmsg.position     = list(p)
        self.cmdmsg.velocity     = list(v)
        self.cmdmsg.effort       = list(e)
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
