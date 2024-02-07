'''hw7p4sol.py

   This is the solution code for HW7 Problem 4.

   This explores the singularity handing while following a circle
   outside the workspace.

   This builds off Problem 3(a).

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState
                /condition              std_msgs/Float64

'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TransformHelpers   import *
from hw5code.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from hw5sols.KinematicChainSol  import KinematicChain

# Import the format for the condition number message
from std_msgs.msg               import Float64


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Setup up the condition number publisher
        self.pub = node.create_publisher(Float64, '/condition', 10)

        # Define the various points.
        angle = 46.56746
        self.q0 = np.radians(
            np.array([0, angle, 0, -2*angle, 0, 0, angle]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.7, 0.6]).reshape((-1,1))
        self.R0 = Reye()

        # Initialize the current/starting joint position and set the
        # desired tip position/orientation to match.
        self.q  = self.q0
        self.pd = self.p0
        self.Rd = self.R0
        # (self.pd, self.Rd, _, _) = self.chain.fkin(self.q)

        # Pick the sub-problem/part/mode
        self.mode = 'Pseudo'          # Pseudo Inverse only
        # self.mode = 'Weighted'        # Weighted pseudo inverse
        # self.mode = 'Secondary'       # Secondary task as well
        node.get_logger().info("Mode = " + self.mode)

        # Pick the convergence bandwidth.
        self.lam = 20

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1','theta2','theta3','theta4','theta5','theta6','theta7']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # # End after 2pi s.
        # if t>2*pi:
        #     return None

        # Compute the desired trajectory.
        pd = np.array([0,
                       0.95 - 0.25*cos(t),
                       0.60 + 0.25*sin(t)]).reshape((3,1))
        vd = np.array([0, 0.25*sin(t), 0.25*cos(t)]).reshape((3,1))

        Rd = Reye()
        wd = np.zeros((3,1))

        # Grab the last joint value and desired orientation.
        qlast  = self.q
        pdlast = self.pd
        Rdlast = self.Rd

        # Compute the old forward kinematics.
        (p, R, Jv, Jw) = self.chain.fkin(qlast)

        # Set up the inverse kinematics.
        vr    = vd + self.lam * ep(pdlast, p)
        wr    = wd + self.lam * eR(Rdlast, R)
        J     = np.vstack((Jv, Jw))
        xrdot = np.vstack((vr, wr))

        # Compute the inverse kinematics according to the sub-problem:
        if self.mode == 'Pseudo':
            # Part (a): pseudo-inverse
            Jinv  = np.linalg.pinv(J)
            qdot  = Jinv @ xrdot

        elif self.mode == 'Weighted':
            # Part (b): Weighted Pseudo Inverse
            gamma = 0.1
            Jinv = J.T @ np.linalg.pinv(J @ J.T + gamma**2 * np.eye(6))
            qdot = Jinv @ xrdot

        elif self.mode == 'Secondary':
            # Part (c): Weighted Pseudo Inverse PLUS Secondary Task
            gamma = 0.1
            Jinv = J.T @ np.linalg.pinv(J @ J.T + gamma**2 * np.eye(6))

            lams = 10.0
            qsdot = np.zeros((7,1))
            qsdot[3,0] = lams * (-np.pi/2 - qlast[3,0])

            qdot = Jinv @ xrdot + (np.eye(7) - Jinv @ J) @ qsdot

        else:
            raise Exception("Unknown Singularity Mode")
        
        # Integrate the joint position.
        q = qlast + dt * qdot

        # Save the joint value and desired values for next cycle.
        self.q  = q
        self.pd = pd
        self.Rd = Rd

        # Compute the condition number.
        L = 0.4
        for row in range(3):
            J[row,:] = J[row,:]/L
        condition = np.linalg.cond(J)

        # Publish the condition number.
        msg = Float64()
        msg.data = condition
        self.pub.publish(msg)

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
