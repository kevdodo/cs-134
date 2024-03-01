'''hw6p5sol.py

   This is the solution code for HW6 Problem 5.

   This uses the inverse kinematics from Problem 4, but adds a more
   complex trajectory.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

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


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Define the various points.
        self.q0 = np.radians(np.array([0, 90, -90, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1,1))
        self.R0 = Reye()

        self.pleft  = np.array([ 0.3, 0.5, 0.15]).reshape((-1,1))
        self.pright = np.array([-0.3, 0.5, 0.15]).reshape((-1,1))
        self.Rleft  = Rotx(-np.pi/2) @ Roty(-np.pi/2)
        self.Rleft  = Rotz( np.pi/2) @ Rotx(-np.pi/2)
        self.Rright = Reye()

        # Initialize the current/starting joint position and set the
        # desired tip position/orientation to match.
        self.q  = self.q0
        self.pd = self.p0
        self.Rd = self.R0
        # (self.pd, self.Rd, _, _) = self.chain.fkin(self.q)

        # Pick the convergence bandwidth.
        self.lam = 20

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # End after 13s.
        if t>13:
            return None

        # Decide which phase we are in:
        if t < 3.0:
            # Approach movement:
            (s0, s0dot) = goto(t, 3.0, 0.0, 1.0)

            pd = self.p0 + (self.pright - self.p0) * s0
            vd =           (self.pright - self.p0) * s0dot

            Rd = Reye()
            wd = np.zeros((3,1))

        else:
            # Cyclic (sinusoidal) movements, after the first 3s.
            s    =            cos(pi/2.5 * (t-3))
            sdot = - pi/2.5 * sin(pi/2.5 * (t-3))

            # Use the path variables to compute the position trajectory.
            pd = np.array([-0.3*s    , 0.5, 0.75-0.6*s**2  ]).reshape((3,1))
            vd = np.array([-0.3*sdot , 0.0,     -1.2*s*sdot]).reshape((3,1))

            # Choose one of the following methods to compute orientation.
            if False:
                alpha    = - pi/4 * (s-1)
                alphadot = - pi/4 * sdot

                Rd = Rotx(-alpha) @ Roty(-alpha)
                wd = (- ex() - Rotx(-alpha) @ ey()) * alphadot

            elif False:
                alpha    = - pi/4 * (s-1)
                alphadot = - pi/4 * sdot
                
                Rd = Rotz(alpha) @ Rotx(-alpha)
                wd = (ez() - Rotz(alpha) @ ex()) * alphadot

            else:
                alpha    = - pi/3 * (s-1)
                alphadot = - pi/3 * sdot

                eleft = np.array([1, 1, -1]).reshape((3,1)) / sqrt(3)
                Rd    = Rote(eleft, -alpha)
                wd    = - eleft * alphadot

        # Grab the last joint value and desired orientation.
        qlast  = self.q
        pdlast = self.pd
        Rdlast = self.Rd

        # Compute the old forward kinematics.
        (p, R, Jv, Jw) = self.chain.fkin(qlast)

        # Compute the inverse kinematics
        vr    = vd + self.lam * ep(pdlast, p)
        wr    = wd + self.lam * eR(Rdlast, R)
        J     = np.vstack((Jv, Jw))
        xrdot = np.vstack((vr, wr))
        qdot  = np.linalg.inv(J) @ xrdot

        # Integrate the joint position.
        q = qlast + dt * qdot

        # Save the joint value and desired values for next cycle.
        self.q  = q
        self.pd = pd
        self.Rd = Rd

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
