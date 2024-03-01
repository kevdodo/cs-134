'''hw6p3sol.py

   This is the solution code for HW6 Problem 3.

   This creates a purely rotational movement.

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

        # Initialize the current joint position to the starting
        # position and set the desired orientation to match.
        self.q = np.zeros((3,1))
        (_, self.Rd, _, _) = self.chain.fkin(self.q)

        # Pick the convergence bandwidth.
        self.lam = 20

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['pan', 'tilt', 'roll']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # End after 15s.
        if t>15:
            return None

        # Choose the alpha/beta angles based on the phase.
        if t <= 2.0:
            # Part A (t<=2):
            (alpha, alphadot) = goto(t, 2.0, 0.0, -pi/2)
            (beta,  betadot)  = (0.0, 0.0)
        else:
            # Part B (t>2):
            (alpha, alphadot) = (-pi/2, 0.0)
            (beta,  betadot)  = (t-3+exp(2-t), 1-exp(2-t))

        # Set up the desired rotation and angular velocity.
        # You can use either - they compute the same numbers.
        if False:
            # Use the diagonal axis expressed in the A frame.
            eA = exyz(0.0, 1/sqrt(2), -1/sqrt(2))
            Rd = Roty(alpha) @ Rote(eA, beta)

            wd = ey() * alphadot + Roty(alpha) @ eA * betadot
        else:
            # Use the diagonal axis expressed in the O frame.
            eO = exyz(1/sqrt(2), 1/sqrt(2), 0.0)
            Rd = Rote(eO, beta) @ Roty(alpha)

            wd = eO * betadot + Rote(eO, beta) @ ey() * alphadot

        # Grab the last joint value and desired orientation.
        qlast  = self.q
        Rdlast = self.Rd

        # Compute the old forward kinematics.
        (_, R, _, Jw) = self.chain.fkin(qlast)

        # Compute the inverse kinematics
        wr   = wd + self.lam * eR(Rdlast, R)
        qdot = np.linalg.inv(Jw) @ wr

        # Integrate the joint position.
        q = qlast + dt * qdot

        # Save the joint value and desired orientation for next cycle.
        self.q  = q
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
