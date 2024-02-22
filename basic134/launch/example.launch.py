"""Example Launch File

   This launch file is intended show how the pieces come together.

   This could start
     1) RVIZ, ready to view the robot
     2) The robot_state_publisher (listening to the actual or commands)
     3) The HEBI node to communicate with the motors (slow or normal)
     4) The demo (placeholder for your code) or gui to issues commands

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import Shutdown
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('basic134'), 'rviz/viewurdf.rviz')

    # Locate/load the robot's URDF file (XML).
    urdf = os.path.join(pkgdir('basic134'), 'urdf/threedofexample.urdf')
    with open(urdf, 'r') as file:
        robot_description = file.read()


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher_ACTUAL = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])

    node_robot_state_publisher_COMMAND = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}],
        remappings = [('/joint_states', '/joint_commands')])

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())

    # Configure a node for the hebi interface.  Note the 200ms timeout
    # is useful as the GUI only runs at 10Hz.
    node_hebi_SLOW = Node(
        name       = 'hebi', 
        package    = 'hebiros',
        executable = 'hebinode',
        output     = 'screen',
        parameters = [{'family':   'robotlab'},
                      {'motors':   ['3.3',  '3.5',      '3.4']},
                      {'joints':   ['base', 'shoulder', 'elbow']},
                      {'lifetime': 200.0}],
        on_exit    = Shutdown())

    node_hebi = Node(
        name       = 'hebi', 
        package    = 'hebiros',
        executable = 'hebinode',
        output     = 'screen',
        parameters = [{'family':   'robotlab'},
                      {'motors':   ['3.3',  '3.5',      '3.4']},
                      {'joints':   ['base', 'shoulder', 'elbow']}],
        on_exit    = Shutdown())

    # Configure a node for the simple demo.  PLACEHOLDER FOR YOUR CODE!!
    node_demo = Node(
        name       = 'color_touch', 
        package    = 'basic134',
        executable = 'color_touch',
        output     = 'screen')

    # Configure a node for the GUI to command the robot.
    node_gui = Node(
        name       = 'gui', 
        package    = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
        output     = 'screen',
        remappings = [('/joint_states', '/joint_commands')],
        on_exit    = Shutdown())


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # STEP 1: Start if you just want to see the URDF.
        #node_robot_state_publisher_COMMAND,
        #node_rviz,
        #node_gui,

        # # STEP 2: Start if you just want to watch the actual robot.
        # node_robot_state_publisher_ACTUAL,
        # node_rviz,
        # node_hebi,

        # # STEP 3: Start if we want the demo code to command the robot.
        # node_robot_state_publisher_ACTUAL,
        # node_rviz,
        # node_hebi,
        # node_demo,

        # # ALTERNATE: Start if we want the GUI to command the robot.
        # # THIS WILL BE **VERY** JITTERY, running at 10Hz!
        # node_robot_state_publisher_ACTUAL,
        # node_rviz,
        # node_hebi_SLOW,
        # node_gui,

        # # ALTERNATE: Start if we want RVIZ to watch the commands.
        node_robot_state_publisher_COMMAND,
        node_rviz,
        node_hebi,
        node_demo,
    ])
