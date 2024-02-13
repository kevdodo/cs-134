# Copy of the realsense align launch file used to attempt to subscribe to all 
# needed nodes.
import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription
from launch.actions                    import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Use the standard realsense launch description.  But override
    # what we need.
    rsfile = os.path.join(pkgdir('realsense2_camera'), 'launch/rs_launch.py')

    # Profile is Width x Height x FPS.  0 is default.
    rsargs = {'camera_name':             'camera',  # camera unique name
              'depth_module.profile':    '0,0,0',   # depth W, H, FPS
              'rgb_camera.profile':      '0,0,0',   # color W, H, FPS
              'enable_color':            'true',    # enable color stream
              'enable_infra1':           'false',   # enable infra1 stream
              'enable_infra2':           'false',   # enable infra2 stream
              'enable_depth':            'true',    # enable depth stream
              'align_depth.enable':      'true',    # enabled aligned depth
              'pointcloud.enable':       'false',   # Turn on point cloud
              'allow_no_texture_points': 'true'}    # All points without texture

    incl_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsfile),
        launch_arguments=rsargs.items())
    
    node_camera = Node(
        name       = 'eyes', 
        package    = 'bmoHanoi',
        executable = 'bmoHanoi',
        output     = 'screen'
        )
    
    node_hebi = Node(
        name       = 'hebi', 
        package    = 'hebiros',
        executable = 'hebinode',
        output     = 'screen',
        parameters = [{'family':   'robotlab'},
                      {'motors':   ['3.6',  '1.7',      '3.5',   '3.4',   '3.3']},
                      {'joints':   ['base', 'shoulder', 'elbow', 'wrist', 'head']}],
        on_exit    = Shutdown())

    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([

        # Start the nodes.
        incl_realsense,
        node_hebi,
        node_camera,
    ])
