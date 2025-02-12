from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    mir_nav_dir = get_package_share_directory('mir_navigation')
    

    def find_map_file(context):
        map_arg = context.launch_configurations['map']
        print(f"trying to find map file with with arg {map_arg}")
        print(f"trying to find map at: {os.path.join(mir_nav_dir,'maps', map_arg)}")
        if os.path.isfile(os.path.join(mir_nav_dir, 'maps', map_arg)):
            print(f"map found in mir nav dir with path: \n{os.path.join(mir_nav_dir,'maps', map_arg)}")
            return [SetLaunchConfiguration('map_file', os.path.join(mir_nav_dir, 'maps', map_arg))]

        elif os.path.isfile(map_arg):
            print(f"using full map path")
            return [SetLaunchConfiguration('map_file', map_arg)]

    declare_map_file_argument = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(mir_nav_dir, 'maps', 'workspace1_map.yaml'),
        description='Relative path to map in mir_navigation/maps or full path to map (yaml).',
    )
    
    
    webui = Node(package="grasping_navigator_web_ui",
                 executable="webui",
                 parameters=[{"map_path":LaunchConfiguration('map')}],
                 output = "screen")



    ld = LaunchDescription()
    

    ld.add_action(declare_map_file_argument)
    ld.add_action(OpaqueFunction(function=find_map_file))
    ld.add_action(webui)    
    
    return ld
