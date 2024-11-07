import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node



def generate_launch_description():
    testbed_pkg_share = get_package_share_directory('plot_joint_states')

    yaml_file = os.path.join(testbed_pkg_share, 'config/plotter.yaml')

    plotter = Node(
        package='plot_joint_states',
        executable='joint_states_plotter',
        name='joint_states_plotter',
        output='screen',
        parameters=[{'yaml_file': yaml_file}]
    )

    args = [
        plotter
    ]
    
    return LaunchDescription(args)



if __name__ == '__main__':
    generate_launch_description()