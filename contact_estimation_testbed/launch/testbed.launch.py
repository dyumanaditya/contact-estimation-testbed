import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node



def generate_launch_description():
    testbed_pkg_share = get_package_share_directory('contact_estimation_testbed')

    mujoco_file = os.path.join(testbed_pkg_share, 'urdf/mujoco/silver_badger_testbed.xml')

    testbed = Node(
        package='contact_estimation_testbed',
        executable='contact_estimation_testbed',
        name='contact_estimation_testbed',
        output='screen',
        parameters=[{'mujoco_file': mujoco_file}]
    )

    args = [
        testbed
    ]
    
    return LaunchDescription(args)



if __name__ == '__main__':
    generate_launch_description()