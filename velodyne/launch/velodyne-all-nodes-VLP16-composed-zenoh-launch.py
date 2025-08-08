import os
import yaml

import ament_index_python.packages
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    with open(driver_params_file, 'r') as f:
        driver_params = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']

    convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
        convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')

    # Make sure you have installed the zenoh router first
    #
    # sudo apt install ros-<distro>-rmw-zenoh-cpp
    # source /opt/ros/<distro>/setup.bash
    #
    # then run 
    #
    # export RMW_IMPLEMENTATION=rmw_zenoh_cpp 
    # 
    # from your terminal
    zenoh_router_action = ExecuteProcess(
        cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
        name='rmw_zenohd',
        output='screen'
    )

    container = Node(
        name='velodyne_container',
        package='rclcpp_components',
        executable='component_container',
        output='screen'
    )
    load_composable_nodes = LoadComposableNodes(
        target_container='velodyne_container',
        composable_node_descriptions=[
            ComposableNode(
                package='velodyne_driver',
                plugin='velodyne_driver::VelodyneDriver',
                name='velodyne_driver_node',
                parameters=[driver_params]),
            ComposableNode(
                package='velodyne_pointcloud',
                plugin='velodyne_pointcloud::Transform',
                name='velodyne_transform_node',
                parameters=[convert_params]),
        ])

    return LaunchDescription([zenoh_router_action,
                              container,
                              RegisterEventHandler(
                                event_handler=OnProcessStart(
                                target_action=container,
                                on_start=[load_composable_nodes],
                              )),
                            ])