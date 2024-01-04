# Copyright 2023 Áron Svastits
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Pull the launch arguments out
    robot_model = LaunchConfiguration('robot_model')
    robot_family = LaunchConfiguration('robot_family')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    rsi_config_file = LaunchConfiguration('rsi_config_file')
    ros2_controller_config_file = LaunchConfiguration('ros2_controller_config_file')
    joint_traj_controller_config_file = LaunchConfiguration("joint_trajectory_controller_config_file")
    controller_to_load = LaunchConfiguration("controller_to_load")
    ns = LaunchConfiguration('namespace')
    # if there is an actual namespace, we'll want to append it with "_"
    # so that when it's put into the URDF as the prefix, it'll look nice
    namespace = ns.perform(context)
    if(namespace != ""):
        namespace = namespace + "_"
    
    # TODO(Svastits):better way to handle supported robot models and families
    if robot_model.perform(context) in ["kr6_r700_sixx", "kr6_r900_sixx", "kr8_r1640-2"]:
        robot_family = "agilus"
    elif robot_model.perform(context) in ["kr16_r2010_2"]:
        robot_family = "cybertech"
    elif robot_model.perform(context) in ["kr210_r2700_2", "kr210_r3100_2", "kr150_r3100-2"]:
        robot_family = "quantec"
    else:
        print("[ERROR] [launch]: robot model not recognized")
        raise Exception

    # Determine name of support package
    robot_support_package = PathJoinSubstitution(
        [FindPackageShare('kuka_{}_support'.format(robot_family)),
            "urdf", robot_model.perform(context) + ".urdf.xacro"]
    )

    print("Support package is ", robot_support_package.perform(context))
    # if rsi_config_file is the default value, then we need to make sure
    # it is expressed with respect to the robot_support_package. 
    if(rsi_config_file.perform(context) == "driver_config.yaml"):
        rsi_config_file = PathJoinSubstitution([FindPackageShare('kuka_kss_rsi_driver'), "config",
                                                rsi_config_file])
        
    print("RSI CONFIG FILE: ", rsi_config_file.perform(context))
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_support_package,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ", 
            "namespace:=",
            namespace,
            " ", 
            "rsi_config_file:=",
            rsi_config_file
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    controller_config = (get_package_share_directory('kuka_kss_rsi_driver') +
                         "/config/ros2_controller_config.yaml")
    print("Controller config file: ", controller_config)
    if(ros2_controller_config_file.perform(context) != "ros2_controller_config.yaml"):
        controller_config = ros2_controller_config_file.perform(context)
        
    joint_traj_controller_config = (get_package_share_directory('kuka_kss_rsi_driver') +
                                    "/config/joint_trajectory_controller_config.yaml")
    print("JTController config file: ", joint_traj_controller_config)

    if(joint_traj_controller_config_file.perform(context) != "joint_trajectory_controller_config.yaml"):
        joint_traj_controller_config = joint_traj_controller_config_file.perform(context)
    
    controller_manager_node = ns.perform(context) + '/controller_manager'
    print("FINAL Controller config file: ", controller_config)
    print("FINAL JTController config file: ", joint_traj_controller_config)

    control_node = Node(
        namespace=ns.perform(context),
        package='kuka_drivers_core',
        executable='control_node',
        parameters=[robot_description, controller_config, joint_traj_controller_config]
    )
    robot_manager_node = LifecycleNode(
        name=['robot_manager'],
        namespace=ns.perform(context),
        package="kuka_kss_rsi_driver",
        executable="robot_manager_node",
        parameters=[{'robot_model': robot_model},
                    {'controller_to_load': controller_to_load.perform(context)}]
    )
    robot_state_publisher = Node(
        namespace=ns.perform(context),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # Spawn controllers
    def controller_spawner(controller_with_config, activate=False):
        arg_list = [controller_with_config[0], "-c", controller_manager_node, "-p",
                    controller_with_config[1], "-n", ns.perform(context)]
        if not activate:
            arg_list.append("--inactive")
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=arg_list
        )

    controller_names_and_config = [
        ("joint_state_broadcaster", []),
        ("joint_trajectory_controller", joint_traj_controller_config),
    ]

    controller_spawners = [controller_spawner(controllers)
                           for controllers in controller_names_and_config]

    nodes_to_start = [
        control_node,
        robot_manager_node,
        robot_state_publisher
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument(
        'robot_model',
        default_value='kr6_r700_sixx',
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'robot_family',
        default_value='agilus'
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'use_fake_hardware',
        default_value="false"
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'namespace',
        default_value=""
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'rsi_config_file',
        default_value="driver_config.yaml",
        description="full file path to driver_config.yaml, or equivalent. "+
        "This file must contain the client IP and client port of the HW to connect to. \n" +
        "The default will be appended to ""$(this driver's share dir)/config"""
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'joint_trajectory_controller_config_file',
        default_value="joint_trajectory_controller_config.yaml",
        description="Full file path to a yaml file containing the configuration info for joint_trajectory_controller."+
        " The default will once again be inside of kuka_kss_rsi_driver/config"
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'ros2_controller_config_file',
        default_value="ros2_controller_config.yaml",
        description="Full file path pointing to a yaml file containing the configuration for the " +
        " controller manager node. The default will be gotten from kuka_kss_rsi_driver/config"
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'controller_to_load',
        default_value="joint_trajectory_controller",
        description="Motion controller (that is, excluding joint_state_broadcaster) to load. " +
        "Must match the controller named in joint_trajectory_controller_config.yaml " +
        "and ros2_controller_config.yaml"
    ))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
