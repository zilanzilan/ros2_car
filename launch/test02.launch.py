from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明全局目标点参数，允许在命令行中传入
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='0.0',
        description='X coordinate of the global goal'
    )
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='0.0',
        description='Y coordinate of the global goal'
    )

    # 节点1：激光障碍物检测器
    obstacle_detector_node = Node(
        package='test',          # 替换为实际包名
        executable='obstacle_dist',           # 编译后的可执行文件名称
        name='laser_obstacle_detector',
        output='screen',
        parameters=[{
            'robot_namespace': '',            # 命名空间参数（可留空）
            'obstacle_threshold': 2.0,
            'cluster_distance_threshold': 0.12
        }]
    )

    # 节点2：全局目标发布节点（核心向量场算法）
    goal_pub_node = Node(
        package='test',
        executable='goal_pub',
        name='goal_pub',
        output='screen',
        # 用 parameters 传递
        parameters=[{
            'namespace': '',
            'goal_x': LaunchConfiguration('goal_x'),  # 关键：映射到 ROS 参数
            'goal_y': LaunchConfiguration('goal_y')   # 关键：映射到 ROS 参数
        }]
    )

    # 节点3：局部目标点转速度指令节点
    twist_pub_node = Node(
        package='test',
        executable='twist_pub',                 # 编译后的可执行文件名称
        name='local_goal_to_cmd_vel',
        output='screen',
        parameters=[{
            'v_max': 0.5,
            'omega_max': 1.0,
            'angle_threshold': 0.1,
            'k_angular': 2.0,
            'stop': False
        }]
    )

    return LaunchDescription([
        goal_x_arg,
        goal_y_arg,
        obstacle_detector_node,
        goal_pub_node,
        twist_pub_node
    ])