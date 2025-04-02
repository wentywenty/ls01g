from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('ls01g')
    
    # 修改文件路径和读取方式
    urdf_file = os.path.join(pkg_dir, 'xacro', 'ls01g.urdf.xacro')
    # print(f"Loading URDF from: {urdf_file}")

    # 使用 xacro 处理而不是直接读取
    robot_description = Command(['xacro ', urdf_file])

    # 使用直接读取文件
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    # 声明参数
    inverted_arg = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='如果0度方向在串口线的方向上设置为true'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        # default_value='/dev/laser',
        description='激光雷达串口设备'
    )
    
    # 创建激光雷达节点
    ls01g_node = Node(
        package='ls01g',
        executable='ls01g_node',
        name='ls01g_node',
        output='screen',
        parameters=[{
            'scan_topic': 'scan',                    # 设置激光数据topic名称
            'laser_link': 'base_laser_link',         # 激光坐标
            'serial_port': LaunchConfiguration('port'),  # 雷达连接的串口
            'zero_as_max': False,                    # 设置为true探测不到区域会变成最大值
            'min_as_zero': False,                    # true：探测不到区域为0，false：探测不到区域为inf
            'angle_disable_min': -1.0,               # 角度制，从angle_disable_min到angle_disable_max之前的值为0
            'angle_disable_max': -1.0,
            'inverted': LaunchConfiguration('inverted'),
        }]
    )
    
    # 创建机器人状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen'
    # )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen'
    # )

    # 添加静态变换发布器 map -> base_laser_link
    tf_static_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_laser_tf',
        arguments=['0.5', '0.0', '0.2', '1.57079632679', '0.0', '0.0', 'map', 'base_laser_link']
    )
    
    # 添加RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'ls01g.rviz')]
    )
    
    # 返回启动描述
    return LaunchDescription([
        port_arg,
        inverted_arg,
        robot_state_publisher_node,
        # joint_state_publisher_node,
        tf_static_node,
        ls01g_node,
        rviz_node,
    ])