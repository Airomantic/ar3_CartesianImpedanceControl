from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明可配置参数
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.2.160',
            description='机器人控制器IP地址'
        ),
        DeclareLaunchArgument(
            'local_ip',
            default_value='192.168.2.100',
            description='本地计算机IP地址'
        ),
        DeclareLaunchArgument(
            'robot_class',
            default_value='collaborative',
            description='collaborative 或 standard（XB7s/XB7h/NB25s/NB25h/NB12s/NB12h/EB4/NB4/XB4s/XB4h/NB220s 工业机用 standard）'
        ),
        DeclareLaunchArgument(
            'xMate_type',
            default_value='',
            description='机型后缀，如 XB7s、XB7h、NB25s、NB25h、NB12s、NB12h、EB4、NB4、XB4s、XB4h、NB220s（也可触发 StandardRobot）'
        ),
        
        # 启动rokae_driver节点
        Node(
            package='rokae_hardware',
            executable='rokae_driver',
            name='rokae_driver',
            output='screen',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'local_ip': LaunchConfiguration('local_ip'),
                'robot_class': LaunchConfiguration('robot_class'),
                'xMate_type': LaunchConfiguration('xMate_type'),
            }],
            # 可以设置remapping等
            remappings=[
                # 如果需要重映射话题，可以在这里添加
                # ('/rokae_driver/joint_states', '/joint_states'),
            ],
            # 可以设置节点命名空间
            # namespace='robot1',
        )
    ])