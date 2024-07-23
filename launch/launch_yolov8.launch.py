from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    yolov8 = Node(
        package='arceus_recognition',
        executable='yolov8_ros2_pt.py',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    inference_position = Node(
        package = 'arceus_recognition',
        executable='inference_position.py',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    person_goal_tf = Node(
        package = 'arceus_recognition',
        executable='person_goal_tf.py',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                description='Flag to enable use_sim_time'),
        yolov8,
        inference_position,
        person_goal_tf
    ])