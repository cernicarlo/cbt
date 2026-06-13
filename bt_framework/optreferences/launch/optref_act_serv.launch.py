from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Parameters to pass to the node
    params = {
        "max_lin_vel": 1.0,
        "max_lin_acc": 0.5,
        "max_z_vel": 2.0,
        "max_z_acc": 1.0,
        "max_ang_vel": 0.15,
        "max_ang_acc": 0.1,
        "tol_pose_odom": 0.1,  # m
        "tol_orient_odom": 0.1,  # rad
        "fact_tol_pose": 10.0,  # no unit - always > 1.0 (tol_goal_ = tol_ * fact_tol_)
        "fact_tol_z": 25.0,  # no unit - always > 1.0
        "fact_tol_roll": 20.0,  # no unit - always > 1.0
        "fact_tol_pitch": 20.0,  # no unit - always > 1.0
        "fact_tol_yaw": 2.0,  # no unit - always > 1.0
        "z_offset": 0.0,
        "is_sequence_control": True,
    }

    return LaunchDescription([
        Node(
            package='optreferences',
            executable='optreferences_action_server_node',
            name='optreferences_action_server',
            output='screen',
            parameters=[params]
        )
    ])
