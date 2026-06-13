from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Parameters to pass to the node
    params = {
        "kp_lin": 0.1, # linear gain
        "kd_lin": 0.01,
        "kp_ang": 0.3, # angular gain
        "kd_ang": 0.00,
        "tol_pose": 0.5, # tolerance for odometry position
        "tol_orient": 0.1, # tolerance for odometry orientation
        "max_thrust": 0.5, # maximum thrust
        "min_thrust": 0.15, # minimum thrust
    }

    return LaunchDescription([
        Node(
            package='optreferences',
            executable='bb_position_hold_action_server_node',
            name='bb_position_hold_action_server',
            output='screen',
            parameters=[params]
        )
    ])
