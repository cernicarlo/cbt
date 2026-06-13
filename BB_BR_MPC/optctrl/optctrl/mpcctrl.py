#!/usr/bin/env python3
"""
MPC AUV ros2 stonefish
version : 0.1
author : sumer
"""
import rclpy
import torch
import numpy as np
import pickle
from rclpy.node import Node
from ctrl.mpc import MPC
from stonefish_ros2.msg import INS
from std_msgs.msg import Float64MultiArray
from fossens.fossen19 import AUVFossen
from ament_index_python.packages import get_package_share_directory
from optctrl.utils import parse_param, generate_random_ref, quaternion_to_euler, euler_to_quaternion
from nav_msgs.msg import Odometry

class LowPassFilter:
    def __init__(self, cutoff_frequency: float, sampling_rate: float):
        dt = 1 / sampling_rate
        rc = 1 / (2 * 3.1416 * cutoff_frequency)
        self.alpha = dt / (rc + dt)
        self.last_value = None

    def filter(self, input_value, debug = False):
        if self.last_value is None:
            self.last_value = input_value
            return input_value
        
        filtered_value = self.alpha * input_value + (1 - self.alpha) * self.last_value
        self.last_value = filtered_value
        if debug:
            print(f"filter - diff: {filtered_value - input_value} \nbefore: {input_value} \nafter: {filtered_value}")
        return filtered_value
    
    def filter2(self, input_value, debug = False):
        filtered_value = self.filter(input_value, debug)
        filtered_value = self.filter(filtered_value, debug)


class MPCNode(Node):
    
    def __init__(self, debug=False):
        super().__init__("MPCNode")

        self.debug = debug

        # self.cmd_filter = LowPassFilter(cutoff_frequency=10, sampling_rate=50)
        self.lp_x = LowPassFilter(cutoff_frequency=1, sampling_rate=500)
        self.lp_v = LowPassFilter(cutoff_frequency=1, sampling_rate=500)

        self.model = AUVFossen(config=parse_param(f"{get_package_share_directory('optctrl')}/config/config.yaml"))
        self.mpc = MPC(model=self.model)
        
        self.mpc.set_reference(ref=torch.tensor([0,0,0,0,0,0,1,
                                                 0,0,0,0,0,0], dtype=torch.float64).numpy())
        
        self.odom = Odometry()
        
        with open(f"{get_package_share_directory('optctrl')}/config/force_to_pwm_16V.pkl", 'rb') as f:
            self.force2pwm_interpolated = pickle.load(f)
        
        
        self.matrix_omega_to_force = torch.tensor([[0.707107, -0.707107, 0.707107, -0.707107, 0, 0, 0, 0],
                                              [0.707107, 0.707107, -0.707107, -0.707107, 0, 0, 0, 0],
                                              [0, 0, 0, 0, 1, 1, 1, 1],
                                              [-0.0215732, -0.0213654, 0.0211532, 0.0216418, 0.123596, 0.123596, -0.116404, -0.116404],
                                              [0.0215732, -0.0213654, 0.0211532, -0.0216418, 0.217818, -0.218182, 0.217818, -0.218182],
                                              [-0.169589, 0.169061, 0.176, -0.174921, 0, 0, 0, 0]], 
                                             dtype=torch.float64)
        
        self.matrix_force_to_omega = torch.linalg.pinv(self.matrix_omega_to_force)
        
        self._init_vars()
        self._init_subscribers()
        self._init_publishers()
        
    def _init_vars(self):
        
        self.x_fb = torch.zeros(size=(7,), dtype=torch.float64)
        self.v_fb = torch.zeros(size=(6,), dtype=torch.float64)

        
        self.x_fb[6] = 1.0
        
        self.reset_cmd = 0
        
    def _init_subscribers(self):

        # self._IMU_sub = self.create_subscription(
        #     msg_type=INS,
        #     topic='/bluerov/navigator/imu',
        #     callback=self._IMU_sub_callback,
        #     qos_profile=10
        # )
        
        self._odometry_sub = self.create_subscription(
            msg_type=Odometry,
            topic='/bluerov/navigator/odometry',
            callback=self._odometry_sub_callback,
            qos_profile=10
        )
        
        self._reference_sub = self.create_subscription(
            msg_type=Float64MultiArray,
            topic='/optreferences',
            callback=self._reference_sub_callback,
            qos_profile=10
        )
        
    def _init_publishers(self):
        
        self._cmd_pub = self.create_publisher(
            msg_type=Float64MultiArray,
            topic='/bluerov/controller/thruster_setpoints_sim',
            qos_profile=10
        )
        
        self.timer = self.create_timer(
            timer_period_sec=0.02,
            callback=self._send_cmd,
        ) # 50 Hz ctrl
    
    def _send_cmd(self):
        x0 = torch.cat((self.x_fb, self.v_fb), dim=0)
        u = self.mpc.solve_problem(x0=x0.numpy())
        msg = Float64MultiArray()
        
        cmds = self._force2motorcmd(u=u)
        
        msg.data = [float(cmds[0]), float(cmds[1]), float(cmds[2]), float(cmds[3]), float(cmds[4]), float(cmds[5]), float(cmds[6]), float(cmds[7])]
        
        if self.reset_cmd:
            msg.data = [float(0), float(0), float(0), float(0), float(0), float(0), float(0), float(0)]
        
        self._cmd_pub.publish(msg=msg)
    
    def _update_odom(self):
        self.x_fb = torch.tensor([self.odom.pose.pose.position.x,
                                  self.odom.pose.pose.position.y,
                                  self.odom.pose.pose.position.z,
                                  self.odom.pose.pose.orientation.x,
                                  self.odom.pose.pose.orientation.y,
                                  self.odom.pose.pose.orientation.z,
                                  self.odom.pose.pose.orientation.w], dtype=torch.float64)
        self.x_fb = self.lp_x.filter(self.x_fb)
        
        
        
        self.v_fb = torch.tensor([self.odom.twist.twist.linear.x,
                                  self.odom.twist.twist.linear.y,
                                  self.odom.twist.twist.linear.z,
                                  self.odom.twist.twist.angular.x,
                                  self.odom.twist.twist.angular.y,
                                  self.odom.twist.twist.angular.z], dtype=torch.float64)
        self.v_fb = self.lp_v.filter(self.v_fb)

        
    def _odometry_sub_callback(self, msg: Odometry):
        ''' Take only linear positions & velocities'''
        # self.odom.pose.pose.position = msg.pose.pose.position
        # self.odom.twist.twist.linear = msg.twist.twist.linear

        self.odom = msg
        self._update_odom()

        if self.debug:
            print(f"odom - x: {self.x_fb}")
            print(f"odom - v: {self.v_fb}")
        
    def _IMU_sub_callback(self, msg: INS):
        ''' Take only angular positions and velocities'''
        quats = euler_to_quaternion(torch.tensor([msg.pose.pitch, msg.pose.roll, msg.pose.yaw], dtype=torch.float64)).numpy()
        # TODO: check stonefish axes definitions for INS.
        self.odom.pose.pose.orientation.x = quats[0]
        self.odom.pose.pose.orientation.y = quats[1]
        self.odom.pose.pose.orientation.z = quats[2]
        self.odom.pose.pose.orientation.w = quats[3]
        self.odom.twist.twist.angular.x = msg.rpy_rate.x
        self.odom.twist.twist.angular.y = msg.rpy_rate.y
        self.odom.twist.twist.angular.z = msg.rpy_rate.z
        self._update_odom()
        
        if self.debug:
            print(f"imu - x: {self.x_fb} \n")
            print(f"imu - v: {self.v_fb} \n")
            
    def _force2motorcmd(self, u: np.ndarray) -> torch.Tensor:
        forces = torch.tensor(u, dtype=torch.float64)
        motorcmds = self.pwm2normalized(self.force2pwm_interpolated(
            torch.matmul(self.matrix_force_to_omega, forces).numpy()))
        return motorcmds
    
    def pwm2normalized(self, pwm:float, pwm_min: float = 1100, pwm_max: float = 1900):
        return 2 * (pwm - pwm_min) / (pwm_max - pwm_min) - 1
    
    def _reference_sub_callback(self, msg: Float64MultiArray):
        self.mpc.set_reference(
            ref=torch.tensor(
                [msg.data[0],
                 msg.data[1],
                 msg.data[2],
                 msg.data[3],
                 msg.data[4],
                 msg.data[5],
                 msg.data[6],
                 msg.data[7],
                 msg.data[8],
                 msg.data[9],
                 msg.data[10],
                 msg.data[11],
                 msg.data[12],
                 ], dtype=torch.float64).numpy())
        
        self.reset_cmd = msg.data[13]
    
def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()