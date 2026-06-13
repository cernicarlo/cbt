import yaml
import torch
import time
import numpy as np
from functools import wraps
from typing import Callable, Any
import cvxpy as cp

def quaternion_error(q1: torch.Tensor, q2: torch.Tensor) -> torch.Tensor:
    """
    q1 desired, q2 actual
    """
    # q2_conj = torch.tensor([-q2[0], -q2[1], -q2[2], q2[3]], dtype=torch.float64)

    x1, y1, z1, w1 = q1[0], q1[1], q1[2], q1[3]
    x2, y2, z2, w2 = q2[0], q2[1], q2[2], q2[3]
    
    x_err = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1
    y_err = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1
    z_err = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1
    w_err = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1
    
    q_err = cp.hstack([x_err, y_err, z_err, w_err])
    # q_err = torch.tensor([x_err, y_err, z_err, w_err])  # Construct in xyzw order
    # if q_err[3] < 0: # Check w component (index 3)
    #     q_err = -q_err
    return q_err

def chronometer(func: Callable[..., Any]) -> Callable[..., Any]:
    @wraps(func)
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        start_time = time.time() 
        result = func(*args, **kwargs) 
        end_time = time.time()  
        runtime = end_time - start_time 
        print(f"Function '{func.__name__}' executed in {runtime:.6f} seconds")
        return result  

    return wrapper

def parse_param(file):
    with open(file) as file:
        conf = yaml.load(file, Loader=yaml.FullLoader)
    return conf

def quaternion_to_euler(quat: torch.Tensor,  
                        degrees: bool = False
                        ) -> torch.Tensor:
    x, y, z, w = quat[0], quat[1], quat[2], quat[3]
    # roll 
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = torch.atan2(sinr_cosp, cosr_cosp)
    # pitch 
    sinp = 2 * (w * y - z * x)
    pitch = torch.asin(sinp)
    # yaw 
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = torch.atan2(siny_cosp, cosy_cosp)
    euler_angles = torch.stack((roll, pitch, yaw))
    if degrees:
        euler_angles = euler_angles * 180.0 / torch.pi
    return euler_angles

def euler_to_quaternion(euler: torch.Tensor, degrees: bool = False) -> torch.Tensor:
    if degrees:
        euler = euler * 180 / torch.pi
    roll, pitch, yaw = euler[0], euler[1], euler[2]
    cy = torch.cos(yaw * 0.5)
    sy = torch.sin(yaw * 0.5)
    cp = torch.cos(pitch * 0.5)
    sp = torch.sin(pitch * 0.5)
    cr = torch.cos(roll * 0.5)
    sr = torch.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    quaternion = torch.stack((x, y, z, w))
    return quaternion

def quaternion_to_rotation_matrix(pose: torch.Tensor) -> torch.Tensor:
    x, y, z, w = pose[3:7]
    R = torch.tensor([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),   1 - 2 * (x**2 + y**2)]
    ], dtype=torch.float64)
    return R

def body2inertial(pose: torch.Tensor) -> torch.Tensor:
    quat = pose[3:7]
    x, y, z, w = quat[0], quat[1], quat[2], quat[3]
    r1 = torch.tensor([
        1 - 2 * (y**2 + z**2),
        2 * (x * y - z * w),
        2 * (x * z + y * w)], dtype=torch.float64)
    r2 = torch.tensor([
        2 * (x * y + z * w),
        1 - 2 * (x**2 + z**2),
        2 * (y * z - x * w)], dtype=torch.float64)
    r3 = torch.tensor([
        2 * (x * z - y * w),
        2 * (y * z + x * w),
        1 - 2 * (x**2 + y**2)], dtype=torch.float64)
    return torch.stack([r1, r2, r3], dim=0)

def generate_random_ref():
    ref_eul = (torch.randn(3, dtype=torch.float64) / 5.0) * 1
    ref_quat = euler_to_quaternion(euler=ref_eul)
    ref_speed = torch.tensor([0,0,0,0,0,0], dtype=torch.float64)
    ref = torch.cat((torch.randn(3, dtype=torch.float64) * 3, ref_quat, ref_speed)) 
    return ref