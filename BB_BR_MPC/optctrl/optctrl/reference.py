#!/usr/bin/env python3

import torch
import tyro
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from dataclasses import dataclass

from optctrl.utils import euler_to_quaternion

@dataclass
class Args:
    """Reference publisher generator. \n""" \
    """- v0.1\n""" \
    """- author: sumer"""
    x: float = 0.0
    """x ref"""
    y: float = 0.0
    """y ref"""
    z: float = 0.0
    """z ref"""
    phi: float = 0.0
    """phi ref"""
    theta: float = 0.0
    """theta ref"""
    psi: float = 0.0
    """psi ref"""
    u: float = 0.0
    """u ref"""
    v: float = 0.0
    """v ref"""
    w: float = 0.0
    """w ref"""
    p: float = 0.0
    """p ref"""
    q: float = 0.0
    """q ref"""
    r: float = 0.0
    """r ref"""
    reset: bool = False
    """reset cmd"""

class ReferenceNode(Node):
    
    def __init__(self, args: Args, debug: bool=False):
        super().__init__("ReferenceNode")
        self.debug = debug
        
        self.references = args
        
        self._init_pub()
        
    def _init_pub(self, ):
        
        self._reference_publisher = self.create_publisher(
            msg_type=Float64MultiArray,
            topic='/optreferences',
            qos_profile=10
        )
        
        self.timer = self.create_timer(
            timer_period_sec=0.1,
            callback=self._send_reference,
        )
        
    def _send_reference(self, ):
        
        msg = Float64MultiArray()
        
        quats = euler_to_quaternion(
            euler=torch.tensor(
                [self.references.phi, self.references.theta, self.references.psi], 
                dtype=torch.float64)
            ).numpy()
        
        msg.data = [float(self.references.x),
                    float(self.references.y),
                    float(self.references.z),
                    float(quats[0]),
                    float(quats[1]),
                    float(quats[2]),
                    float(quats[3]),
                    float(self.references.u),
                    float(self.references.v),
                    float(self.references.w),
                    float(self.references.p),
                    float(self.references.q),
                    float(self.references.r),
                    float(self.references.reset)]
        
        self._reference_publisher.publish(msg=msg)
        
def main(args=None):
    parsedargs = tyro.cli(Args)
    rclpy.init(args=args)
    node = ReferenceNode(args=parsedargs, debug=True)
    rclpy.spin(node=node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()