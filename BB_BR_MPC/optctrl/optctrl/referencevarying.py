#!/usr/bin/env python3

import torch
import tyro
import rclpy
from rclpy.node import Node

import threading
from pynput import keyboard

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
        self.running = True
        self.references = args
        
        self._init_pub()
        
        self.keyboard_thread = threading.Thread(target=self.listen_keyboard, daemon=True)
        self.keyboard_thread.start()
        self.keys_pressed = set()
        
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
                    float(self.references.reset),
                    float(self.references.phi),
                    float(self.references.theta),
                    float(self.references.psi)
                    ]
        
        self._reference_publisher.publish(msg=msg)

    def _init_input_handler(self, ):
        input_thread = threading.Thread(target=self._get_input, daemon=True)
        
    def listen_keyboard(self):
        def on_press(key):
            self.keys_pressed.add(key)
            if  keyboard.Key.up in self.keys_pressed and keyboard.Key.shift not in self.keys_pressed and keyboard.Key.alt not in self.keys_pressed:
                self.references.x += 0.1
                self.get_logger().info(f"UP arrow pressed, x increased: {self.references.x}")
            elif keyboard.Key.down in self.keys_pressed and keyboard.Key.shift not in self.keys_pressed and keyboard.Key.alt not in self.keys_pressed:
                self.references.x -= 0.1
                self.get_logger().info(f"DOWN arrow pressed, x decreased: {self.references.x}")
            elif keyboard.Key.left in self.keys_pressed and keyboard.Key.shift not in self.keys_pressed and keyboard.Key.alt not in self.keys_pressed:
                self.references.y += 0.1
                self.get_logger().info(f"LEFT arrow pressed, y increased: {self.references.y}")
            elif keyboard.Key.right in self.keys_pressed and keyboard.Key.shift not in self.keys_pressed and keyboard.Key.alt not in self.keys_pressed: 
                self.references.y -= 0.1
                self.get_logger().info(f"RIGHT arrow pressed, y decreased: {self.references.y}")
            elif keyboard.Key.up in self.keys_pressed and keyboard.Key.shift in self.keys_pressed and keyboard.Key.alt not in self.keys_pressed:
                self.references.z += 0.1
                self.get_logger().info(f"SHIFT+UP arrow pressed, z increased: {self.references.z}")
            elif keyboard.Key.down in self.keys_pressed and keyboard.Key.shift in self.keys_pressed and keyboard.Key.alt not in self.keys_pressed:
                self.references.z -= 0.1
                self.get_logger().info(f"SHIFT+DOWN arrow pressed, z decreased: {self.references.z}")
            elif keyboard.Key.left in self.keys_pressed and keyboard.Key.shift in self.keys_pressed and keyboard.Key.alt not in self.keys_pressed:
                self.references.psi += 0.05
                self.get_logger().info(f"SHIFT+LEFT arrow pressed, YAW decreased: {self.references.psi}")
            elif keyboard.Key.right in self.keys_pressed and keyboard.Key.shift in self.keys_pressed and keyboard.Key.alt not in self.keys_pressed:
                self.references.psi -= 0.05
                self.get_logger().info(f"SHIFT+RIGHT arrow pressed, YAW decreased: {self.references.psi}")
            elif keyboard.Key.left in self.keys_pressed and keyboard.Key.alt in self.keys_pressed and keyboard.Key.shift not in self.keys_pressed:
                self.references.phi += 0.05
                self.get_logger().info(f"ALT+LEFT arrow pressed, ROLL increased: {self.references.phi}")
            elif keyboard.Key.right in self.keys_pressed and keyboard.Key.alt in self.keys_pressed and keyboard.Key.shift not in self.keys_pressed:
                self.references.phi -= 0.05
                self.get_logger().info(f"ALT+RIGHT pressed, ROLL decreased: {self.references.phi}")
            elif keyboard.Key.up in self.keys_pressed and keyboard.Key.alt in self.keys_pressed and keyboard.Key.shift not in self.keys_pressed:
                self.references.theta += 0.05
                self.get_logger().info(f"ALT+UP pressed, PITCH increased: {self.references.theta}")
            elif keyboard.Key.down in self.keys_pressed and keyboard.Key.alt in self.keys_pressed and keyboard.Key.shift not in self.keys_pressed:
                self.references.theta -= 0.05
                self.get_logger().info(f"ALT+DOWN pressed, PITCH decreased: {self.references.theta}")
                
        def on_release(key):
            try:
                self.keys_pressed.remove(key)
            except KeyError:
                pass
            
            if key == keyboard.Key.esc:
                return False
        
        
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()  # Keeps listening in the background
                
    
    def stop(self, ):
        self.running = False
    
def main(args=None):
    parsedargs = tyro.cli(Args)
    rclpy.init(args=args)
    node = ReferenceNode(args=parsedargs, debug=True)
    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()