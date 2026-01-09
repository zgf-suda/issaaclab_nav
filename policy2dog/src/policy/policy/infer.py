#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import torch
import torch.nn as nn
import numpy as np
from std_msgs.msg import Float32MultiArray
from pathlib import Path

class PolicyInferNode(Node):
    def __init__(self):
        super().__init__('policy_infer_node')

        # 订阅 base_link_velocity
        # self.subscription = self.create_subscription(
        #     TwistStamped,
        #     '/base_link_velocity',
        #     self.listener_callback,
        #     10
        # )
        self.subscription_command = self.create_subscription(
            Float32MultiArray,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.policy_pub = self.create_publisher(Twist, '/pp_twist_cmd', 10)
        # 加载 PyTorch 模型
        self.get_logger().info("Loading policy.pt ...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        policy_path = str(Path(__file__).resolve().parent)
        self.model = torch.jit.load(policy_path + "/policy.pt", map_location=self.device)
        self.model.eval()
        self.get_logger().info(f"Model loaded on {self.device}")

    def listener_callback(self, msg: Float32MultiArray):
        command_base = msg.data
        vx_base = command_base[0]
        vy_base = command_base[1]
        wz_base = command_base[2]
        x_base = command_base[3]
        y_base = command_base[4]
        z_base = command_base[5]
        yaw_base = command_base[6]

        # 转成 Tensor，输入模型
        state = torch.tensor([vx_base, vy_base, wz_base,x_base,y_base,z_base,yaw_base], dtype=torch.float32, device=self.device).unsqueeze(0)

        with torch.no_grad():
            output = self.model(state)

        # 输出 numpy
        result = output.cpu().numpy()
        

        # --- 发布 Twist ---
        twist = Twist()
        # twist.header.stamp = self.get_clock().now()
        # twist.header.frame_id = "base_link"
        twist.twist.linear.x = result[0][0].item()
        twist.twist.linear.y = result[0][1].item()
        twist.twist.angular.z = result[0][2].item()
        self.policy_pub.publish(twist)


        # msg = Float32MultiArray()
        # msg.data = result.flatten().tolist()  # 转成列表
        # self.policy_pub.publish(msg)

        self.get_logger().info(
            f"Model Output: {result}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PolicyInferNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
