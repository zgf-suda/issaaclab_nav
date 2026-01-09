#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TwistStamped, TransformStamped
from tf_transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion, quaternion_matrix,euler_from_matrix
import numpy as np
from std_msgs.msg import Float32MultiArray

class VelocityEstimator(Node):
    def __init__(self):
        super().__init__('velocity_estimator')

        # TF buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher
        #self.vel_pub = self.create_publisher(Float32MultiArray, '/base_link_velocity', 10)
        self.commond_pub = self.create_publisher(Float32MultiArray, '/commond_base_link', 10)
        # Timer
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz

        self.last_transform = None
        self.last_time = None

        #维护一个odom下的固定点,发布在base_link下的坐标值
        self.world_point_translation = [2.0, 0.0, 0.0]   # world frame 坐标
        self.world_point_rotation = [0,0,0,1]

    def timer_callback(self):
        try:
            # 查询 TF: odom -> base_link，表示 把 base_link 坐标系下的点，转换到 odom 坐标系下。
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())

            now = transform.header.stamp
            if self.last_transform is not None:
                dt = (now.sec + now.nanosec * 1e-9) - (self.last_time.sec + self.last_time.nanosec * 1e-9)
                if dt <= 0.0:
                    return

                # --- 平移差分 (odom系下的速度) ---
                dx = transform.transform.translation.x - self.last_transform.transform.translation.x
                dy = transform.transform.translation.y - self.last_transform.transform.translation.y
                vx_odom = dx / dt
                vy_odom = dy / dt

                # --- 姿态差分 -> 角速度 (绕Z轴) ---
                q1 = [
                    self.last_transform.transform.rotation.x,
                    self.last_transform.transform.rotation.y,
                    self.last_transform.transform.rotation.z,
                    self.last_transform.transform.rotation.w,
                ]
                q2 = [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]
                dq = quaternion_multiply(q2, quaternion_inverse(q1))
                _, _, dyaw = euler_from_quaternion(dq)
                wz = dyaw / dt

                # --- 坐标变换 (odom -> base_link) ---
                q_base = [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]
                rot_mat = quaternion_matrix(q_base)[:3, :3]  #base_link到odom

                vel_world = np.array([vx_odom, vy_odom, 0.0])
                vel_body = rot_mat.T @ vel_world  # odom转到 base_link 系

                vx_body = vel_body[0]
                vy_body = vel_body[1]

                # --- 发布 Twist ---
                # twist = TwistStamped()
                # twist.header.stamp = now
                # twist.header.frame_id = "base_link"
                # twist.twist.linear.x = vx_body
                # twist.twist.linear.y = vy_body
                # twist.twist.angular.z = wz
                # self.vel_pub.publish(twist)

                self.get_logger().info(
                    f"vx={vx_body:.3f}, vy={vy_body:.3f}, wz={wz:.3f} (base_link系)"
                )

                #计算base_link下的目标点姿态
                _, _, yaw_base = euler_from_matrix(rot_mat.T, axes='sxyz') #弧度

                x_base,y_base,z_base = rot_mat.T @ (np.array(self.world_point_translation) - np.array([transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z]))

                command_base = np.array([vx_body,vy_body,wz,x_base,y_base,z_base,yaw_base])
                msg = Float32MultiArray()
                msg.data = command_base.flatten().tolist()  # 转成列表
                self.commond_pub.publish(msg)
                self.get_logger().info(
                    f"x_base={x_base:.3f}, y_base={y_base:.3f}, z_base={z_base:.3f},yaw_base={yaw_base:.3f} (base_link系)"
                )
            # 更新历史值
            self.last_transform = transform
            self.last_time = now

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
