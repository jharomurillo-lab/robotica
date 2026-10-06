#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np


class XArm6FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.d1 = 0.267
        self.a2 = np.sqrt(0.2845**2 + 0.0535**2)
        self.off2 = -np.arctan2(284.5, 53.5)
        self.off3 = -self.off2
        self.a3 = 0.0775
        self.d4 = 0.3425
        self.a5 = 0.076
        self.d6 = 0.097

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.listener_callback, 10
        )
        self.get_logger().info('Nodo FK (grupo01_xarm6_kinematics) iniciado. Escuchando /joint_states...')

    def dh_std(self, theta, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0, sa,       ca,      d],
            [0.0, 0.0,      0.0,     1.0]
        ])

    def compute_fk(self, q):
        dh_params = [
            (q[0],             self.d1, 0.0,     -np.pi / 2),
            (q[1] + self.off2, 0.0,     self.a2,  0.0),
            (q[2] + self.off3, 0.0,     self.a3, -np.pi / 2),
            (q[3],             self.d4, 0.0,      np.pi / 2),
            (q[4],             0.0,     self.a5, -np.pi / 2),
            (q[5],             self.d6, 0.0,      0.0)
        ]
        T = np.eye(4)
        for th, d, a, al in dh_params:
            T = T @ self.dh_std(th, d, a, al)
        return T

    def rot_to_quat(self, R):
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return np.array([qx, qy, qz, qw])

    def listener_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if not all(j in name_to_pos for j in self.joint_names):
            return
        q = np.array([name_to_pos[j] for j in self.joint_names])
        T = self.compute_fk(q)
        p = T[:3, 3]
        R = T[:3, :3]
        quat = self.rot_to_quat(R)

        self.get_logger().info(
            f'\n--- FK UFACTORY xArm 6 (Grupo 01) ---\n'
            f'q [rad]: {np.round(q, 4).tolist()}\n'
            f'Posición (x, y, z) [m]: [{p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f}]\n'
            f'Cuaternión (qx, qy, qz, qw): [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = XArm6FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
