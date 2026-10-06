#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np


class XArm6IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.d1 = 0.267
        self.a2 = np.sqrt(0.2845**2 + 0.0535**2)
        self.off2 = -np.arctan2(284.5, 53.5)
        self.off3 = -self.off2
        self.a3 = 0.0775
        self.d4 = 0.3425
        self.a5 = 0.076
        self.d6 = 0.097

        self.q_min = np.array([-2 * np.pi, -2.0595, -3.9270, -2 * np.pi, -1.6930, -2 * np.pi])
        self.q_max = np.array([ 2 * np.pi,  2.0944,  0.1920,  2 * np.pi,  np.pi,   2 * np.pi])
        self.q_seed = np.array([0.0, -0.3, -0.5, 0.0, 0.2, 0.0])
        self.current_q = self.q_seed.copy()

        self.alpha = 0.5
        self.tol = 1e-4
        self.max_iter = 200

        self.sub_target = self.create_subscription(
            Point, '/target', self.target_callback, 10
        )
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_current_joints)
        self.get_logger().info('Nodo IK (grupo01_xarm6_kinematics) listo. Esperando objetivos en /target...')

    def dh_std(self, theta, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0, sa,       ca,      d],
            [0.0, 0.0,      0.0,     1.0]
        ])

    def forward_chain(self, q):
        dh_params = [
            (q[0],             self.d1, 0.0,     -np.pi / 2),
            (q[1] + self.off2, 0.0,     self.a2,  0.0),
            (q[2] + self.off3, 0.0,     self.a3, -np.pi / 2),
            (q[3],             self.d4, 0.0,      np.pi / 2),
            (q[4],             0.0,     self.a5, -np.pi / 2),
            (q[5],             self.d6, 0.0,      0.0)
        ]
        T = np.eye(4)
        Ts = [T.copy()]
        for th, d, a, al in dh_params:
            T = T @ self.dh_std(th, d, a, al)
            Ts.append(T.copy())
        return T, Ts

    def jacobian_pos(self, q):
        _, Ts = self.forward_chain(q)
        o_n = Ts[-1][:3, 3]
        J_v = np.zeros((3, 6))
        for i in range(6):
            z_prev = Ts[i][:3, 2]
            o_prev = Ts[i][:3, 3]
            J_v[:, i] = np.cross(z_prev, o_n - o_prev)
        return J_v

    def target_callback(self, msg: Point):
        p_d = np.array([msg.x, msg.y, msg.z])
        q = self.q_seed.copy()
        converged = False

        for k in range(1, self.max_iter + 1):
            T, _ = self.forward_chain(q)
            p_curr = T[:3, 3]
            e = p_d - p_curr
            err_norm = np.linalg.norm(e)
            if err_norm < self.tol:
                converged = True
                break
            J = self.jacobian_pos(q)
            dq = self.alpha * (np.linalg.pinv(J) @ e)
            q = np.clip(q + dq, self.q_min, self.q_max)

        T_final, _ = self.forward_chain(q)
        p_final = T_final[:3, 3]
        final_err = np.linalg.norm(p_d - p_final)
        self.current_q = q.copy()

        estado = 'CONVERGIÓ' if converged else 'NO CONVERGIÓ'
        self.get_logger().info(
            f'\n================ RESULTADO IK xArm 6 (Grupo 01) ================\n'
            f'Objetivo recibido (xd, yd, zd) [m]: {np.round(p_d, 4).tolist()}\n'
            f'Configuración inicial q0 [rad]:     {np.round(self.q_seed, 4).tolist()}\n'
            f'Solución articular q* [rad]:        {np.round(q, 4).tolist()}\n'
            f'Iteraciones empleadas:              {k}\n'
            f'Posición alcanzada [m]:             {np.round(p_final, 6).tolist()}\n'
            f'Error final ||pd - pf||_2 [m]:      {final_err:.2e}\n'
            f'Estado:                             {estado}\n'
            f'================================================================'
        )

    def publish_current_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_q.tolist()
        self.pub_joints.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = XArm6IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
