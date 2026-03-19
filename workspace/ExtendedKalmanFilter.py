#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion

from tf_transformations import quaternion_from_euler

import numpy as np  

class EKF(Node):

    # Inicializando nó
    def __init__(self):
        super().__init__('EKF')
        self.get_logger().info('Inicializando o nó!')

        self.create_subscription(Odometry, '/odom', self.subscriber_callback, 10)
        self.publisher_pose = self.create_publisher(Pose, '/pose', 10)

        ########## inicializando variáveis ##########
        self.dT = 1.0
        
        self.x = np.array([[0.0],[0.0],[0.0]])                # vetor do estado 
        self.u = None                         # vetor de controle 
        self.P = np.diag([0.1**2,0.1**2,np.radians(10)**2])                   # matriz de covariância da estimativa inicial
        self.Q = np.diag([0.1**2,0.1**2,np.radians(10)**2])
        self.z = None
        self.R = None
        #############################################

        self.timer = self.create_timer(self.dT,self.timer_callback)
        rclpy.spin(self)

    # Finalizando nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó!')
        self.destroy_node()

    # Odom callback
    def subscriber_callback(self, msg):
        self.z = np.array([[msg.pose.pose.position.x],[msg.pose.pose.position.y]])
        self.R = np.diag([msg.pose.covariance[0],msg.pose.covariance[7]])
        self.u = np.array([[msg.twist.twist.linear.x],[msg.twist.twist.angular.z]])

    # Executando
    def timer_callback(self):        
        self.x, self.P = self.ekf_algorithm(self.x,self.P,self.u,self.z,self.Q,self.R)

        msg = Pose()
        msg.position = Point(x=self.x[0].item(), y=self.x[1].item(), z=0.0) 
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.x[2].item())
        msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.publisher_pose.publish(msg)

    # Atualização
    def ekf_algorithm(self, x, P, u, z, Q, R):
        if u is None or z is None or R is None:
            return x.copy(), P.copy()

        # Jacobiana da medição
        H = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ])

        # Jacobiana do movimento
        F = np.array([
            [1.0, 0.0, -u[0, 0] * np.sin(x[2, 0]) * self.dT],
            [0.0, 1.0,  u[0, 0] * np.cos(x[2, 0]) * self.dT],
            [0.0, 0.0, 1.0]
        ])

        # 2: x_bar_t <- f(x_t-1, u_t)
        x_bar = np.array([
            [x[0, 0] + u[0, 0] * np.cos(x[2, 0]) * self.dT],
            [x[1, 0] + u[0, 0] * np.sin(x[2, 0]) * self.dT],
            [x[2, 0] + u[1, 0] * self.dT]
        ])

        # 3: P_bar_t <- F_t * P_t-1 * F_t^T + Q
        P_bar = F @ P @ F.T + Q

        # 4: K_t <- P_bar_t * H_t^T * (H_t * P_bar_t * H_t^T + R)^-1
        Kt = P_bar @ H.T @ np.linalg.inv(H @ P_bar @ H.T + R)

        # h(x_bar_t)
        h = H @ x_bar

        # 5: x_t <- x_bar_t + K_t * (z_t - h(x_bar_t))
        x = x_bar + Kt @ (z - h)

        # 6: P_t <- (I - K_t * H_t) * P_bar_t
        P = (np.eye(3) - Kt @ H) @ P_bar

        print("Estado predito:\n", x_bar)
        print("Estado corrigido:\n", x)

        return x.copy(), P.copy()

def main(args=None):
    rclpy.init(args=args) # Inicializando ROS
    node = EKF()          # Inicializando nó
    del node              # Finalizando nó
    rclpy.shutdown()      # Finalizando ROS

if __name__ == '__main__':
    main()
