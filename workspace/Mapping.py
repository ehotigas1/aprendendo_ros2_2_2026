#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

import tf_transformations
from bresenham import bresenham

import numpy as np
import math

class Mapeamento(Node):
    def __init__(self):
        super().__init__('mapeamento')
        self.get_logger().info('Inicializando o nó!')

        qos_profile_map = QoSProfile(depth=10)
        qos_profile_map.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile_map.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_profile_scan_odom = QoSProfile(depth=10)
        qos_profile_scan_odom.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_profile_map)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback,  qos_profile_scan_odom)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback,  qos_profile_scan_odom)
    
        ########## inicializando variáveis ##########
        self.dT = 1.0                                   
        self.pose = None
        self.map_res = 0.05
        self.map_real_size = 20 # mXm
        self.map_size = int(self.map_real_size / self.map_res)
        self.map = np.full((self.map_size, self.map_size), -1, dtype=np.int8)  
        self.origin = (-self.map_real_size/2, -self.map_real_size/2) #  centro do mapa
        #############################################

        self.timer = self.create_timer(self.dT,self.timer_callback)
        rclpy.spin(self)

    # Finalizando nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó!')
        self.destroy_node()

    def odom_callback(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        _, _, yaw = tf_transformations.euler_from_quaternion([x, y, z, w])

        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        self.pose = (pos_x, pos_y, yaw)

    def laser_callback(self, msg):
        self.laser = msg

    # Executando
    def timer_callback(self):
        if self.pose is None or self.laser == None:
            return       
        
        self.mapping_algorithm()
        self.publicar_mapa()

    # algorithm
    def mapping_algorithm(self):

        for i in range(len(self.laser.ranges)):

            r = self.laser.ranges[i]

            if math.isinf(r) or math.isnan(r):
                continue

            angle = self.laser.angle_min + i * self.laser.angle_increment

            xr = r * math.cos(angle)
            yr = r * math.sin(angle)

            x = self.pose[0] + xr * math.cos(self.pose[2]) - yr * math.sin(self.pose[2])
            y = self.pose[1] + xr * math.sin(self.pose[2]) + yr * math.cos(self.pose[2])

            x0, y0 = self.world_to_map(self.pose[0], self.pose[1])
            x1, y1 = self.world_to_map(x, y)

            cells = list(bresenham(x0, y0, x1, y1))

            for cx, cy in cells[:-1]:
                if 0 <= cx < self.map_size and 0 <= cy < self.map_size:
                    self.map[cy, cx] = 0

            cx, cy = cells[-1]
            if 0 <= cx < self.map_size and 0 <= cy < self.map_size:
                self.map[cy, cx] = 100
        return self.map
    
    def world_to_map(self, x, y):

        mx = int((x - self.origin[0]) / self.map_res)
        my = int((y - self.origin[1]) / self.map_res)

        return mx, my


    def publicar_mapa(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.map_res
        msg.info.width = self.map_size
        msg.info.height = self.map_size
        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]
        msg.data = self.map.flatten().tolist()
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args) # Inicializando ROS
    node = Mapeamento()   # Inicializando nó
    del node              # Finalizando nó
    rclpy.shutdown()      # Finalizando ROS

if __name__ == '__main__':
    main()
