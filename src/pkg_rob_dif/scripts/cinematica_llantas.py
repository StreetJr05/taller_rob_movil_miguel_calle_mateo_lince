#!/usr/bin/env python3

import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


# Funciones de apoyo para los angulos
def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# Guarda el error RMS y el maximo de cada comparacion
class ErrorStats:

    def __init__(self):
        self.n = 0
        self.sum_sq_pos = 0.0
        self.sum_sq_yaw = 0.0
        self.max_pos = 0.0
        self.max_yaw = 0.0

    def add(self, e_pos, e_yaw):
        self.n += 1
        self.sum_sq_pos += e_pos * e_pos
        self.sum_sq_yaw += e_yaw * e_yaw
        self.max_pos = max(self.max_pos, abs(e_pos))
        self.max_yaw = max(self.max_yaw, abs(e_yaw))

    def summary(self):
        if self.n == 0:
            return 'no samples yet'
        rms_pos = math.sqrt(self.sum_sq_pos / self.n)
        rms_yaw = math.sqrt(self.sum_sq_yaw / self.n)
        return (f'pos RMS {rms_pos * 1000:.1f} mm, max {self.max_pos * 1000:.1f} mm | '
                f'yaw RMS {math.degrees(rms_yaw):.2f} deg, max {math.degrees(self.max_yaw):.2f} deg')


class CinematicaLlantas(Node):

    def __init__(self):
        super().__init__('cinematica_llantas')

        # Parametros del robot que llegan desde el launch
        self.r = self.declare_parameter('wheel_radius', 0.05).value
        self.b = self.declare_parameter('wheel_separation', 0.36).value
        self.right_joint = self.declare_parameter('right_joint', 'right_wheel_joint').value
        self.left_joint = self.declare_parameter('left_joint', 'left_wheel_joint').value
        csv_path = self.declare_parameter('csv_path', '').value

        # Pose que vamos integrando
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.last_stamp = None

        self.gt_origin = None
        self.gt = None

        self.stats_odom = ErrorStats()
        self.stats_gt_own = ErrorStats()
        self.stats_gt_plugin = ErrorStats()

        self.csv_file = None
        self.csv_writer = None
        if csv_path:
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['t',
                                      'x_own', 'y_own', 'th_own',
                                      'x_odom', 'y_odom', 'th_odom',
                                      'x_gt', 'y_gt', 'th_gt'])
            self.get_logger().info(f'Writing samples to {csv_path}')

        self.create_subscription(JointState, '/joint_states', self.on_joint_states,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Odometry, '/ground_truth', self.on_ground_truth, 10)

        self.pub_odom = self.create_publisher(Odometry, '/odom_cinematica', 10)
        self.pub_err_odom = self.create_publisher(Vector3Stamped, '/cinematica/error_odom', 10)
        self.pub_err_gt = self.create_publisher(Vector3Stamped, '/cinematica/error_gt', 10)

        self.create_timer(5.0, self.log_summary)

        self.get_logger().info(f'r = {self.r} m, b = {self.b} m')

    # Cinematica de las llantas
    def on_joint_states(self, msg):
        try:
            i_r = msg.name.index(self.right_joint)
            i_l = msg.name.index(self.left_joint)
        except ValueError:
            return
        if len(msg.velocity) <= max(i_r, i_l):
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is None:
            self.last_stamp = stamp
            return
        dt = stamp - self.last_stamp
        self.last_stamp = stamp
        if dt <= 0.0:
            return

        w_r = msg.velocity[i_r]
        w_l = msg.velocity[i_l]

        # Pasamos de velocidad de llantas a velocidad del robot
        self.v = self.r * (w_r + w_l) / 2.0
        self.omega = self.r * (w_r - w_l) / self.b

        # Integramos la pose usando el angulo a mitad del paso
        theta_mid = self.theta + self.omega * dt / 2.0
        self.x += self.v * math.cos(theta_mid) * dt
        self.y += self.v * math.sin(theta_mid) * dt
        self.theta = wrap_angle(self.theta + self.omega * dt)

        self.publish_odometry(msg.header.stamp)

    def publish_odometry(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega
        self.pub_odom.publish(odom)

    # Comparacion contra la odometria del plugin y la pose real
    def on_ground_truth(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.gt_origin is None:
            self.gt_origin = (p.x, p.y, yaw)
        x0, y0, th0 = self.gt_origin
        # Llevamos la pose real al marco donde arranco el robot
        dx, dy = p.x - x0, p.y - y0
        c, s = math.cos(-th0), math.sin(-th0)
        self.gt = (c * dx - s * dy, s * dx + c * dy, wrap_angle(yaw - th0))

    def on_odom(self, msg):
        if self.last_stamp is None:
            return
        p = msg.pose.pose.position
        x_odom, y_odom = p.x, p.y
        th_odom = yaw_from_quaternion(msg.pose.pose.orientation)

        own = (self.x, self.y, self.theta)
        self.add_error(own, (x_odom, y_odom, th_odom), self.stats_odom,
                       self.pub_err_odom, msg.header.stamp)

        if self.gt is not None:
            self.add_error(own, self.gt, self.stats_gt_own, self.pub_err_gt, msg.header.stamp)
            e_pos = math.hypot(x_odom - self.gt[0], y_odom - self.gt[1])
            self.stats_gt_plugin.add(e_pos, wrap_angle(th_odom - self.gt[2]))

        if self.csv_writer is not None:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            gt = self.gt if self.gt is not None else (float('nan'),) * 3
            self.csv_writer.writerow([f'{t:.3f}', *own, x_odom, y_odom, th_odom, *gt])

    def add_error(self, a, b, stats, publisher, stamp):
        ex, ey = a[0] - b[0], a[1] - b[1]
        eth = wrap_angle(a[2] - b[2])
        stats.add(math.hypot(ex, ey), eth)
        err = Vector3Stamped()
        err.header.stamp = stamp
        err.header.frame_id = 'odom'
        err.vector.x, err.vector.y, err.vector.z = ex, ey, eth
        publisher.publish(err)

    def log_summary(self):
        self.get_logger().info(
            f'q = ({self.x:.3f}, {self.y:.3f}, {math.degrees(self.theta):.1f} deg)\n'
            f'  own    vs /odom        : {self.stats_odom.summary()}\n'
            f'  own    vs ground truth : {self.stats_gt_own.summary()}\n'
            f'  plugin vs ground truth : {self.stats_gt_plugin.summary()}')

    def destroy_node(self):
        self.log_summary()
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CinematicaLlantas()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
