#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import math


class SafetyMonitor(Node):

    def __init__(self):
        super().__init__('safety_monitor')

        # --- Parameters ---
        self.danger_distance = 0.5      # metres — obstacle closer than this = danger
        self.warn_distance   = 1.5      # metres — obstacle closer than this = warning
        self.check_angle_deg = 45.0     # degrees either side of forward to check
        self.replan_cooldown = 3.0      # seconds between replans

        # --- Subscribers ---
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # --- Publishers ---
        self.replan_pub = self.create_publisher(Bool,   '/replan_trigger', 10)
        self.status_pub = self.create_publisher(String, '/safety_status',  10)

        # --- Replan service client ---
        self.replan_client = self.create_client(Empty, '/request_replan')

        # --- State ---
        self.robot_x         = 0.0
        self.robot_y         = 0.0
        self.current_speed   = 0.0
        self.obstacle_active = False
        self.last_replan_time = 0.0

        self.get_logger().info('Safety Monitor initialised')
        self.get_logger().info(
            f'  danger={self.danger_distance}m  '
            f'warn={self.warn_distance}m  '
            f'angle=±{self.check_angle_deg}°'
        )

    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx**2 + vy**2)

    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        angle_min  = msg.angle_min
        angle_inc  = msg.angle_increment
        ranges     = msg.ranges
        range_min  = msg.range_min
        range_max  = msg.range_max

        check_rad = math.radians(self.check_angle_deg)

        closest_front   = float('inf')
        closest_warning = float('inf')

        for i, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if not (range_min <= r <= range_max):
                continue

            angle = angle_min + i * angle_inc
            while angle >  math.pi: angle -= 2 * math.pi
            while angle < -math.pi: angle += 2 * math.pi

            if abs(angle) <= check_rad:
                if r < closest_front:
                    closest_front = r
            if abs(angle) <= check_rad * 2:
                if r < closest_warning:
                    closest_warning = r

        # --- Decision logic ---
        replan_msg = Bool()
        status_msg = String()

        if closest_front <= self.danger_distance:
            replan_msg.data = True
            status_msg.data = (
                f'DANGER — obstacle at {closest_front:.2f}m! Triggering replan.'
            )
            if not self.obstacle_active:
                self.get_logger().warn(status_msg.data)
                self.obstacle_active = True
                self.trigger_replan()

        elif closest_warning <= self.warn_distance:
            replan_msg.data = False
            status_msg.data = (
                f'WARNING — obstacle at {closest_warning:.2f}m ahead. Slowing down.'
            )
            self.get_logger().info(status_msg.data)
            self.obstacle_active = False

        else:
            replan_msg.data = False
            status_msg.data = f'CLEAR — nearest obstacle: {closest_front:.2f}m'
            if self.obstacle_active:
                self.get_logger().info('Path clear — resuming.')
            self.obstacle_active = False

        self.replan_pub.publish(replan_msg)
        self.status_pub.publish(status_msg)

    # ------------------------------------------------------------------
    def trigger_replan(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_replan_time < self.replan_cooldown:
            self.get_logger().info('Replan cooldown active — skipping.')
            return

        self.get_logger().warn(
            f'Requesting replan from position ({self.robot_x:.2f}, {self.robot_y:.2f})'
        )

        if not self.replan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Replan service not available!')
            return

        self.last_replan_time = now
        future = self.replan_client.call_async(Empty.Request())
        future.add_done_callback(self.replan_response_callback)

    def replan_response_callback(self, future):
        try:
            future.result()
            self.get_logger().info('Replan request accepted by RRT planner!')
        except Exception as e:
            self.get_logger().error(f'Replan service call failed: {e}')


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
