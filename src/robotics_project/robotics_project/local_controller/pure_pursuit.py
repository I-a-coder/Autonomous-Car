#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
import math


def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        self.lookahead_distance = 1.5
        self.max_speed          = 0.3
        self.max_angular_vel    = 1.0
        self.goal_tolerance     = 0.2

        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_theta = 0.0
        self.odom_received   = False
        self.current_path    = None
        self.path_received   = False
        self.goal_reached    = False

        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub  = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Float32, '/cross_track_error', 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Pure Pursuit Controller initialised')
        self.get_logger().info(f'  lookahead={self.lookahead_distance}m  speed={self.max_speed}m/s  max_w={self.max_angular_vel}rad/s')

    def path_callback(self, msg):
        if len(msg.poses) < 2:
            return
        self.current_path  = msg
        self.path_received = True
        self.goal_reached  = False
        self.get_logger().info(f'New path received: {len(msg.poses)} waypoints')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.robot_x     = pos.x
        self.robot_y     = pos.y
        self.robot_theta = euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
        self.odom_received = True

    def control_loop(self):
        if not self.odom_received:
            return
        if not self.path_received or self.current_path is None:
            return
        if self.goal_reached:
            self.stop_robot()
            return

        goal = self.current_path.poses[-1].pose.position
        dist_to_goal = math.hypot(goal.x - self.robot_x, goal.y - self.robot_y)
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(f'Goal reached! (dist={dist_to_goal:.3f}m)')
            self.goal_reached = True
            self.stop_robot()
            return

        lookahead_pt = self.find_lookahead_point()
        if lookahead_pt is None:
            self.stop_robot()
            return

        dx = lookahead_pt.x - self.robot_x
        dy = lookahead_pt.y - self.robot_y
        e  = -math.sin(self.robot_theta) * dx + math.cos(self.robot_theta) * dy

        L     = self.lookahead_distance
        kappa = (2.0 * e) / (L ** 2)
        omega = self.max_speed * kappa
        omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))

        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.twist.linear.x  = self.max_speed
        cmd.twist.angular.z = omega
        self.cmd_pub.publish(cmd)

        cte_msg = Float32()
        cte_msg.data = float(abs(e))
        self.error_pub.publish(cte_msg)

        self.get_logger().info(
            f'lookahead=({lookahead_pt.x:.2f},{lookahead_pt.y:.2f})  e={e:.3f}m  w={omega:.3f}rad/s',
            throttle_duration_sec=1.0)

    def find_lookahead_point(self):
        poses = self.current_path.poses
        closest_idx  = 0
        closest_dist = float('inf')
        for i, pose_stamped in enumerate(poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            d  = math.hypot(px - self.robot_x, py - self.robot_y)
            if d < closest_dist:
                closest_dist = d
                closest_idx  = i

        accumulated = 0.0
        for i in range(closest_idx, len(poses) - 1):
            x0 = poses[i].pose.position.x
            y0 = poses[i].pose.position.y
            x1 = poses[i + 1].pose.position.x
            y1 = poses[i + 1].pose.position.y
            accumulated += math.hypot(x1 - x0, y1 - y0)
            if accumulated >= self.lookahead_distance:
                return poses[i + 1].pose.position

        return poses[-1].pose.position

    def stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
