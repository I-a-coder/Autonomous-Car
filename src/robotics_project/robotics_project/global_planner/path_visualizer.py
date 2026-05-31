#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/path_markers',
            10
        )

        self.get_logger().info('Path Visualizer ready')

    def path_callback(self, msg):
        if not msg.poses:
            self.get_logger().warn("Received empty path")
            return

        marker_array = MarkerArray()

        # 🔥 FORCE FRAME (IMPORTANT FIX)
        frame = "odom"
        stamp = self.get_clock().now().to_msg()

        # --- LINE STRIP (path line) ---
        line = Marker()
        line.header.frame_id = frame
        line.header.stamp = stamp
        line.ns = 'rrt_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD

        line.scale.x = 0.2
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 1.0

        for pose_stamped in msg.poses:
            p = Point()
            p.x = pose_stamped.pose.position.x
            p.y = pose_stamped.pose.position.y
            p.z = 0.2
            line.points.append(p)

        marker_array.markers.append(line)

        # --- WAYPOINT SPHERES ---
        for i, pose_stamped in enumerate(msg.poses):
            sphere = Marker()
            sphere.header.frame_id = frame
            sphere.header.stamp = stamp
            sphere.ns = 'rrt_waypoints'
            sphere.id = i + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD

            sphere.pose.position.x = pose_stamped.pose.position.x
            sphere.pose.position.y = pose_stamped.pose.position.y
            sphere.pose.position.z = 0.2

            sphere.scale.x = 0.3
            sphere.scale.y = 0.3
            sphere.scale.z = 0.3

            sphere.color.r = 1.0
            sphere.color.g = 0.5
            sphere.color.b = 0.0
            sphere.color.a = 1.0

            marker_array.markers.append(sphere)

        # --- START ---
        start = Marker()
        start.header.frame_id = frame
        start.header.stamp = stamp
        start.ns = 'rrt_start_goal'
        start.id = 1000
        start.type = Marker.CYLINDER
        start.action = Marker.ADD

        start.pose.position.x = msg.poses[0].pose.position.x
        start.pose.position.y = msg.poses[0].pose.position.y
        start.pose.position.z = 0.5

        start.scale.x = 0.6
        start.scale.y = 0.6
        start.scale.z = 1.0

        start.color.r = 0.0
        start.color.g = 0.0
        start.color.b = 1.0
        start.color.a = 1.0

        marker_array.markers.append(start)

        # --- GOAL ---
        goal = Marker()
        goal.header.frame_id = frame
        goal.header.stamp = stamp
        goal.ns = 'rrt_start_goal'
        goal.id = 1001
        goal.type = Marker.SPHERE
        goal.action = Marker.ADD

        goal.pose.position.x = msg.poses[-1].pose.position.x
        goal.pose.position.y = msg.poses[-1].pose.position.y
        goal.pose.position.z = 0.2

        goal.scale.x = 1.0
        goal.scale.y = 1.0
        goal.scale.z = 1.0

        goal.color.r = 0.0
        goal.color.g = 1.0
        goal.color.b = 0.0
        goal.color.a = 1.0

        marker_array.markers.append(goal)

        # Publish
        self.marker_pub.publish(marker_array)

        self.get_logger().info(
            f'Published {len(marker_array.markers)} markers'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()