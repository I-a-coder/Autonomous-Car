#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Empty
import numpy as np
import time


class RRTPlanner(Node):

    def __init__(self):
        super().__init__('rrt_planner')

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.plan_srv = self.create_service(Empty, '/request_replan', self.plan_path_callback)

        self.start_pos = np.array([0.0, 0.0])
        self.goal_pos  = np.array([20.0, 0.0])

        self.max_iterations   = 2000
        self.step_size        = 1.0
        self.goal_bias        = 0.15
        self.goal_threshold   = 1.0

        self.obstacles = []          # add real obstacle coords here
        self.obstacle_radius = 1.0

        self.latest_path = None
        self.first_planning_done = False

        self.get_logger().info(f'RRT Planner ready. Start={self.start_pos} Goal={self.goal_pos}')

        # Wait for subscriber then plan
        self.create_timer(1.0, self.initial_planning)
        # Republish path every 3s for any late subscribers
        self.create_timer(3.0, self.republish_path)

    def initial_planning(self):
        if self.first_planning_done:
            return
        count = self.path_pub.get_subscription_count()
        self.get_logger().info(f'Waiting for controller... ({count} subscribers)')
        if count > 0:
            self.get_logger().info('Controller connected! Starting RRT planning...')
            self.plan_path_callback(None, None)
            self.first_planning_done = True

    def republish_path(self):
        if self.latest_path is not None:
            self.path_pub.publish(self.latest_path)
            self.get_logger().info('Path republished', throttle_duration_sec=5.0)

    def plan_path_callback(self, request, response):
        self.get_logger().info('=' * 50)
        self.get_logger().info('PLANNING NEW PATH')

        start_time = time.time()
        path_nodes, success = self.rrt_star()
        elapsed = time.time() - start_time

        self.get_logger().info(f'Planning took {elapsed:.3f}s')

        if success:
            self.get_logger().info(f'Path found with {len(path_nodes)} waypoints')
            path_msg = self.nodes_to_path_message(path_nodes)
            self.latest_path = path_msg
            self.path_pub.publish(path_msg)
        else:
            self.get_logger().warn('No path found! Increase max_iterations or check obstacles.')

        if response is not None:
            return response

    def rrt_star(self):
        nodes   = [self.start_pos.copy()]
        parents = [-1]

        goal_reached    = False
        goal_node_idx   = -1

        for iteration in range(self.max_iterations):
            # Sample
            if np.random.random() < self.goal_bias:
                sample = self.goal_pos.copy()
            else:
                sample = np.array([
                    np.random.uniform(self.start_pos[0] - 2, self.goal_pos[0] + 2),
                    np.random.uniform(-5.0, 5.0)
                ])

            # Nearest node
            nearest_idx = int(np.argmin([np.linalg.norm(n - sample) for n in nodes]))
            nearest     = nodes[nearest_idx]

            # Steer
            direction = sample - nearest
            dist = np.linalg.norm(direction)
            if dist < 1e-6:
                continue
            direction /= dist
            new_node = nearest + direction * min(self.step_size, dist)

            # Collision check along segment
            if not self.is_collision_free(nearest, new_node):
                continue

            new_idx = len(nodes)
            nodes.append(new_node)
            parents.append(nearest_idx)

            # Goal check
            if np.linalg.norm(new_node - self.goal_pos) < self.goal_threshold:
                self.get_logger().info(f'Goal reached at iteration {iteration}!')
                nodes.append(self.goal_pos.copy())
                parents.append(new_idx)
                goal_node_idx = len(nodes) - 1
                goal_reached = True
                break

            if iteration % 200 == 0:
                closest = min(np.linalg.norm(n - self.goal_pos) for n in nodes)
                self.get_logger().info(
                    f'Iter {iteration}: tree={len(nodes)} nodes, closest to goal={closest:.2f}m')

        if not goal_reached:
            return [], False

        # Trace path back from goal to start
        path = []
        idx = goal_node_idx
        while idx != -1:
            path.append(nodes[idx])
            idx = parents[idx]
        path.reverse()
        return path, True

    def is_collision_free(self, from_pt, to_pt):
        for t in np.linspace(0, 1, 10):
            pt = from_pt + t * (to_pt - from_pt)
            for obs in self.obstacles:
                if np.linalg.norm(pt - obs) < self.obstacle_radius:
                    return False
        return True

    def nodes_to_path_message(self, path_nodes):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp    = self.get_clock().now().to_msg()

        for point in path_nodes:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp    = self.get_clock().now().to_msg()
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg


def main(args=None):
    rclpy.init(args=args)
    planner = RRTPlanner()
    rclpy.spin(planner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()