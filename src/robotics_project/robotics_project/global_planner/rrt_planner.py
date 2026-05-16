#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Empty
import numpy as np
import time

class RRTPlanner(Node):
    """RRT* Path Planner Node"""
    
    def __init__(self):
        super().__init__('rrt_planner')
        
        # Publisher
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Service
        self.plan_srv = self.create_service(Empty, '/request_replan', self.plan_path_callback)
        
        # Start and goal
        self.start_pos = np.array([0.0, 0.0])
        self.goal_pos = np.array([3.0, 0.0])
        
        # RRT* parameters (TUNE THESE)
        self.max_iterations = 500
        self.step_size = 0.3
        self.goal_bias = 0.2
        self.goal_threshold = 0.3
        
        # Obstacles (from Gazebo world)
        self.obstacles = [
            np.array([2.0, 0.0]),
            np.array([1.0, 1.5]),
            np.array([1.0, -1.5])
        ]
        self.obstacle_radius = 0.5
        
        self.get_logger().info('RRT Planner Node initialized')
        self.get_logger().info(f'Start: {self.start_pos}')
        self.get_logger().info(f'Goal: {self.goal_pos}')
        
        # Trigger planning after 2 seconds
        self.create_timer(2.0, self.initial_planning)
        self.first_planning_done = False
    
    def initial_planning(self):
        """Trigger planning on startup"""
        if not self.first_planning_done:
            self.get_logger().info('Triggering initial planning...')
            self.plan_path_callback(None, None)
            self.first_planning_done = True
    
    def plan_path_callback(self, request, response):
        """Main planning function"""
        self.get_logger().info('='*50)
        self.get_logger().info('PLANNING NEW PATH')
        self.get_logger().info('='*50)
        
        start_time = time.time()
        tree = self.rrt_star()
        computation_time = time.time() - start_time
        
        self.get_logger().info(f'Computation time: {computation_time:.3f} seconds')
        
        path_msg = self.tree_to_path_message(tree)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(tree)} nodes')
        
        if response is not None:
            return response
    
    def rrt_star(self):
        """RRT* Algorithm"""
        tree = [self.start_pos.copy()]
        self.get_logger().info(f'Tree initialized with start position: {self.start_pos}')
        
        for iteration in range(self.max_iterations):
            # A) Sample random point
            if np.random.random() < self.goal_bias:
                sample = self.goal_pos.copy()
            else:
                sample = np.array([
                    np.random.uniform(-1, 4),
                    np.random.uniform(-2, 2)
                ])
            
            # B) Find nearest node
            nearest_idx = self.find_nearest_node(tree, sample)
            nearest = tree[nearest_idx]
            
            # C) Extend towards sample
            direction = sample - nearest
            distance = np.linalg.norm(direction)
            
            if distance > 1e-6:
                direction = direction / distance
            
            new_node = nearest + direction * self.step_size
            
            # D) Check collision
            if self.is_collision_free(new_node):
                tree.append(new_node)
            else:
                continue
            
            # E) Check goal reached
            distance_to_goal = np.linalg.norm(new_node - self.goal_pos)
            if distance_to_goal < self.goal_threshold:
                self.get_logger().info(f'✓ Goal reached at iteration {iteration}!')
                tree.append(self.goal_pos.copy())
                break
            
            if iteration % 50 == 0:
                self.get_logger().info(f'Iteration {iteration}: Tree size = {len(tree)}')
        
        self.get_logger().info(f'RRT* finished. Tree size: {len(tree)}')
        return tree
    
    def find_nearest_node(self, tree, sample):
        """Find closest node in tree to sample"""
        min_distance = np.inf
        nearest_idx = 0
        
        for i, node in enumerate(tree):
            distance = np.linalg.norm(sample - node)
            if distance < min_distance:
                min_distance = distance
                nearest_idx = i
        
        return nearest_idx
    
    def is_collision_free(self, point):
        """Check if point is safe"""
        for obstacle_center in self.obstacles:
            distance_to_obstacle = np.linalg.norm(point - obstacle_center)
            if distance_to_obstacle < self.obstacle_radius:
                return False
        return True
    
    def tree_to_path_message(self, tree):
        """Convert tree to Path message"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for point in tree:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            pose_stamped.pose.position.x = float(point[0])
            pose_stamped.pose.position.y = float(point[1])
            pose_stamped.pose.position.z = 0.0
            
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        return path_msg


def main(args=None):
    rclpy.init(args=args)
    planner = RRTPlanner()
    rclpy.spin(planner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()