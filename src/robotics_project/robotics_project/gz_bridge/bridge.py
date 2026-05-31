#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time as TimeMsg
import json
import threading
import subprocess


def _quat_correction(ox, oy, oz, ow):
    cw = 0.7071067811865476
    cz = -0.7071067811865476
    return (cw * ox - cz * oy,
            cw * oy + cz * ox,
            cw * oz + cz * ow,
            cw * ow - cz * oz)


class GazeboBridge(Node):

    def __init__(self):
        super().__init__('gz_bridge')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self._echo_process = None
        self._running = True
        self._start_odom_echo()

        self.get_logger().info('Gazebo Bridge started (odometry only)')

    def _start_odom_echo(self):
        def _run():
            try:
                proc = subprocess.Popen(
                    ['gz', 'topic', '-e', '-t', '/odom', '--json-output'],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    text=True, bufsize=1
                )
                self._echo_process = proc
                for line in proc.stdout:
                    if not self._running:
                        break
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        data = json.loads(line)

                        stamp = TimeMsg()
                        s = data.get('header', {}).get('stamp', {})
                        stamp.sec = int(s.get('sec', 0))
                        stamp.nanosec = int(s.get('nsec', 0))

                        odom = Odometry()
                        odom.header.stamp = stamp
                        odom.header.frame_id = 'odom'
                        odom.child_frame_id = 'base_footprint'

                        p = data.get('pose', {}).get('position', {})
                        o = data.get('pose', {}).get('orientation', {})
                        tw = data.get('twist', {}).get('linear', {})
                        ta = data.get('twist', {}).get('angular', {})

                        odom.pose.pose.position.x = float(p.get('x', 0.0))
                        odom.pose.pose.position.y = float(p.get('y', 0.0))
                        odom.pose.pose.position.z = float(p.get('z', 0.0))

                        ox = float(o.get('x', 0.0))
                        oy = float(o.get('y', 0.0))
                        oz = float(o.get('z', 0.0))
                        ow = float(o.get('w', 1.0))

                        cx, cy, cz, cw = _quat_correction(ox, oy, oz, ow)
                        odom.pose.pose.orientation.x = cx
                        odom.pose.pose.orientation.y = cy
                        odom.pose.pose.orientation.z = cz
                        odom.pose.pose.orientation.w = cw

                        lx = float(tw.get('x', 0.0))
                        ly = float(tw.get('y', 0.0))
                        lz = float(tw.get('z', 0.0))
                        ax = float(ta.get('x', 0.0))
                        ay = float(ta.get('y', 0.0))
                        az = float(ta.get('z', 0.0))
                        odom.twist.twist.linear.x = -ly
                        odom.twist.twist.linear.y = lx
                        odom.twist.twist.linear.z = lz
                        odom.twist.twist.angular.x = -ay
                        odom.twist.twist.angular.y = ax
                        odom.twist.twist.angular.z = az

                        self.odom_pub.publish(odom)
                    except Exception as e:
                        self.get_logger().debug(f'Odom parse error: {e}')

            except Exception as e:
                self.get_logger().error(f'Odom echo failed: {e}')

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()

    def destroy_node(self):
        self._running = False
        if self._echo_process:
            self._echo_process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
