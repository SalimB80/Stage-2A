import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

from .detector import detect_helmet, bearing_to_angle, range_from_lidar
from .formations import get_offset


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower')

        self.declare_parameter('robot_index', 2)
        self.declare_parameter('formation', 'colonne')
        self.declare_parameter('max_lin', 0.18)
        self.declare_parameter('max_ang', 1.0)
        self.declare_parameter('k_lin', 0.35)
        self.declare_parameter('k_ang', 0.8)
        self.declare_parameter('stop_range', 0.25)
        self.declare_parameter('search_when_lost', True)
        self.declare_parameter('deadband_bearing', 0.12)
        self.declare_parameter('deadband_range', 0.10)

        idx = self.get_parameter('robot_index').value
        form = self.get_parameter('formation').value
        self.des_range, self.des_bearing = get_offset(form, idx)
        self.get_logger().info(
            f'Follower idx={idx} formation={form} '
            f'-> range={self.des_range:.2f} bearing={self.des_bearing:.2f}rad')

        self.bridge = CvBridge()
        self.last_scan = None
        self.lost_count = 0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Image, 'camera/image_raw', self.image_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan',
                                 lambda m: setattr(self, 'last_scan', m),
                                 qos_profile_sensor_data)

        self.add_on_set_parameters_callback(self.on_params)

    def on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        form = self.get_parameter('formation').value
        idx = self.get_parameter('robot_index').value
        for p in params:
            if p.name == 'formation':
                form = p.value
            if p.name == 'robot_index':
                idx = p.value
        if any(p.name in ('formation', 'robot_index') for p in params):
            self.des_range, self.des_bearing = get_offset(form, idx)
            self.get_logger().info(
                f'MAJ offset -> range={self.des_range:.2f} '
                f'bearing={self.des_bearing:.2f}')
        return SetParametersResult(successful=True)

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        det = detect_helmet(img)
        cmd = Twist()

        if det is None:
            self.lost_count += 1
            if self.lost_count > 10 and \
               self.get_parameter('search_when_lost').value:
                cmd.angular.z = 0.3
            self.cmd_pub.publish(cmd)
            return

        self.lost_count = 0
        bearing_norm, area_ratio, _ = det
        angle = bearing_to_angle(bearing_norm)

        rng = None
        if self.last_scan is not None:
            rng = range_from_lidar(self.last_scan, angle)
        if rng is None:
            rng = 0.5 / max(area_ratio, 1e-3) ** 0.5  # fallback grossier

        e_range = rng - self.des_range
        e_bearing = angle - self.des_bearing

        max_l = self.get_parameter('max_lin').value
        max_a = self.get_parameter('max_ang').value
        k_l = self.get_parameter('k_lin').value
        k_a = self.get_parameter('k_ang').value
        db_b = self.get_parameter('deadband_bearing').value
        db_r = self.get_parameter('deadband_range').value

        if abs(e_bearing) < db_b:
            ang_cmd = 0.0
        else:
            ang_cmd = k_a * e_bearing

        if abs(e_range) < db_r:
            lin_cmd = 0.0
        else:
            lin_cmd = k_l * e_range

        cmd.linear.x = max(-max_l, min(max_l, lin_cmd))
        cmd.angular.z = max(-max_a, min(max_a, ang_cmd))

        if rng < self.get_parameter('stop_range').value:
            cmd.linear.x = min(cmd.linear.x, 0.0)

        src = 'lidar' if self.last_scan is not None and \
            range_from_lidar(self.last_scan, angle) is not None else 'FALLBACK'
        self.get_logger().info(
            f'rng={rng:.2f}({src}) e_r={e_range:+.2f} '
            f'bear={angle:+.2f} e_b={e_bearing:+.2f} '
            f'-> lin={cmd.linear.x:+.2f} ang={cmd.angular.z:+.2f}',
            throttle_duration_sec=0.5)

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
