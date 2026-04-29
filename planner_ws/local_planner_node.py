#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── Tuning knobs ──────────────────────────────────────────
K_ATT   = 1.2   # how strongly the drone is pulled toward the goal
K_REP   = 3.0   # how strongly the drone is pushed away from obstacles
D0      = 3.0   # metres — obstacles beyond this distance are ignored
V_MAX   = 2.0   # maximum speed in m/s
# ─────────────────────────────────────────────────────────

class LocalPlannerNode(Node):

    def __init__(self):
        super().__init__('local_planner_node')

        # Define a QoS profile that matches MAVROS and Camera sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.pose          = None
        self.goal          = None
        self.obstacle_pts  = None

        # Apply sensor_qos to the position and camera subscribers
        self.create_subscription(PoseStamped,
            '/mavros/local_position/pose', self.pose_cb, sensor_qos)

        self.create_subscription(PointCloud2,
            '/camera/depth/points', self.cloud_cb, sensor_qos)

        # Goal can stay at 10 (Reliable) as it's usually sent from a standard CLI
        self.create_subscription(PoseStamped,
            '/goal_pose', self.goal_cb, 10)

        # Send velocity commands to the drone via MAVROS
        self.vel_pub = self.create_publisher(Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Run the planner at 20 Hz
        self.create_timer(1.0 / 20.0, self.control_loop)

        self.get_logger().info('Local planner node started')

    # ── Callbacks ─────────────────────────────────────────

    def pose_cb(self, msg):
        self.pose = msg

    def goal_cb(self, msg):
        self.goal = msg
        self.get_logger().info(
            f'New goal received: x={msg.pose.position.x:.1f} '
            f'y={msg.pose.position.y:.1f} z={msg.pose.position.z:.1f}')

    def cloud_cb(self, msg):
        # Convert the ROS point cloud message into a plain numpy array
        pts = np.array(
            [(p[0], p[1], p[2])
             for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)],
            dtype=np.float32
        )

        if len(pts) == 0:
            self.obstacle_pts = None
            return

        # Remove ground returns — ignore anything more than 1m below or 4m above the drone
        pts = pts[(pts[:, 2] > -1.0) & (pts[:, 2] < 4.0)]

        # Only keep points within the influence radius to save CPU
        distances = np.linalg.norm(pts, axis=1)
        pts = pts[distances < D0]

        self.obstacle_pts = pts if len(pts) > 0 else None

    # ── Main control loop ─────────────────────────────────

    def control_loop(self):
        # Do nothing until we have both a position fix and a goal
        if self.pose is None or self.goal is None:
            return

        pos  = self._to_array(self.pose)
        goal = self._to_array(self.goal)

        # Stop if we're close enough to the goal
        if np.linalg.norm(goal - pos) < 0.5:
            self.get_logger().info('Goal reached!')
            self.goal = None
            self._stop()
            return

        # ── 1. Attractive force — pulls toward goal ────────
        diff      = goal - pos
        dist      = np.linalg.norm(diff)
        f_att     = K_ATT * diff / dist * min(dist, 3.0)

        # ── 2. Repulsive force — pushes away from obstacles ─
        f_rep = np.zeros(3)
        if self.obstacle_pts is not None:
            for obs_point in self.obstacle_pts:
                d = float(np.linalg.norm(obs_point))
                d = max(d, 0.05)   # avoid division by zero
                if d < D0:
                    # direction pointing away from this obstacle point
                    direction_away = -obs_point / d
                    magnitude      = K_REP * (1.0/d - 1.0/D0) * (1.0/d**2)
                    f_rep         += magnitude * direction_away

        # ── 3. Combine and cap speed ───────────────────────
        f_total = f_att + f_rep
        speed   = np.linalg.norm(f_total)
        if speed > V_MAX:
            f_total = f_total / speed * V_MAX

        self._publish_velocity(f_total)

        self.get_logger().info(
            f'dist={dist:.1f}m  '
            f'f_att={np.linalg.norm(f_att):.2f}  '
            f'f_rep={np.linalg.norm(f_rep):.2f}  '
            f'vel={f_total.round(2)}')

    # ── Helpers ───────────────────────────────────────────

    def _to_array(self, pose_stamped):
        p = pose_stamped.pose.position
        return np.array([p.x, p.y, p.z])

    def _publish_velocity(self, vel):
        msg = Twist()
        msg.linear.x = float(vel[0])
        msg.linear.y = float(vel[1])
        msg.linear.z = float(vel[2])
        self.vel_pub.publish(msg)

    def _stop(self):
        self.vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()