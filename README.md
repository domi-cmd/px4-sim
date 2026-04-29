# Exercise 9 Submission

Link to repository: https://github.com/domi-cmd/px4-sim/

For setting up the github project, docker and QGroundControll, refer to the step by step of Exercise 8 below.

The project environment is containerized. You can build the Docker image containing all dependencies and the local planner node using the provided Dockerfile.

The code for the new node looks as follows:
```py
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
```

It relies on an Artificial Potential Field (APF), and uses a pulling force (k_att) which moves it towards the goal, as well as a repelling force (k_rep) which makes it avoid
obstacles more agrgessively if needed.

To push the node script to the docker container, place it in a folder at root (in my case, in planner_ws), name the file local_planner_node.py and run:
```cmd
docker exec -it px4_sitl bash
root@338622ee7517:~# docker cp planner_ws/local_planner_node.py 338622ee7517:/root/planner_ws/local_planner_node.py
Successfully copied 7.68kB to 338622ee7517:/root/planner_ws/local_planner_node.py
```

Verify that adding the script worked by running:
```cmd

verify node creation and addition to docker container:

docker exec -it px4_sitl bash   
root@338622ee7517:~# ls -l /root/planner_ws/local_planner_node.py
-rwxr-xr-x 1 root root 5884 Apr 29 12:47 /root/planner_ws/local_planner_node.py
```


Now you just need to start the drone simulation! For this, open a terminal and run:
```cmd
docker exec -it px4_sitl bash
root@338622ee7517:~# cd /root/PX4-Autopilot
root@338622ee7517:~# make px4_sitl gz_x500_depth PX4_GZ_WORLD=baylands
```

In the same terminal, disable safety warnings:

```cmd
param set COM_ARM_WO_GPS 1
param set COM_RC_IN_MODE 4
param set NAV_DLL_ACT 0
param set NAV_RCL_ACT 0
```

In a new terminal, open the ros-Gazebo bridge:
```cmd
docker exec -it px4_sitl bash
ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args \
  -r /depth_camera/points:=/camera/depth/points
```

In a third terminal, start MAVROS:
```cmd
docker exec -it px4_sitl bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557
```

Then start and connect to QGroundControl (see guide for Exercise 8 below)

Once we connected to the drone in QGroundControl, in the first terminal were we loaded the baylands map, arm and start the drone 
```cmd
commander arm
commander takeoff
```

You can now open your browser at localhost:6080, connect either to VCN or vcn_lite using the password: 1234, and see the drone.

Now to run the pathing script for the drone, run the node script in a new terminal:
```cmd
docker exec -it px4_sitl bash   
python3 /root/planner_ws/local_planner_node.py                                                      
[INFO] [1777467855.654782191] [local_planner_node]: Local planner node started
```


To send the drone to a new target location/position, open a new terminal and run:
```cmd
docker exec -it px4_sitl bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 10.0, y: 0.0, z: 3.0}, orientation: {w: 1.0}}}'
root@338622ee7517:~# ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 10.0, y: 0.0, z: 3.0}, orientation: {w: 1.0}}}'
Waiting for at least 1 matching subscription(s)...
publisher: beginning loop
publishing #1: geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='map'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=10.0, y=0.0, z=3.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))
```

You should now see useful metrics in the terminal where you started the node script, such as the distance to the target, the x,y and z coordinates, the velocity and the attractive and repelling forces that make the drone avoid obstacles on the way to the destination.

To actually make the drone move, you need to allow the script to take control of it. For this, run in a new terminal to change the drone to offboard mode:
```cmd
docker exec -it px4_sitl bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
```

And thats it! The drone should now move towards the given destination while avoiding obstacles. To get further insights such as the angular velocity, you can run (in again a new terminal):
```cmd
docker exec -it px4_sitl bash
ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped
```

Below is a video of an incredibly low framerate drone (best my laptop can manage) moving towards the given destination, as well as relevant metrics as specified above:

<img width="1803" height="915" alt="task_9" src="https://github.com/user-attachments/assets/c8672279-227e-43e7-ab28-c05468cdd4ce" />


# Exercise 8 Submission

Link to repository: https://github.com/domi-cmd/px4-sim/

## Problems
I managed to finish neither of the two tasks completely. I have spent countless hours on trying to accomplish them this week, but am unable to do so due to what I think are hardware limitations. This homework was genuinely the most frustrating experience I have had in over 4 years studying at this university.

Docker Image: https://hub.docker.com/r/domicmd/homework_8

## Step by step set up of project
1. Fork the repository: https://github.com/erdemuysalx/px4-sim
2. Clone it locally on your remote device
3. Navigate into the root of the project using a terminal capable of executing bash commands
4. Build the project by running:
```cmd
./build.sh --all
```
5. Install docker desktop and start it using
```cmd
docker-compose up
```

## Task 1
This task requires the creation (or download) of a gazebo environment and a ros2 node. 
1. The following environment can simply be downloaded to avoid manual creation of one: https://app.gazebosim.org/VentuRobotics/fuel/models/baylands.
2. Place the following code for the node in a new folder in the root directoy called "yolo_ws", call the node file "yolo_node.py":
```py
#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/yolo/image_annotated', 10)
        self.create_subscription(Image, '/camera/image_raw', self.cb, 10)

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        t0 = time.perf_counter()
        results = self.model.predict(frame, conf=0.45, verbose=False)
        ms = (time.perf_counter() - t0) * 1000
        annotated = results[0].plot()
        self.get_logger().info(f'{len(results[0].boxes)} detections | {ms:.1f}ms')
        self.pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))

rclpy.init()
rclpy.spin(YoloNode())
```

To be able to measure performance and whatnot, we need to install ultralytics. To do so we first need to create a venv.
```cmd
apt update
apt install -y python3-venv
python3 -m venv /root/venv
source /root/venv/bin/activate
pip install ultralytics
```

This step will take most likely forever and is incredibly frustrating, as it downloads a lot of things that we do not care about :)

3. Test out if the connection and drone control without the custom node first. For this, create a terminal in your project root, run
```cmd
docker exec -it px4_sitl bash
cd /root/PX4-Autopilot
make px4_sitl gz_x500 PX4_GZ_WORLD=baylands
```
4. In a second terminal, run:
```cmd
docker exec -it px4_sitl bash
ros2 run ros_gz_bridge parameter_bridge
/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image
/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image
/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
--ros-args
-r /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info:=/camera/color/camera_info
-r /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera/color/image
-r /depth_camera:=/camera/depth/image
-r /depth_camera/points:=/camera/depth/points
```
5. In a third, run:
```cmd
docker exec -it px4_sitl bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557
```
6. Go back to the first terminal and disable warnings using:
```cmd
param set COM_ARM_WO_GPS 1
param set COM_RC_IN_MODE 4
param set NAV_DLL_ACT 0
param set NAV_RCL_ACT 0
```
7. Connect to drone over UDP by downloading: http://qgroundcontrol.com, and creating a new connection with the following parameters:
```
Name: PX4-Docker-Sim

Type: UDP

Listening Port: 15871 (You may need to change this to 14550)

Target Hosts (Server): Click Add and enter 127.0.0.1:18570
```

8. Open http://localhost:6080/vnc_lite.html to watch the drone simulation
9. Launch the drone by running both:
```cmd
commander arm
commander takeoff
```
10. Land it using
```cmd
commander land
```

11. Video demonstrating the steps above: (I had to drastically cut everything interesting from the video length to accomodate Githubs 10mb limit)
<img width="1899" height="985" alt="task_11" src="https://github.com/user-attachments/assets/425c80fc-6121-44ca-adfb-eddeb94bcbd3" />


12. (Didnt work) run the same thing, but with our custom node in an extra terminal:
```cmd
docker exec -it px4_sitl bash
python3 /root/yolo_ws/yolo_node.py
```


### Why it didnt work:

Whenever I tried to use the node, I ran into performance issue where gazebo would crash. The framerate was terrible even when things worked. I was unable to capture a moment where it actually worked, even after lowering performance parameters and forcing the launch:
<img width="599" height="189" alt="3" src="https://github.com/user-attachments/assets/1784e66b-4285-4519-b39c-a2bb0a82c450" />

## Task 2
1. The custom node required here that should be added to the same folder as task 1 is this:
```cmd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import message_filters
import open3d as o3d
import numpy as np
import cv2

class DepthPerceptionNode(Node):
    def __init__(self):
        super().__init__('depth_perception_node')
        self.bridge = CvBridge()
        self.pcd_publisher = self.create_publisher(PointCloud2, '/filtered_cloud', 10)
        
        # Camera Intrinsics (Initialized via CameraInfo)
        self.intrinsic = None

        # Subscribers
        self.info_sub = self.create_subscription(CameraInfo, 
            '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info', 
            self.info_callback, 10)
            
        self.rgb_sub = message_filters.Subscriber(self, Image, 
            '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image')
            
        self.depth_sub = message_filters.Subscriber(self, Image, 
            '/world/baylands/model/x500_depth_0/link/camera_link/sensor/StereoOV7251/depth_image')

        # Synchronize RGB and Depth (10-frame queue, 0.1s max slop)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

    def info_callback(self, msg):
        # Extract intrinsic parameters from CameraInfo
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            msg.width, msg.height, msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        )

    def sync_callback(self, rgb_msg, depth_msg):
        if self.intrinsic is None:
            return

        # 1. Convert ROS images to Open3D format
        color_data = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_data = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1") # Gazebo depth is Float32

        # Convert meters to millimeters (Open3D standard)
        depth_mm = (depth_data * 1000).astype(np.uint16)
        
        color_o3d = o3d.geometry.Image(cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB))
        depth_o3d = o3d.geometry.Image(depth_mm)

        # 2. Create RGBD Image and PointCloud
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d, depth_scale=1000.0, depth_trunc=10.0, convert_rgb_to_intensity=False
        )
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsic)

        # 3. RANSAC Ground Segmentation
        if len(pcd.points) > 100:
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=1000)
            
            # Filter: Keep only the outliers (everything that ISN'T the ground)
            obstacle_pcd = pcd.select_by_index(inliers, invert=True)
            
            # 4. Convert back to ROS PointCloud2
            self.publish_pc2(obstacle_pcd, rgb_msg.header)

    def publish_pc2(self, pcd, header):
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) * 255
        
        # Combine XYZ and RGB (packed into float)
        cloud_data = []
        for i in range(len(points)):
            r, g, b = colors[i].astype(int)
            rgb_packed = (r << 16) | (g << 8) | b
            cloud_data.append([points[i][0], points[i][1], points[i][2], rgb_packed])

        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
        ]

        pc2_msg = pc2.create_cloud(header, fields, cloud_data)
        self.pcd_publisher.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. Running the drone and the node works the same way as shown in task 1 above. Because of this, I ran into the exact same problems as with task 1.


# PX4-Sim

Fully containerized PX4 Autopilot simulation environment with browser-based GUI access.

## What's Included

- **PX4 Autopilot** - Pre-built and ready to use
- **ROS 2 Jazzy** - Latest ROS 2 LTS release
- **Gazebo Harmonic** - Latest Gazebo simulator
- **ROS Gazebo Bridge** (`ros-jazzy-ros-gz-bridge`) - Bidirectional transport bridge between Gazebo and ROS
- **MAVROS** (`ros-jazzy-mavros`) - PX4 to ROS gateway
- **TigerVNC + NoVNC** - Browser-based desktop access
- **XFCE4 Desktop** - Full desktop environment

## Quick Start

### 1. Build the Images

```bash
# Build everything
./build.sh --all

# Or build step by step
./build.sh --base  # ROS 2 + Gazebo
./build.sh --full  # PX4 Autopilot + MAVROS + NoVNC
```

### 2. Run the Container

```bash
# Interactive mode (recommended)
docker-compose up

# Or detached mode
docker-compose up -d
docker attach px4_sitl
```

### 3. Access the GUI

You can access GUI applications in different ways depending on your operating system:

#### macOS

You can access the GUI via built-in **noVNC** using your browser:

- Open: http://localhost:6080/vnc.html
- Password: `1234`

#### Linux

You can use **X11 forwarding** by enabling the appropriate configuration in your `docker-compose.yml` file (uncomment or add the required lines).

```yaml
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw

    environment:
      #- DISPLAY=:1
      - DISPLAY=${DISPLAY}
```

### 4. Control Interface

You can control your vehicle in simulation using different ways:

#### Connect QGroundControl (optional)

This step is required only if you want to control the vehicle using a graphical user interface.

1. Install **QGroundControl** on your host machine:
   http://qgroundcontrol.com

2. Create a custom communication link with the following configuration:

   * Type: UDP
   * Port: `15871`
   * Server: `0.0.0.0:18570`

3. QGroundControl will automatically connect to:

   * `udp://localhost:18570`

#### Offboard mode (alternative)

Alternatively, you can control the vehicle in **offboard mode**. This mode bypasses certain PX4 safety constraints in **PX4 Autopilot** when running in SITL.

Run the following commands in the PX4 SITL console:

```bash
param set COM_ARM_WO_GPS 1
param set COM_RC_IN_MODE 4
param set NAV_DLL_ACT 0
param set NAV_RCL_ACT 0
param set COM_OBL_ACT 0
```

### 5. Run the Simulation

Requires three terminals. Start each in order and wait for it to initialize before proceeding.

**Terminal 1 - PX4 SITL + Gazebo**

Run PX4 SITL:

```bash
docker exec -it px4_sitl bash
cd /root/PX4-Autopilot
make px4_sitl gz_x500
```

You will see:
- Gazebo simulation with a quadcopter
- PX4 console showing startup messages
- Drone ready for commands!

**Terminal 2 - ROS Gazebo Bridge**

Run ROS Gazebo Bridge, below is an example on how to map relevant topics of depth camera from Gazebo to ROS:

```bash
docker exec -it px4_sitl bash
ros2 run ros_gz_bridge parameter_bridge
/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image
/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image
/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
--ros-args
-r /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info:=/camera/color/camera_info
-r /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera/color/image
-r /depth_camera:=/camera/depth/image
-r /depth_camera/points:=/camera/depth/points
```

**Terminal 3 - MAVROS**

Run MAVROS: 

```bash
docker exec -it px4_sitl bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557
```

### 6. Integrate a Local Workspace

To test a custom module in PX4 SITL, you can mount your local development workspace into the Docker container as a bind volume. This allows live access to your host-side code from within the simulation environment.

Add the following volume mapping to your `docker-compose.yml` file:

```yaml
    volumes:
      - /home/user/workspace:/root/workspace
```

This binds the host directory /home/user/workspace to /root/workspace inside the container, making your module directly available to PX4 SITL without needing to rebuild the image.

## Usage Examples

### Start/Stop

```bash
# Start (interactive)
docker-compose up

# Start (background)
docker-compose up -d

# Stop
docker-compose down

# Restart
docker-compose restart

# View logs
docker-compose logs -f
```

### Attach/Detach

```bash
# Attach to running container
docker attach px4_sitl

# Detach without stopping: Ctrl+P, Ctrl+Q

# Or use exec for new shell
docker exec -it px4_sitl bash
```

### Multiple Terminal Windows

```bash
# Start container
docker-compose up -d

# Terminal 1: Run PX4
docker exec -it px4_sitl bash
cd /root/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2: Monitor ROS topics
docker exec -it px4_sitl bash
ros2 topic list
ros2 topic echo /mavros/altitude

# Terminal 3: Build custom packages
docker exec -it px4_sitl bash
cd /root/ros2_ws
colcon build
```

### Try Different Vehicles

See full list of vehicles [here](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles).

```bash
# X500 Quadcopter
make px4_sitl gz_x500

# RC Cessna
make px4_sitl gz_rc_cessna

# Ackermann Rover
make px4_sitl gz_rover_ackermann
```

## Network Ports

| Port | Service |
|------|---------|
| 5901 | VNC server |
| 6080 | NoVNC web interface |
| 14550/udp | PX4 MAVLink (QGroundControl) |
| 18570/udp | Local UDP port used when PX4 communicates inside container/VM setups |

## Acknowledgement

- [PX4 Autopilot Documentation](https://docs.px4.io/)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/)
- [ROS Gazebo Bridge](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)
- [MAVROS](https://github.com/mavlink/mavros)
- [QGroundControl](http://qgroundcontrol.com)
- [TigerVNC Documentation](https://tigervnc.org/)


**Happy Simulating!** 🚁
