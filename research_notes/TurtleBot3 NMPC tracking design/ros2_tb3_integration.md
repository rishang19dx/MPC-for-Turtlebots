# ROS 2 Humble + Gazebo Classic + TurtleBot3 Burger integration notes for MPC tracking with LiDAR obstacle avoidance

Scope: system integration (topics, launch, sim-to-real), sensing pipeline, hardware pitfalls. Researched 2026-11-17.
Note on sources: docs.ros.org blocked automated fetches (Anubis bot protection / DNS errors) during this session. Items marked **[PK]** come from prior knowledge of stable ROS conventions and APIs. The cited URL is the canonical location, but it was not re-fetched this session. Verify these against the docs before relying on them.

---

## 1. TB3 Burger ROS 2 Humble stack: bringup, topics, rates, networking, time sync

### Takeaway
On the Pi, run `export TURTLEBOT3_MODEL=burger; export LDS_MODEL=LDS-02` (or LDS-01), set `ROS_DOMAIN_ID=30` (the same on the laptop), then `ros2 launch turtlebot3_bringup robot.launch.py`. That gives `/cmd_vel`, `/odom`, `/scan`, `/imu`, `/joint_states`, `/tf`, and other topics. Odom/joint_states/IMU come from a 50 ms (20 Hz) OpenCR polling timer. The LiDAR runs at about 5 Hz with 1° resolution (360 beams). Treat 5 Hz scans and Wi-Fi DDS as the dominant constraints on the MPC loop.

### Cited Findings
**Bringup and environment**
- Bringup on the SBC: `ssh ubuntu@{IP}`, then `export TURTLEBOT3_MODEL=burger` and `ros2 launch turtlebot3_bringup robot.launch.py`. The expected log shows robot_state_publisher and turtlebot3_node starting, `/dev/ttyACM0` opening, a baudrate change, gyro calibration, and "Run!". — [ROBOTIS e-Manual Bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)
- `ROS_DOMAIN_ID` (example 30) and `RMW_IMPLEMENTATION` (example `rmw_fastrtps_cpp`) must match on the remote PC and the SBC. The Wi-Fi router must allow multicast for DDS discovery. — [e-Manual Bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)
- SBC setup:
  - `echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc` (or LDS-01 / LDS-03).
  - `echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc`. The manual warns: "Do not use an identical ROS_DOMAIN_ID to others in the same network". This matters in a shared lab where several students run TB3s.
  - Install the OpenCR udev rules: `sudo cp $(ros2 pkg prefix turtlebot3_bringup)/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`.
  - Configure Wi-Fi in `/etc/netplan/50-cloud-init.yaml`.
  - `systemctl mask systemd-networkd-wait-online.service` avoids a boot hang.
  - Source: [e-Manual SBC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)
- Topics after bringup: `/battery_state, /cmd_vel, /imu, /joint_states, /magnetic_field, /odom, /parameter_events, /robot_description, /rosout, /scan, /sensor_state, /tf, /tf_static`. Services: `/motor_power, /reset, /sound`. — [e-Manual Bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)

**cmd_vel message type (pitfall)**
- The e-Manual says that from Jazzy onward `/cmd_vel` is `TwistStamped`, switchable with `enable_stamped_cmd_vel: false`. — [e-Manual Bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)
- On the current `humble` branch, `turtlebot3_node` builds a `TwistSubscriber` with callbacks for both `geometry_msgs/Twist` and `TwistStamped`. Its cmd_vel QoS is `KeepLast(10)` with default (reliable) reliability. — [turtlebot3_node/src/turtlebot3.cpp (humble)](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/src/turtlebot3.cpp)
- `param/burger.yaml` on the humble branch, as fetched, contains `enable_stamped_cmd_vel: true`. This **conflicts** with the manual, which presents stamped cmd_vel as Jazzy-only. Older Humble apt releases are expected to use plain `Twist`. — [burger.yaml (humble branch)](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_bringup/param/burger.yaml)

**Robot parameters**
- Burger limits: max linear 0.22 m/s, max angular 2.84 rad/s. Size 138×178×192 mm, about 1 kg. Raspberry Pi 4 SBC, OpenCR (Cortex-M7), 2× DYNAMIXEL XL430-W250. Li-Po 11.1 V 1800 mAh, about 2.5 h. LDS-02 on current units. — [e-Manual Features](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)
- Kinematic parameters from burger.yaml: wheel `separation: 0.160` m, `radius: 0.033` m. Odometry settings: `publish_tf: true, use_imu: true, frame_id: odom, child_frame_id: base_footprint`. — [burger.yaml](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_bringup/param/burger.yaml)
- turtlebot3_node timers: sensor/joint-state publish at `publish_timer(50 ms)` (about 20 Hz), and a `heartbeat_timer(100 ms)` that writes an incrementing counter to an OpenCR register. **This file has no cmd_vel timeout.** — [turtlebot3.cpp](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/src/turtlebot3.cpp)

**LiDAR specs**
- LDS-02:
  - Scan rate "5 Hz or above" (may vary by unit); sampling rate fixed at 2.3 kHz, so angular resolution varies with spin rate.
  - Range 160–8000 mm; 1° resolution over 360°.
  - Accuracy ±10 mm (160–300 mm), ±3% (0.3–6 m), ±5% (6–8 m).
  - Source: [LDS-02 appendix](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_02/)
- LDS-01:
  - 300±10 rpm (5 Hz), 1.8 kHz sampling, range 120–3500 mm, 1° resolution.
  - Accuracy ±15 mm (120–499 mm), ±5% (0.5–3.5 m); precision ±10 mm / ±3.5%.
  - Source: [LDS-01 appendix](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/)

**Time sync**
- Neither the bringup page nor the SBC setup page covers time sync (chrony/NTP). — [Bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/), [SBC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)

**DDS over Wi-Fi**
- Wi-Fi bandwidth and latency vary unpredictably. Reliability problems over Wi-Fi are reported on Humble with both FastDDS and CycloneDDS. — [ROS Discourse: QoS reliability issue](https://discourse.ros.org/t/ros2-qos-reliability-issue/31700)
- CycloneDDS's default max message size is above the 1500 B MTU and broke large topics over Wi-Fi. Small topics (odom, tf, cmd_vel) were unaffected. The suggested fix is about 1470 B `MaxMessageSize`. — [hbrobotics thread](https://groups.google.com/g/hbrobotics/c/1NZXnFtMjmQ)
- One benchmark found CycloneDDS better over Ethernet and Zenoh better over Wi-Fi/4G. — [arXiv 2309.07496](https://arxiv.org/abs/2309.07496v2)

### Inferences
- **Effective feedback rates:** about 20 Hz for `/odom`, `/imu` and `/joint_states`, and about 5 Hz for `/scan`. Run the MPC at 10–20 Hz (dt = 0.05–0.1 s). Update obstacles at 5 Hz and propagate them between scans with odometry.
- **Range limits to use:** take `range_min`/`range_max` from each `LaserScan` message rather than hard-coding them. For a classroom arena, cap usable range at about 3.5 m for both sensors; that is also the Gazebo value.
- **Input bounds for the MPC:** |v| ≤ 0.22 m/s and |ω| ≤ 2.84 rad/s, both at the same time. Derating to v ≤ 0.2 and ω ≤ 1.5–2.0 is prudent because XL430 torque saturates near the limits.
- **Check the cmd_vel type first:** run `ros2 topic info -v /cmd_vel` on the real robot and publish the type it reports. Otherwise the robot silently ignores commands.
- **Networking checklist:**
  - Same `ROS_DOMAIN_ID` and RMW on both machines (FastDDS is the TB3 default).
  - Router multicast enabled; avoid campus/enterprise Wi-Fi with client isolation, and use a dedicated router or phone hotspot.
  - Verify with `ros2 multicast receive` on one machine and `ros2 multicast send` on the other, then `ros2 topic hz /scan` from the laptop.
  - If multicast is blocked, use a FastDDS discovery server (`fastdds discovery -i 0 -l <laptop_ip> -p 11811`, plus `export ROS_DISCOVERY_SERVER=<laptop_ip>:11811` on both) **[PK]**.
- **Time sync recipe [PK]:** `sudo apt install chrony` on both machines. Make the laptop an NTP server (`allow 192.168.x.0/24`, `local stratum 10` in `/etc/chrony/chrony.conf`) and point the Pi at it (`server <laptop_ip> iburst`). Check with `chronyc tracking`.
  - Without sync, TF lookups between Pi-stamped `/scan`/`/odom` and laptop `now()` raise "extrapolation into the future/past" errors. Any latency measured with `now() - msg.header.stamp` is also wrong.
  - Alternative: always look up TF at `msg.header.stamp`, never at `now()`.

### Gaps
- No ROBOTIS-published figure for the real `/odom` output rate. Code analysis gives at most about 20 Hz (50 ms timer). The odometry synchronizer's 75 ms joint-state lower bound suggests about 13 Hz when `use_imu: true`. Measure it with `ros2 topic hz /odom`.
- The LiDAR driver QoS (hls_lfcd_lds_driver / ld08_driver) was not verified from source.
- Whether the OpenCR firmware stops the motors when cmd_vel or the heartbeat stops was not verified.

---

## 2. Gazebo Classic (gazebo11) with turtlebot3_gazebo

### Takeaway
Launch with `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py x_pose:=.. y_pose:=..` (`use_sim_time` defaults to true). For batch runs, include only `gzserver.launch.py` (headless), spawn the robot and obstacles with `spawn_entity.py` or the `/spawn_entity` service, and add a `gazebo_ros_p3d` plugin to the model for ground truth. The simulated LDS is 5 Hz, 360 samples, 0.12–3.5 m, with Gaussian noise σ = 0.01 m. Diff-drive odometry runs at 30 Hz.

### Cited Findings
- `turtlebot3_world.launch.py` declares:
  - `use_sim_time` (default `'true'`), `x_pose` (default `'-2.0'`), `y_pose` (default `'-0.5'`).
  - It includes `gazebo_ros/launch/gzserver.launch.py` with `{'world': world}`, then `gzclient.launch.py`, then the TB3 `robot_state_publisher.launch.py` and `spawn_turtlebot3.launch.py` (which receives x_pose/y_pose).
  - Source: [turtlebot3_world.launch.py (humble)](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3_simulations/humble/turtlebot3_gazebo/launch/turtlebot3_world.launch.py)
- Simulated Burger model (`model.sdf`):
  - LiDAR `hls_lfcd_lds`: `update_rate 5`, 360 samples, angle 0 to 6.28, range 0.12–3.5 m, resolution 0.015, Gaussian noise mean 0, stddev 0.01.
  - IMU: 200 Hz, gyro σ = 2e-4, accel σ = 1.7e-2.
  - `libgazebo_ros_diff_drive.so`: update_rate 30, wheel_separation 0.160, wheel_diameter 0.066, max_wheel_torque 20, max_wheel_acceleration 1.0, publish_odom/publish_odom_tf true, odom frame `odom`, base frame `base_footprint`, topics `cmd_vel`/`odom`.
  - Joint state publisher at 30 Hz.
  - Source: [turtlebot3_burger/model.sdf (humble)](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3_simulations/humble/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf)
- Ground truth with the `gazebo_ros_p3d` plugin (ROS 2 form):

```xml
<plugin name="gazebo_ros_p3d" filename="libgazebo_ros_p3d.so">
  <ros><namespace>/</namespace><remapping>odom:=ground_truth</remapping></ros>
  <body_name>base_footprint</body_name><frame_name>world</frame_name>
  <update_rate>50</update_rate><xyz_offset>0 0 0</xyz_offset>
  <rpy_offset>0 0 0</rpy_offset><gaussian_noise>0.0</gaussian_noise>
</plugin>
```

  - In ROS 2 the tag is `xyz_offset` (`xyz_offsets` only before Galactic).
  - The output topic defaults to `odom`, so a remap is **needed** to avoid colliding with diff-drive `/odom`.
  - The output type is `nav_msgs/Odometry` **[PK]**.
  - Source: [gazebo_ros_pkgs wiki: ROS 2 Migration P3D](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-P3D)

### Inferences
- **Initial yaw:** `spawn_turtlebot3.launch.py` exposes only x/y. For a yaw, call `spawn_entity.py` directly **[PK]**: `ros2 run gazebo_ros spawn_entity.py -entity burger -file $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf -x 0 -y 0 -z 0.01 -Y 1.57`.
  - Better: copy `turtlebot3_world.launch.py` into your own package and add `yaw` and `world` args.
  - Note that `burger` is a **base_footprint** link in the model. Use the `base_footprint` link as the p3d `body_name`.
- **Custom worlds [PK]:** copy `worlds/empty_world.world` and add `<include><uri>model://...</uri><pose>x y 0 0 0 0</pose></include>` entries or inline `<model>` cylinders (static `<static>true</static>`, `<cylinder><radius>0.1</radius><length>0.5</length>`).
  - Cylinders match a circle-obstacle MPC constraint exactly. They also give the LiDAR clean arcs to fit.
  - Install worlds via `data_files` in setup.py and pass the path through `gzserver.launch.py world:=`.
- **Programmatic randomization [PK]:**
  - The `/spawn_entity` service (`gazebo_msgs/srv/SpawnEntity`: `name`, `xml` (SDF string), `robot_namespace`, `initial_pose`, `reference_frame`) and `/delete_entity` (`gazebo_msgs/srv/DeleteEntity`) are provided by the `gazebo_ros_factory` system plugin. `gzserver.launch.py` loads it by default.
  - To move entities between trials, add `<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"><ros><namespace>/gazebo</namespace></ros></plugin>` to the world. That exposes `/gazebo/set_entity_state` and `/gazebo/get_entity_state`.
  - `/reset_world` and `/reset_simulation` (std_srvs/Empty) reset poses and time.
  - Seed a numpy RNG per trial, and reject samples that overlap the start, the goal, or each other.
- **Headless / speed [PK]:**
  - Skip `gzclient` (`gazebo.launch.py gui:=false`, or include only `gzserver.launch.py`). gzserver also accepts `verbose:=true`, `pause:=true`, and `server_required:=true` (shuts down the launch when the server exits).
  - Real-time factor comes from the world `<physics>`: `<max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate>`, which targets RTF 1.0. Set `real_time_update_rate` to 0 to run as fast as possible.
  - **All nodes must use `use_sim_time:=true`** so timers and TF follow `/clock`. rclpy timers then tick in sim time automatically.
  - For fair MPC benchmarks, keep RTF ≤ 1, or make the MPC compute deadline sim-time based. Otherwise solver latency is under-represented.
- **Sim-to-real fidelity gaps:** the Gazebo LDS is 0.12–3.5 m with σ = 1 cm Gaussian noise and no dropouts. The real LDS-02 is 0.16–8 m with ±3% error, plus dropouts and reflective/black-surface failures. Inject noise and dropouts, and add an artificial actuation delay, to make sim results transferable.

### Gaps
- `spawn_entity.py` flags, `gazebo_ros_state` usage and the `gui:=false` arg were not re-verified from source this session (docs.ros.org blocked). They are standard gazebo_ros_pkgs ROS 2 APIs.
- No measured RTF numbers for TB3 worlds on typical laptops were found.

---

## 3. Odometry quality on the real TB3; ground truth without mocap

### Takeaway
TB3 `/odom` already fuses the IMU for **heading only**: with `use_imu: true`, yaw increments come from the IMU quaternion and translation comes from the wheels with midpoint integration. The published covariances are left at default (zeros). A separate robot_localization EKF adds little. Ground truth on hardware comes from tape-marked start and end poses. In sim it comes from p3d.

### Cited Findings
- Odometry (turtlebot3_node `odometry.cpp`):
  - Position always comes from the wheel encoders: `delta_s = r*(wheel_r + wheel_l)/2`.
  - With `use_imu = true`, heading change is `delta_theta = remainder(theta_imu - last_theta, 2π)`, using IMU yaw from `atan2` of the quaternion. With `use_imu = false`, it is `r*(wheel_r - wheel_l)/separation`.
  - Pose update uses midpoint heading: `x += delta_s*cos(theta + delta_theta/2)`.
  - Twist: `v = delta_s/dt`, `w = delta_theta/dt`, with y velocity always 0.
  - Covariance assignments are commented out (`TODO`: needs more accurate values), so covariances are default.
  - Publication is message-driven. With the IMU it uses a synchronizer with inter-message lower bounds of 75 ms (joint states) and 15 ms (IMU).
  - The odometry reset service zeroes the pose.
  - Source: [odometry.cpp (humble)](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/src/odometry.cpp)
- burger.yaml enables `use_imu: true` and `publish_tf: true` (odom→base_footprint). — [burger.yaml](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_bringup/param/burger.yaml)
- At bringup the robot performs gyro calibration ("Calibrate Gyro" in the expected output), so it must stay still at startup. — [e-Manual Bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)

### Inferences
- **Heading drift:** heading drift is limited by gyro bias drift, which is calibrated at boot. Translational error comes from wheel radius error, tire compression and slip.
  - The classic effect: a 1% radius error gives about 1 cm per metre of along-track error.
  - Scale and wheel-base errors grow with distance; heading error turns into lateral error that grows with distance squared.
  - For short (< 5 m) class trajectories on a smooth floor, expect cm-level position drift. Measure it rather than assume it.
- **Calibration:** do a UMBmark-style square or straight-line test, and adjust `wheels.radius`/`separation` in a copied burger.yaml if needed. Calibrate the wheel radius by driving 2 m straight and comparing against a tape measure.
- **robot_localization EKF [PK]** (`ros-humble-robot-localization`):
  - Only worthwhile if you set `use_imu: false` (wheel-only odom) and then fuse wheel twist (vx, vyaw) with IMU vyaw. Otherwise you double-count the IMU.
  - If you do it, set TB3 `publish_tf: false` so that only the EKF publishes odom→base_footprint.
  - For a course project, the TB3 built-in fusion is adequate; document this choice.
- **Avoiding feedback-loop problems:**
  - Reset odom at the start of each run: restart bringup, or record the initial odom pose and express everything relative to it.
  - Hold the robot still for the 1–2 s gyro calibration.
  - Avoid acceleration jumps. XL430 `profile_acceleration` is 0 (unlimited) in burger.yaml, so step changes in cmd_vel cause wheel slip. Add Δu rate limits in the MPC cost or constraints.
- **Ground truth without mocap:**
  1. Tape a start pose (x, y, θ) on the floor with a heading line. Tape waypoints/targets, then measure final position error with a tape measure, and final heading against a taped line or with a protractor.
  2. Record `/scan` in a rosbag. Afterwards, run `slam_toolbox` or scan-matching on the bag to get a LiDAR-corrected trajectory as pseudo ground truth.
  3. Use a phone camera overhead with an ArUco marker on the robot for a cheap trajectory track (outside scope; see the perception notes).
  4. In sim, the p3d `ground_truth` topic gives exact pose. Report both odom-tracking error and true-tracking error.

### Gaps
- No quantitative published drift numbers (m per m, or deg per min) for TB3 Burger on Humble were found in this session.

---

## 4. LiDAR obstacle extraction (LaserScan → circles in odom frame)

### Takeaway
No maintained full Humble port of tysik `obstacle_detector` (circles plus segments) was found. `laser_segmentation` (ajtudela) is released for Humble and does jump-distance / adaptive-threshold segmentation. Simplest robust path: write your own about-80-line rclpy node that converts polar to Cartesian, segments by adaptive breakpoint, fits circles to the segments, transforms the centres to `odom` with tf2, and keeps the K nearest. At 5 Hz with 360 beams, compute is negligible.

### Cited Findings
- tysik `obstacle_detector`:
  - A ROS (1) package that detects and tracks obstacles from 2D laser data as line segments or circles.
  - Needs Armadillo; last updated about 2 years ago.
  - Has no Humble port. A ROS 2 Humble repo exists, but it ports only the `scan_merger` node of the jk-ethz fork.
  - Sources: [relatedrepos tysik/obstacle_detector](https://relatedrepos.com/gh/tysik/obstacle_detector), [romb-technologies fork README](https://github.com/romb-technologies/obstacle_detector/blob/master/README.md)
- `laser_segmentation` (ajtudela):
  - Released for Humble (index lists 3.0.2; humble branch).
  - Jump-distance clustering starts a new segment when the Euclidean distance between adjacent points exceeds a threshold (default `distance_threshold` 0.3 m).
  - Newer docs add dynamic thresholds via `method_threshold`: Lee 2001, Dietmayer 2001, Santos 2003. Humble support for this parameter is unconfirmed.
  - Sources: [docs.ros.org humble laser_segmentation](https://docs.ros.org/en/humble/p/laser_segmentation), [index.ros.org](https://index.ros.org/p/laser_segmentation/)
- Nav2 `collision_monitor` is available for Humble. It is a velocity-gating safety layer with polygon zones, not an obstacle extractor. — [index.ros.org nav2_collision_monitor](https://index.ros.org/p/nav2_collision_monitor/)
- The Gazebo LDS gives 360 beams from 0 to 6.28 rad at 5 Hz. — [model.sdf](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3_simulations/humble/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf)

### Inferences
Recommended pipeline (own node):
1. **Validity mask:** keep beams where `isfinite(r) and range_min < r < min(range_max, R_cut)`, with `R_cut` about 2.5–3 m.
   - Real LDS drivers commonly report invalid returns as 0.0, while Gazebo reports `inf`. Handle both [PK; verify on your unit].
   - Also mask beams hitting the robot's own body if any appear (Burger's LiDAR is on top, so usually none).
2. **Polar to Cartesian** in the `base_scan` frame: θ_i = angle_min + i·angle_increment.
3. **Adaptive breakpoint detector** (Borges & Aldon 2004):
   - D_max = r_{i-1}·sin(Δφ)/sin(λ − Δφ) + 3σ_r, with λ ≈ 10–15° and σ_r ≈ 0.01–0.03 m.
   - Δφ = 1° is about 0.0175 rad. At 2 m, adjacent beams are about 3.5 cm apart, so D_max is roughly 0.15–0.2 m.
   - Merge the wrap-around segment (beam 359 to beam 0).
   - Drop segments with fewer than 3 points as noise.
4. **Circle fit per segment:** algebraic least-squares (Kåsa) fit, with radius clamped to [r_min, r_max] (for example 0.05–0.5 m).
   - Fallback when the fit is ill-conditioned (nearly straight arc, few points): centroid plus half the chord length, pushed away from the sensor by r.
   - Inflate: r_obs = r_fit + r_robot (Burger footprint radius ≈ 0.105 m) + safety margin (about 0.05–0.1 m).
   - Long straight segments such as walls can be split into several circles of chord ≤ 2r_max, or handled as half-planes.
5. **DBSCAN alternative** (`sklearn.cluster.DBSCAN(eps≈0.1–0.15, min_samples=3)`) works but is O(n log n) with an extra dependency. The breakpoint method exploits scan ordering and is simpler.
6. **Transform to `odom` [PK]:**
   - `tf_buffer.lookup_transform('odom', scan.header.frame_id, scan.header.stamp, timeout=Duration(seconds=0.05))`, using `tf2_ros.Buffer` and `TransformListener`.
   - Apply it to the centres manually (2D rotation plus translation from the quaternion yaw).
   - Use the scan stamp, **not** `Time()`/now. This compensates the motion between scan time and control time. Clock sync matters here (section 1).
   - On the TB3 the LiDAR frame is `base_scan`, a child of `base_link` via a fixed transform from `robot_state_publisher` [PK].
7. **Fixed-size list:** sort by distance to the robot and keep the nearest K (for example K = 5–8).
   - Pad with dummy obstacles far away (for example at (1e3, 1e3) with r = 0). The NMPC parameter vector then keeps a constant size, so there is no CasADi rebuild or re-JIT.
8. **Temporal filtering:** associate across scans with nearest-neighbour gating (about 0.2 m) and average or low-pass the centres. Hold the last obstacles up to about 0.5 s if a scan is missed.
   - The obstacles are static in the odom frame, so between the 5 Hz scans the MPC reuses the odom-frame list while the robot pose updates at 20 Hz.
- `laser_line_extraction` (ROS 1, line segments): no Humble release was confirmed this session. `laser_segmentation` is the Humble-native alternative.

### Gaps
- No maintained Humble fork of tysik obstacle_detector with circle extraction was confirmed (the jk-ethz fork branches were not checked).
- The real LDS-02 driver's invalid-range encoding (0 vs inf) was not verified from source.

---

## 5. Latency/delay, delay compensation, rclpy control-loop timing, QoS

### Takeaway
The total loop delay is the sum of:
- Wi-Fi DDS hops (Pi→laptop for odom/scan, and laptop→Pi for cmd_vel), typically a few ms to tens of ms, with spikes.
- Odom sampling at 50 ms.
- Solver time.
- OpenCR/motor response.

Budget about 0.1–0.15 s. Compensate by forward-simulating the unicycle model over the delay with the stored past inputs, and measure the actual delay rather than assuming it. Use best-effort `qos_profile_sensor_data` for `/scan`, reliable depth-1 for `/cmd_vel`, and a single timer-driven control loop in its own callback group.

### Cited Findings
- Wi-Fi latency and bandwidth vary unpredictably, and reliable QoS over Wi-Fi can cause stalls. — [ROS Discourse QoS reliability](https://discourse.ros.org/t/ros2-qos-reliability-issue/31700)
- A Discourse post describes "ghost commands" on bad Wi-Fi. ros2_controllers' mobile-base controllers discard cmd_vel older than their timeout and output zero. This motivates timestamped commands and command guards. — [ROS Discourse: ros2_kinematic_guard](https://discourse.openrobotics.org/t/stop-ghost-commands-on-bad-wi-fi-ros2-kinematic-guard-for-cmd-vel-safety/54681)
- A Discourse thread discusses handling high communication latency (~360 ms) in ROS 2 control. — [ROS Discourse: high latency](https://discourse.openrobotics.org/t/handling-high-hardware-communication-latency-360ms/54754)
- TB3 cmd_vel subscription uses `KeepLast(10)`, default reliability. — [turtlebot3.cpp](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/src/turtlebot3.cpp)
- An older ROS 1 TB3 (Kinetic, Pi 3) report showed large cmd_vel delays that depended on where the master ran. — [ROS Answers 289221](https://answers.ros.org/question/289221/delay-problem-with-cmd_vel-turtlebot3/?sort=votes)

### Inferences
- **Measure the delay:**
  - (a) Sync clocks with chrony, then log `now() - msg.header.stamp` for `/odom` and `/scan` on the laptop.
  - (b) Measure end-to-end actuation delay with a step test. Publish a v step, log the time until `/odom` twist responds, and fit dead time plus a first-order lag.
  - Expected: Wi-Fi one-way delay of several ms (median) with 50–200 ms spikes on congested networks.
- **Delay compensation in the MPC:**
  - Keep a deque of the last ⌈τ/dt⌉ applied inputs with timestamps.
  - On each tick, take the latest odom pose x̂(t_odom) and integrate the unicycle model from t_odom to t_now + τ_act using the inputs actually sent over that interval (RK4 or exact arc integration).
  - Solve the MPC from that predicted state. Optionally fix the first d control moves to the already-committed inputs.
  - With dt = 0.1 s and τ ≈ 0.1–0.15 s, d = 1–2 steps.
- **Running the MPC on the laptop or the Pi:**
  - Running on the laptop minimises solver time but adds two Wi-Fi hops.
  - Running on the Pi 4 removes the network from the loop. CasADi/IPOPT on a Pi 4 may take 20–100 ms for N = 20 with several obstacles (unverified), so test both.
  - A hybrid keeps a minimal watchdog node on the Pi.
- **rclpy timing [PK]:**
  - Use `self.create_timer(dt, self.control_cb, callback_group=MutuallyExclusiveCallbackGroup())`.
  - Put subscriptions in a different group (or a separate MutuallyExclusive group each), and spin with `MultiThreadedExecutor(num_threads=3)` so a long solve doesn't block odom/scan callbacks.
  - Subscriptions only copy the latest message into member variables under a `threading.Lock`; never solve inside a subscriber.
  - The rclpy executor plus the GIL limits true parallelism, but IPOPT releases the GIL in C code to some extent.
  - Log `solve_time` every tick and warn if it exceeds dt. Warm-start from the shifted previous solution.
  - The ROS 2 Humble guide on callback groups explains why a callback group that is Mutually Exclusive with a waiting callback deadlocks synchronous calls: docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html (blocked during this session).
- **QoS [PK]:**
  - `/scan`: subscribe with `rclpy.qos.qos_profile_sensor_data` (BEST_EFFORT, KEEP_LAST 5, VOLATILE). A best-effort subscriber is compatible with both reliable and best-effort publishers, while a reliable subscriber to a best-effort publisher (as the Gazebo ray plugin typically publishes) gets **no data**. Check with `ros2 topic info -v /scan`.
  - `/odom`: depth 1–5. Reliable is fine at this rate, but best-effort avoids head-of-line blocking on Wi-Fi.
  - `/cmd_vel`: reliable, depth 1 (KeepLast(1)), so stale commands are not queued.

### Gaps
- No primary-source measurement of TB3 + Humble Wi-Fi latency distributions was found. Plan to measure it.
- Pi 4 NMPC solve times were not found for this exact setup.

---

## 6. Safety: velocity watchdog, e-stop, zero cmd_vel on shutdown

### Takeaway
Don't rely on TB3 firmware for command timeouts. turtlebot3_node has only a 100 ms heartbeat to OpenCR and no cmd_vel timeout. Build a small safety layer: saturation, a stale-state watchdog, a LiDAR proximity stop, a manual e-stop topic, and guaranteed zero-velocity publishing on shutdown.

### Cited Findings
- turtlebot3_node: there is no cmd_vel timeout in the node; the only watchdog-like mechanism is a 100 ms heartbeat counter written to OpenCR. — [turtlebot3.cpp](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_node/src/turtlebot3.cpp)
- `/motor_power` service exists on the real robot. — [e-Manual Bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)
- Nav2 collision_monitor (Humble) can gate velocity by polygon zones as an off-the-shelf safety layer. — [nav2_collision_monitor](https://index.ros.org/p/nav2_collision_monitor/)

### Inferences
Recommended design:
- **Saturate in the publisher:** clip v to [−0.22, 0.22] (or the derated limit) and ω to [−2.84, 2.84], and rate-limit both, even though the MPC already constrains them.
- **Watchdog in the control node:**
  - If the last `/odom` is older than about 0.3 s or the last `/scan` older than about 0.5 s, publish zero and skip solving.
  - If the solver fails or returns infeasible, publish zero (or a braking fallback) and log it.
- **Proximity stop:** if min(valid ranges in the forward ±30° sector) < 0.15–0.2 m, publish zero for forward motion.
- **E-stop:** subscribe to `/estop` (std_msgs/Bool). Keep a teleop terminal ready (`ros2 run turtlebot3_teleop teleop_keyboard`; the space bar stops). Also keep the physical power switch reachable.
  - Optional hardware cut: `ros2 service call /motor_power std_srvs/srv/SetBool "{data: false}"`. The type was not verified this session.
- **Robot-side guard:** if the MPC runs on the laptop, run a tiny relay node on the Pi that forwards `/cmd_vel_safe` to `/cmd_vel` and publishes zero if no command arrives within about 0.3 s. That guards against Wi-Fi drop-outs, which a laptop-side node cannot do.
- **Zero on shutdown (rclpy) [PK]:**
  ```python
  try:
      executor.spin()
  except (KeyboardInterrupt, ExternalShutdownException):
      pass
  finally:
      for _ in range(3):
          node.cmd_pub.publish(Twist())   # zero; publish BEFORE rclpy.shutdown()
      node.destroy_node(); rclpy.try_shutdown()
  ```
  On Humble, SIGINT may invalidate the context before `finally` runs. As a backup, publish zero from a `signal`/`atexit` handler or use the Pi-side relay timeout. If the robot keeps moving after Ctrl-C, run `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`.

### Gaps
- Whether the OpenCR firmware stops the motors on heartbeat loss was not verified.

---

## 7. Data logging with rosbag2

### Takeaway
Record raw inputs and outputs plus controller internals on the laptop, then post-process in Python into plots. Publish MPC internals (reference, predicted horizon, solve time, cost) as topics so they land in the same bag with consistent stamps.

### Cited Findings
- The topics available to record on the real robot are as listed in section 1. — [e-Manual Bringup](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)

### Inferences [PK]
- **Record:** `ros2 bag record -o runs/$(date +%F_%H%M%S)_trial01 /odom /scan /cmd_vel /imu /tf /tf_static /mpc/reference /mpc/predicted_path /mpc/obstacles /mpc/diagnostics`. In sim, also record `/ground_truth` and `/clock`.
  - Humble's default storage is sqlite3. MCAP is available via `sudo apt install ros-humble-rosbag2-storage-mcap` and `-s mcap`, and opens in Foxglove.
- **Inspect:** `ros2 bag info <dir>`. Replay with `ros2 bag play <dir> --clock` and `use_sim_time:=true`, so the obstacle node can be re-tuned offline on recorded scans. This deterministic replay is very useful.
- **Read in Python** via `rosbag2_py.SequentialReader` plus `rclpy.serialization.deserialize_message` and `rosidl_runtime_py.utilities.get_message`. Convert to pandas/numpy and plot x–y paths vs the reference, tracking error vs time, v/ω vs bounds, min obstacle distance, and solve time.
- **Alternative:** have the controller node write a CSV row per tick (t, x, y, θ, x_ref, y_ref, v, ω, solve_ms, min_clearance). Simple and robust for the report plots.
- **Diagnostics message:** custom messages need an ament_cmake interface package. To stay pure-Python, use `std_msgs/Float64MultiArray` with a documented layout, or `diagnostic_msgs/DiagnosticArray`. For paths use `nav_msgs/Path`; for obstacles use `visualization_msgs/MarkerArray` (visible in RViz).

### Gaps
- None critical. The rosbag2 CLI options above should be checked against `ros2 bag record -h` on the installed version.

---

## 8. Package layout and conventions (ament_python, REP-103/105)

### Takeaway
Use a single ament_python package (for example `tb3_nmpc`) with separate nodes for the controller, obstacle extractor, reference publisher and safety relay. Put launch files for sim and real behind a `use_sim` switch, YAML params per platform, and pytest unit tests for the pure-Python core (model, MPC, clustering) that have no ROS dependency. Follow REP-103 (SI units, x forward / y left / z up, right-handed, yaw CCW positive) and REP-105 (`map → odom → base_footprint/base_link`; `odom` is continuous but drifts).

### Cited Findings
- TB3 odom frames: `odom` → `base_footprint` (burger.yaml; Gazebo diff_drive uses the same frames). — [burger.yaml](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_bringup/param/burger.yaml), [model.sdf](https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3_simulations/humble/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf)
- REP-103 [PK]: metres, radians, seconds, m/s, rad/s; body frame x forward, y left, z up. — [REP-103](https://www.ros.org/reps/rep-0103.html)
- REP-105 [PK]: `odom` is a world-fixed frame in which pose drifts but is continuous, which suits local control. `map` is globally accurate but can jump. — [REP-105](https://www.ros.org/reps/rep-0105.html)
- Prior TB3 MPC projects for reference:
  - lenhatquang2512/MPC_PID_ROS2_Control: C++, ROS 2 Foxy, TB3 with obstacle avoidance. — [relatedrepos](https://relatedrepos.com/gh/lenhatquang2512/MPC_PID_ROS2_Control)
  - lacie-life/TurtleBot3-MPC. — [relatedrepos](https://relatedrepos.com/gh/lacie-life/TurtleBot3-MPC)
  - astomodynamics/casadi_mpc: CasADi MPC node in ROS 2. — [relatedrepos](https://relatedrepos.com/gh/astomodynamics/casadi_mpc)
  - prox_mpc: ProxQP MPC in a Nav2 controller_server with a TB3 in Gazebo Harmonic. — [index.ros.org prox_mpc_controller](https://index.ros.org/p/prox_mpc_controller/)
  - IEEE paper "Trajectory Tracking and Obstacle Avoidance with Turtlebot 3 Burger and ROS 2" (MPC with CBF). — [IEEE Xplore 10349744](https://ieeexplore.ieee.org/document/10349744)
  - GMPC paper: TB3 tracking comparing geometric MPC (qpOASES) against NMPC (IPOPT). — [arXiv 2403.07317](https://arxiv.org/pdf/2403.07317)

### Inferences
Recommended layout [PK for ament_python mechanics]:
```
tb3_nmpc/
  package.xml            # exec_depend: rclpy, geometry_msgs, nav_msgs, sensor_msgs, visualization_msgs,
                         #   tf2_ros, std_msgs, std_srvs, gazebo_msgs (sim), python3-numpy, (casadi via pip)
  setup.py               # entry_points console_scripts; data_files for launch/, config/, worlds/, models/
  setup.cfg              # [develop] script_dir=$base/lib/tb3_nmpc ; [install] install_scripts=$base/lib/tb3_nmpc
  resource/tb3_nmpc
  tb3_nmpc/
    core/                # NO ROS imports -> unit-testable
      unicycle.py        #   model, RK4, delay predictor
      nmpc.py            #   CasADi Opti/nlpsol builder, warm start, fixed-K obstacle params
      obstacles.py       #   breakpoint segmentation, circle fit, K-nearest padding
      references.py      #   figure-8 / circle / lemniscate generators
    nodes/
      mpc_node.py        # timer loop, delay comp, publishes /cmd_vel + diagnostics
      obstacle_node.py   # /scan -> /mpc/obstacles (odom frame)
      safety_relay.py    # runs on Pi: timeout + clipping
      obstacle_spawner.py# sim only: randomized /spawn_entity
  launch/
    sim.launch.py        # gzserver(+gzclient opt) + spawn(x,y,yaw) + nodes, use_sim_time:=true
    real.launch.py       # nodes only (bringup runs on Pi), use_sim_time:=false
  config/ mpc_sim.yaml, mpc_real.yaml   # dt, N, Q, R, v/w bounds, delay, K, inflation
  worlds/ arena_cylinders.world
  test/ test_unicycle.py, test_obstacles.py, test_nmpc_feasibility.py, test_flake8.py, test_pep257.py
```
- Build and run: `colcon build --symlink-install --packages-select tb3_nmpc && source install/setup.bash`, then `ros2 launch tb3_nmpc sim.launch.py x:=0 y:=0 yaw:=0 headless:=true`.
- Install `casadi` via pip in the user environment: `pip install casadi`. No rosdep key may exist, so document it in the README.
- On the Pi, install only `safety_relay`, unless the MPC runs onboard.
- Load parameters from YAML with `declare_parameter`. Pass `use_sim_time` to every node from the launch file.
- Run tests with `colcon test --packages-select tb3_nmpc && colcon test-result --verbose`.
- Frame and unit rules:
  - All MPC states in the `odom` frame, yaw unwrapped consistently (wrap the heading error with `atan2(sin, cos)`).
  - Inputs (v, ω) in `base_footprint`, SI units.
  - Reference trajectories defined in `odom` relative to the odom pose at start.

### Gaps
- None of the GitHub MPC projects listed was reviewed in depth. The listings do not confirm their Humble compatibility or solver choices beyond what is noted.
