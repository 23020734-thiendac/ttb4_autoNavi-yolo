# TurtleBot 4 – Week 5 Small House

Project ROS 2 Humble cho bài thực hành tuần 5: localization bằng AMCL,
navigation tự động qua 3 goal và hiển thị bounding box vật thể trong môi
trường AWS RoboMaker Small House.

## Nội dung repository

Repository này chỉ dùng map Small House, không dùng map maze:

- `tb4_sim/maps/small_house_slam.yaml`
- `tb4_sim/maps/small_house_slam.pgm`
- `tb4_week5/`: node navigation, launch, goals và cấu hình RViz
- `aws-robomaker-small-house-world/worlds/small_house.world`
- `aws-robomaker-small-house-world/models/`: model nội thất cho Gazebo

Các map maze cũ, thư mục `build/`, `install/`, `log/`, `__pycache__/`, dataset
và kết quả train không được đưa vào repository.

## Yêu cầu

- Ubuntu 22.04
- ROS 2 Humble
- TurtleBot 4 Simulation / Gazebo Sim
- Các package:

```bash
sudo apt update
sudo apt install \
  ros-humble-turtlebot4-navigation \
  ros-humble-nav2-simple-commander \
  ros-humble-cv-bridge
```

## Build

```bash
cd ~/tb4_autoNavi-yolo
source /opt/ros/humble/setup.bash

PYTHONNOUSERSITE=1 colcon build \
  --symlink-install \
  --base-paths . \
  --packages-select tb4_sim tb4_week5

source install/setup.bash
unset PYTHONNOUSERSITE
```

## Chạy Small House, localization và navigation 3 goal

Trước khi chạy, dọn các process cũ:

```bash
pkill -f 'ros2 launch tb4_week5|week5_autonomous_nav|week5_yolo_detector|week5_world_object_bbox|rviz2|ign gazebo|gzserver|gzclient' 2>/dev/null || true
```

Sau đó chạy:

```bash
cd ~/tb4_autoNavi-yolo
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tb4_week5 week5_navigation_yolo.launch.py \
  ros_domain_id:=68 \
  world:=warehouse \
  model:=standard \
  map:=$(pwd)/tb4_sim/maps/small_house_slam.yaml \
  world_file:=$(pwd)/aws-robomaker-small-house-world/worlds/small_house.world \
  goal_count:=3 \
  auto_undock:=true \
  run_navigation:=true \
  run_yolo:=true \
  show_window:=true \
  max_bbox_area_ratio:=0.60
```

`run_yolo:=true` được giữ lại để tương thích với lệnh cũ. Phiên bản code hiện
tại chạy `world_object_bbox_node`: đọc tên và pose model từ
`small_house.world`, kết hợp TF/camera/depth để vẽ bounding box. Các nhãn hiển
thị gồm `chair`, `chandelier`, `ball`, `bed`, `coffee_table`, `door`, `tablet`,
`fitness_equipment`, `sofa`, `trash_bin` và `tv`.

## Luồng hoạt động

```text
Gazebo + map Small House
  → LiDAR/Odometry
  → AMCL tự nhận initial pose
  → autonomous_nav_node
  → undock và lùi khỏi dock
  → Nav2
  → Goal 1 → Goal 2 → Goal 3
```

Camera chạy song song:

```text
Camera RGB + CameraInfo + Depth
  → world_object_bbox_node
  → world model + TF projection
  → bounding box + nhãn + terminal log
```

Robot không cần click `2D Pose Estimate` và không cần click `Nav2 Goal` trên
RViz. Gazebo chỉ spawn robot; AMCL vẫn thực hiện localization, còn node tuần 5
tự publish initial pose cho AMCL.

## Ba goal mặc định

Các goal nằm trong `tb4_week5/config/goals.yaml`:

| Goal | Tọa độ `[x, y, yaw]` trong frame `map` |
|---|---|
| Goal 1 | `[5.0, -3.0, 0.0]` |
| Goal 2 | `[-3.0, -2.0, 3.14]` |
| Goal 3 | `[0.0, 3.0, 1.57]` |

Có thể chỉnh goal trực tiếp trong file YAML, nhưng cần bảo đảm tọa độ nằm trên
vùng trống của `small_house_slam`.

## Topic và action chính

- Map: `/map`
- LiDAR: `/scan`
- Pose AMCL: `/amcl_pose`
- Camera RGB: `/oakd/rgb/preview/image_raw`
- Camera info: `/oakd/rgb/preview/camera_info`
- Depth: `/oakd/rgb/preview/depth`
- Ảnh đã vẽ box: `/tb4_week5/yolo/detections_image`
- Navigation: `/navigate_to_pose`
- Undock: `/undock`
- Điều khiển: `/cmd_vel`

## Kiểm tra nhanh

```bash
ros2 node list | grep -E 'week5|amcl|planner|controller'
ros2 topic echo /amcl_pose --once
ros2 topic list | grep -E 'map|scan|camera|tf'
```

Nếu RobotModel bị đỏ, kiểm tra robot đã spawn, `/robot_description`, `/tf`,
`/scan` và `/amcl_pose`. Không click 2D Pose Estimate trước khi kiểm tra log của
`week5_autonomous_nav`.

## Cấu trúc code chính

- `tb4_week5/tb4_week5/autonomous_nav_node.py`: initial pose, undock, lùi,
  tránh dock, recovery và gửi tuần tự 3 goal.
- `tb4_week5/tb4_week5/world_object_bbox_node.py`: đọc model từ world và vẽ
  bounding box theo camera.
- `tb4_week5/launch/week5_navigation_yolo.launch.py`: launch tổng.
- `tb4_week5/config/goals.yaml`: goal và tham số dock/recovery.
- `tb4_week5/rviz/small_house_slam.rviz`: cấu hình RViz cho Small House.
