# TurtleBot 4 Auto Navi + Yolo Detect/Bounding Box

Project ROS 2 Humble cho bài thực hành tuần 5: localization bằng AMCL,
navigation tự động qua 3 goal và hiển thị bounding box vật thể trong môi
trường AWS RoboMaker Small House. Do tuần 5 cần bổ sung nhiều vật thể đa dạng
để camera nhận diện và vẽ bounding box, map maze của tuần 4 được thay thế bằng
map Small House.

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

## Chạy SLAM để tạo lại map Small House

Nếu muốn quét lại môi trường và lưu bản đồ mới, dừng các tiến trình cũ rồi
chạy Gazebo cùng SLAM Toolbox:

```bash
cd ~/tb4_autoNavi-yolo
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tb4_week5 small_house_mapping.launch.py \
  ros_domain_id:=68 \
  world:=warehouse \
  model:=standard \
  world_file:=$(pwd)/aws-robomaker-small-house-world/worlds/small_house.world \
  rviz:=true
```

Điều khiển robot để quét toàn bộ nhà, sau đó lưu map:

```bash
ros2 run nav2_map_server map_saver_cli -f tb4_sim/maps/small_house_slam
```

Sau khi lưu xong, dùng lại lệnh navigation ở trên để AMCL, Nav2 và nhận diện
vật thể hoạt động trên map Small House.

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

Robot không cần click `2D Pose Estimate` và không cần click `Nav2 Goal` trên
RViz. Gazebo chỉ spawn robot; AMCL vẫn thực hiện localization, còn node tuần 5
tự publish initial pose cho AMCL.

## Luồng nhận diện và vẽ bounding box

```text
Camera RGB `/oakd/rgb/preview/image_raw`
  + CameraInfo `/oakd/rgb/preview/camera_info`
  + Depth `/oakd/rgb/preview/depth`
  → `world_object_bbox_node`
  → bounding box và nhãn vật thể
```

Node `world_object_bbox_node` sử dụng ảnh RGB để hiển thị, CameraInfo để tính
phép chiếu hình học và Depth để kiểm tra vật thể.

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
- Navigation: `/navigate_to_pose`
- Undock: `/undock`
- Điều khiển: `/cmd_vel`

## Cấu trúc code chính

- `tb4_week5/tb4_week5/autonomous_nav_node.py`: initial pose, undock, lùi,
  tránh dock, recovery và gửi tuần tự 3 goal.
- `tb4_week5/tb4_week5/world_object_bbox_node.py`: đọc model từ world và vẽ
  bounding box theo camera.
- `tb4_week5/launch/week5_navigation_yolo.launch.py`: launch tổng.
- `tb4_week5/config/goals.yaml`: goal và tham số dock/recovery.
- `tb4_week5/rviz/small_house_slam.rviz`: cấu hình RViz cho Small House.
