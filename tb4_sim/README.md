# TurtleBot 4 Simulation, SLAM và Nav2

Package ROS 2 Humble để chạy TurtleBot 4 trong Gazebo, tạo bản đồ bằng SLAM Toolbox, rồi dùng bản đồ đó cho localization và autonomous navigation.

Simulator, controller, SLAM Toolbox và Nav2 được gọi từ các package ROS 2 TurtleBot 4 đã cài. Các launch trong package này là launch wrapper cho quy trình của bài tập.

## Yêu cầu

- Ubuntu 22.04 và ROS 2 Humble Desktop.
- Các package `turtlebot4_ignition_bringup`, `turtlebot4_navigation`, `slam_toolbox`, `nav2_bringup`.
- Gazebo Sim hỗ trợ hiển thị đồ họa.

## Build package

Mở terminal tại workspace `/home/vtd/ros2_ws`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select tb4_sim --symlink-install
source install/setup.bash
```

Package có overlay cục bộ cho `irobot_create_description`, đặt renderer của
plugin Sensors thành `ogre2` để GPU LiDAR trong Gazebo Fortress trả dữ liệu
khoảng cách. Build và source workspace như trên để ROS dùng overlay này; không
cần sửa file trong `/opt/ros`.

Mỗi terminal ROS mới cần source môi trường:

```bash
source /opt/ros/humble/setup.bash
source /home/vtd/ros2_ws/install/setup.bash
```

## Phần 1: chạy mô phỏng

```bash
ros2 launch tb4_sim sim.launch.py world:=maze model:=standard rviz:=true
```

Để chạy world khác, thay `maze` bằng `warehouse` hoặc `depot`. Dừng mô phỏng bằng `Ctrl+C` trong terminal launch.

Mở terminal khác và kiểm tra các topic:

```bash
source /opt/ros/humble/setup.bash
source /home/vtd/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic info /scan
ros2 topic info /odom
ros2 topic info /cmd_vel
```

Teleop trên Gazebo phát lệnh vào `/cmd_vel`. Lái tránh chướng ngại vật; khi Create3 đang chạy safety reflex, nó có thể tạm bỏ qua lệnh vận tốc.

Robot mô phỏng có thể bắt đầu ở dock. Hãy undock trước khi dùng Teleop:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

Đợi action kết thúc rồi mới lái. Không gửi lệnh vận tốc khi robot còn nằm trên dock.

## Phần 2: tạo bản đồ bằng SLAM Toolbox

Đảm bảo không còn phiên mô phỏng cũ chạy, rồi mở:

```bash
ros2 launch tb4_sim mapping.launch.py world:=maze model:=standard
```

Undock robot trước bằng lệnh ở Phần 1. Sau đó điều khiển bằng Teleop, chạy từ từ qua hành lang và khảo sát toàn bộ khu vực có thể đi tới. Theo dõi map trên RViz2. Khi hoàn tất, ở terminal thứ hai lưu bản đồ vào thư mục dự án:

```bash
source /opt/ros/humble/setup.bash
source /home/vtd/ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f /home/vtd/ros2_ws/src/tb4_sim/maps/map
```

Lệnh này tạo `maps/map.yaml` và `maps/map.pgm`. Sau khi lưu xong, dừng SLAM/mô phỏng bằng `Ctrl+C`.

## Phần 3: localization với bản đồ đã lưu

Khởi động lại robot trong cùng world và cùng vị trí ban đầu:

```bash
ros2 launch tb4_sim localization.launch.py world:=maze model:=standard
```

Trong RViz2, dùng **2D Pose Estimate** để chỉ vị trí và hướng ban đầu của robot trên map. Sau đó kiểm tra robot trên bản đồ có khớp vị trí trong Gazebo.

## Phần 4: autonomous navigation với Nav2

```bash
ros2 launch tb4_sim navigation.launch.py world:=maze model:=standard
```

Trong RViz2, dùng **2D Pose Estimate** nếu cần khởi tạo vị trí; sau đó dùng **Nav2 Goal** để gửi goal. Chọn các goal nằm trong vùng trống trên bản đồ. Không dùng Teleop khi đang đánh giá một navigation goal.

## Cấu hình và bản đồ

- Simulator và Nav2 dùng cấu hình mặc định từ các package TurtleBot 4 đã cài trong ROS 2 Humble.
- Tham số SLAM của dự án nằm tại `config/slam.yaml`; thuật toán được thực thi bởi package `slam_toolbox` đã cài trong ROS 2. File đặt minimum laser range là `0.2 m`, phù hợp với RPLIDAR mô phỏng.
- Bản đồ thật được tạo từ lần chạy SLAM của sinh viên và lưu tại `maps/map.yaml`, `maps/map.pgm`; không dùng map mẫu để thay cho kết quả mapping.
- Nếu lưu map với tên hoặc vị trí khác, truyền đường dẫn YAML vào launch, ví dụ `map:=/duong/dan/map.yaml`.

### Cấu hình LiDAR trong mô phỏng

Robot dùng cảm biến RPLIDAR được mô phỏng bằng `gpu_lidar` của Gazebo Ignition.
Các tham số trong `turtlebot4_description/urdf/sensors/rplidar.urdf.xacro`
trên máy hiện tại:

| Tham số | Giá trị | Ý nghĩa |
|---|---:|---|
| Topic ROS | `/scan` | Laser scan được bridge từ Gazebo |
| Số mẫu ngang | `640` | Số mẫu trong một vòng quét mô phỏng |
| Góc ngang | `-3.1416` đến `+3.1416 rad` | Góc quét 360°, khớp model TurtleBot 4 Humble trên GitHub |
| Số mẫu dọc | `1` | Scan 2D, không quét thành point cloud 3D |
| Tần số cập nhật | `62 Hz` | Tần suất mô phỏng cảm biến |
| Tầm đo tối thiểu | `0.164 m` | Vật cản gần hơn ngưỡng này không được đo tin cậy |
| Tầm đo tối đa | `12.0 m` | Tầm xa tối đa của cảm biến mô phỏng |
| Độ phân giải khoảng cách | `0.01 m` | Bước phân giải khai báo trong Gazebo |
| Pose trong URDF | `xyz=(-0.04, 0, 0.098715) m`, `rpy=(0, 0, π/2)` | Pose lắp cảm biến tương đối với `shell_link`, theo model TurtleBot 4 Humble trên GitHub |

Mô hình LiDAR được đặt theo [xacro TurtleBot 4 Humble trên GitHub](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_description/urdf/sensors/rplidar.urdf.xacro)
và [URDF mẫu Standard](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro).
Phần cứng TurtleBot 4 thật cũng dùng RPLIDAR A1M8 quét 360°, tầm tối đa 12 m.
Tuy nhiên, `62 Hz`, 640 mẫu, `r_min=0.164 m` và `r_res=0.01 m` là giá trị mô
hình/driver trong simulator; SLAMTEC công bố tốc độ quay của A1 có thể cấu hình
2–10 Hz. `ogre2` là renderer Gazebo, không phải cấu hình phần cứng. Tham khảo
[thông tin phần cứng TurtleBot 4](https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html)
và [thông số RPLIDAR A1 từ SLAMTEC](https://www.slamtec.com/en/lidar/a1/).

SLAM Toolbox dùng `scan_topic: scan`, được launch remap tới `/scan`; cấu hình
`minimum_laser_range: 0.2 m` lọc các phép đo quá gần. Plugin Gazebo Sensors
được đặt `render_engine: ogre2` trong overlay dự án
`vendor/irobot_create_description/urdf/create3.urdf.xacro`, vì cảm biến là
GPU LiDAR. Khi dùng workspace này, cần build và source `install/setup.bash`
để ROS tìm thấy overlay.
Tham khảo cách cấu hình GPU LiDAR trong [tài liệu cảm biến Gazebo Fortress](https://gazebosim.org/docs/fortress/sensors/).

Hai package description được overlay trong `vendor/` để các tham số và renderer
được giữ cùng dự án. Sau khi chỉnh overlay, build lại `tb4_sim` và source
`install/setup.bash` trước khi launch.

## Cấu trúc package

```text
tb4_sim/
├── launch/
│   ├── sim.launch.py
│   ├── mapping.launch.py
│   ├── localization.launch.py
│   └── navigation.launch.py
├── maps/                 # map.yaml và map.pgm sau khi chạy SLAM
├── config/slam.yaml      # tham số SLAM Toolbox của dự án
├── vendor/               # description overlays cho Gazebo/LiDAR
├── resource/tb4_sim
├── tb4_sim/__init__.py
├── package.xml
├── setup.py
└── README.md
```
