# TB4 Week 5 - Small House

Package cho bài thực hành 05: localization, navigation tự động 3 goal và vẽ
bounding box vật thể trong AWS RoboMaker Small House.

Package dùng lại mô phỏng tuần 4 nhưng không sửa mã nguồn tuần 4.

## Cài đặt

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-navigation \
  ros-humble-nav2-simple-commander ros-humble-cv-bridge
python3 -m pip install --user ultralytics opencv-python
```

## Nhận diện theo model trong world

Detector hiện tại không dùng YOLO để đoán nhãn. Node đọc trực tiếp tên và pose
của model trong `aws-robomaker-small-house-world/worlds/small_house.world`,
chiếu hộp 3-D lên camera bằng camera-info và TF. Vì vậy không nhận nhầm
`picture` thành `person`, cũng không vẽ các vật thể ngoài danh sách.

Các model được phép và nhãn hiển thị rút gọn là:

- `ChairA_01_*`, `ChairD_01_*` → `chair`
- `Chandelier_01_*` → `chandelier`
- `Ball_01_*` → `ball`
- `Bed_01_*` → `bed`
- `CoffeeTable_01_*` → `coffee_table`
- `Door_01_*` → `door`
- `Tablet_01_*` → `tablet`
- `FitnessEquipment_01_*` → `fitness_equipment`
- `SofaC_01_*` → `sofa`
- `Trash_01_*` → `trash_bin`
- `TV_01_*` → `tv`

Các model khác, gồm `TV_02`, `Wardrobe`, `Refrigerator`, `KitchenTable` và
các ảnh trên tường, được bỏ qua. Hộp quá lớn chiếm trên 60% ảnh cũng được bỏ
qua để tránh che toàn bộ camera. Có depth image thì node còn bỏ hộp bị vật
khác che ở tâm hộp.

Dataset được tạo sạch hoàn toàn từ
texture của `aws-robomaker-small-house-world/models`; không đọc các ảnh
screenshot đã gửi. Các texture sofa, thùng rác, table, bed, chandelier và
refrigerator được dùng để tạo bounding box tương ứng.

Các file train/weights cũ vẫn được giữ lại để không làm mất dữ liệu trước đây,
nhưng launch hiện tại không nạp `small_house_best.pt` và không dùng YOLO.

## Build

```bash
cd /home/thiendac/ros2_ws
source /opt/ros/humble/setup.bash

PYTHONNOUSERSITE=1 colcon build \
  --symlink-install \
  --base-paths src/TuongTacNguoi_Robot \
  --packages-select tb4_sim tb4_week5

source /home/thiendac/ros2_ws/install/setup.bash
```

## Chạy Small House + navigation 3 goal + YOLO

Dọn process cũ, build và chạy bằng model custom:

```bash
pkill -f 'ros2 launch tb4_week5|week5_autonomous_nav|week5_yolo_detector|week5_world_object_bbox|rviz2|ign gazebo|gzserver|gzclient' 2>/dev/null || true
cd /home/thiendac/ros2_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install \
  --base-paths src/TuongTacNguoi_Robot --packages-select tb4_week5
source install/setup.bash
unset PYTHONNOUSERSITE
ros2 launch tb4_week5 week5_navigation_yolo.launch.py \
  ros_domain_id:=68 goal_count:=3 \
  run_navigation:=true run_yolo:=true show_window:=true
```

Tên tham số `run_yolo` được giữ để tương thích với lệnh cũ; đặt `true` sẽ chạy
`world_object_bbox_node`. Launch mặc định dùng `small_house_slam.yaml`. Node
navigation tự publish initial pose, xử lý rời dock/lùi an toàn rồi gửi lần lượt
ba goal; không cần bấm `2D Pose Estimate` hoặc `Nav2 Goal` trên RViz.

## Goal và topic chính

Ba goal được lưu trong `config/goals.yaml` dưới dạng `[x, y, yaw]` trong frame `map`.
Có thể thay tọa độ bằng kết quả lấy từ `/clicked_point` hoặc `/amcl_pose`.

- Localization: `/map`, `/amcl_pose`, `/scan`, `/tf`
- Navigation action: `/navigate_to_pose`
- Dock action: `/undock`
- Camera RGB: `/oakd/rgb/preview/image_raw`
- Ảnh world-object đã vẽ: `/tb4_week5/yolo/detections_image`
- Cửa sổ OpenCV: `TB4 Small House World Objects`
