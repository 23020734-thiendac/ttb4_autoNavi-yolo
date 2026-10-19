"""Project selected SDF world models into the TurtleBot camera image.

This node is deliberately not a learned detector.  The small-house world
already contains the semantic model name and pose for every object.  We use
that information, the camera calibration and TF to draw an approximate 2-D
box when an object is in the camera view.
"""

from __future__ import annotations

import math
import re
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class WorldObject:
    model_name: str
    label: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


# Approximate footprint/height of the selected assets in metres.  They are
# only used to turn a world pose into a useful image rectangle; the SDF pose
# remains the source of truth for the object's position.
OBJECT_SPECS: Dict[str, Tuple[Tuple[float, float, float], Tuple[int, int, int]]] = {
    'chair': ((0.70, 0.70, 1.05), (40, 220, 40)),
    'chandelier': ((0.75, 0.75, 0.55), (0, 215, 255)),
    'ball': ((0.38, 0.38, 0.38), (255, 120, 0)),
    'bed': ((2.10, 2.20, 0.85), (255, 80, 180)),
    'coffee_table': ((1.25, 0.75, 0.60), (80, 180, 255)),
    'door': ((0.95, 0.16, 2.10), (180, 80, 255)),
    'tablet': ((0.28, 0.10, 0.22), (255, 255, 0)),
    'fitness_equipment': ((2.00, 0.85, 1.25), (0, 165, 255)),
    'sofa': ((2.55, 0.95, 1.05), (255, 0, 160)),
    'trash_bin': ((0.50, 0.50, 0.75), (0, 255, 255)),
    'tv': ((1.25, 0.18, 0.85), (255, 0, 0)),
}


# These patterns intentionally exclude other similarly named objects such as
# KitchenTable, TV_02, Wardrobe and wall pictures.
MODEL_PATTERNS: Sequence[Tuple[re.Pattern[str], str]] = (
    (re.compile(r'^Chair(?:A|D)_01(?:_|$)'), 'chair'),
    (re.compile(r'^Chandelier_01(?:_|$)'), 'chandelier'),
    (re.compile(r'^Ball_01(?:_|$)'), 'ball'),
    (re.compile(r'^Bed_01(?:_|$)'), 'bed'),
    (re.compile(r'^CoffeeTable_01(?:_|$)'), 'coffee_table'),
    (re.compile(r'^Door_01(?:_|$)'), 'door'),
    (re.compile(r'^Tablet_01(?:_|$)'), 'tablet'),
    (re.compile(r'^FitnessEquipment_01(?:_|$)'), 'fitness_equipment'),
    (re.compile(r'^SofaC_01(?:_|$)'), 'sofa'),
    (re.compile(r'^Trash_01(?:_|$)'), 'trash_bin'),
    (re.compile(r'^TV_01(?:_|$)'), 'tv'),
)


def _label_for_model(model_name: str) -> Optional[str]:
    for pattern, label in MODEL_PATTERNS:
        if pattern.match(model_name):
            return label
    return None


def _parse_pose(model: ET.Element) -> Optional[Tuple[float, ...]]:
    pose = model.find('pose')
    if pose is None or not pose.text:
        return None
    try:
        values = [float(value) for value in pose.text.split()]
    except ValueError:
        return None
    if len(values) < 3:
        return None
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])


def parse_world_objects(world_file: str) -> List[WorldObject]:
    """Read selected model names and poses from an SDF world file."""

    path = Path(world_file).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f'World file not found: {path}')

    root = ET.parse(path).getroot()
    objects: List[WorldObject] = []
    seen = set()
    for model in root.findall('.//model'):
        name = model.attrib.get('name', '')
        label = _label_for_model(name)
        if label is None or name in seen:
            continue
        pose = _parse_pose(model)
        if pose is None:
            continue
        seen.add(name)
        objects.append(WorldObject(name, label, *pose))
    return objects


def _quaternion_rotate(
    vector: Sequence[float],
    quaternion: Sequence[float],
) -> np.ndarray:
    """Rotate a 3-D vector by an x/y/z/w quaternion."""

    x, y, z, w = quaternion
    q = np.array([x, y, z], dtype=np.float64)
    v = np.asarray(vector, dtype=np.float64)
    return v + 2.0 * np.cross(q, np.cross(q, v) + w * v)


def _transform_point(point: Sequence[float], transform) -> np.ndarray:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    rotated = _quaternion_rotate(
        point,
        (rotation.x, rotation.y, rotation.z, rotation.w),
    )
    return rotated + np.array(
        [translation.x, translation.y, translation.z], dtype=np.float64)


def _rotate_z(x: float, y: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


class WorldObjectBBoxNode(Node):
    def __init__(self) -> None:
        super().__init__('world_object_bbox_node')

        self.declare_parameter(
            'image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/oakd/rgb/preview/camera_info')
        self.declare_parameter(
            'depth_topic', '/oakd/rgb/preview/depth')
        self.declare_parameter('world_file', '')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter(
            'camera_frame', 'turtlebot4/oakd_rgb_camera_optical_frame')
        self.declare_parameter(
            'output_topic', '/tb4_week5/yolo/detections_image')
        self.declare_parameter('show_window', True)
        self.declare_parameter('max_bbox_area_ratio', 0.60)
        self.declare_parameter('max_bbox_width_ratio', 0.95)
        self.declare_parameter('min_bbox_pixels', 8)
        self.declare_parameter('max_range_m', 25.0)
        self.declare_parameter('occlusion_tolerance_m', 0.30)
        self.declare_parameter('log_period_sec', 1.0)
        self.declare_parameter('horizontal_fov_rad', 1.22)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.max_bbox_area_ratio = float(
            self.get_parameter('max_bbox_area_ratio').value)
        self.max_bbox_width_ratio = float(
            self.get_parameter('max_bbox_width_ratio').value)
        self.min_bbox_pixels = int(self.get_parameter('min_bbox_pixels').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.occlusion_tolerance_m = float(
            self.get_parameter('occlusion_tolerance_m').value)
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        self.horizontal_fov_rad = float(
            self.get_parameter('horizontal_fov_rad').value)

        world_file = str(self.get_parameter('world_file').value)
        self.objects: List[WorldObject] = []
        if world_file:
            try:
                self.objects = parse_world_objects(world_file)
            except (OSError, ET.ParseError) as exc:
                self.get_logger().error(f'Khong doc duoc small_house.world: {exc}')
        else:
            self.get_logger().error(
                'Thieu tham so world_file; khong ve bounding box world-object.')

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info: Optional[CameraInfo] = None
        self.depth_image: Optional[np.ndarray] = None
        self.depth_encoding = ''
        self.last_log_time = 0.0
        self.last_tf_warning = 0.0
        self.last_image_warning = 0.0

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, self.output_topic, 10)

        if self.show_window:
            cv2.namedWindow('TB4 Small House World Objects', cv2.WINDOW_NORMAL)

        by_label: Dict[str, int] = {}
        for obj in self.objects:
            by_label[obj.label] = by_label.get(obj.label, 0) + 1
        summary = ', '.join(
            f'{label}={count}' for label, count in sorted(by_label.items()))
        self.get_logger().info(
            f'Da nap {len(self.objects)} model tu world: {summary or "khong co"}')
        self.get_logger().info(
            'Nhan dang world-object, khong dung YOLO; nhan hien thi la ten rut gon.')

    def camera_info_callback(self, message: CameraInfo) -> None:
        self.camera_info = message

    def depth_callback(self, message: Image) -> None:
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='passthrough')
            self.depth_encoding = message.encoding
        except CvBridgeError:
            self.depth_image = None

    def _camera_model(
        self, width: int, height: int
    ) -> Tuple[float, float, float, float]:
        info = self.camera_info
        if info is not None and len(info.k) >= 9:
            fx, fy, cx, cy = float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])
            if fx > 1.0 and fy > 1.0:
                return fx, fy, cx, cy
        fx = width / (2.0 * math.tan(self.horizontal_fov_rad / 2.0))
        return fx, fx, width / 2.0, height / 2.0

    @staticmethod
    def _model_corners(obj: WorldObject) -> Iterable[np.ndarray]:
        (width, depth, height), _ = OBJECT_SPECS[obj.label]
        for local_z in (0.0, height):
            for local_x in (-width / 2.0, width / 2.0):
                for local_y in (-depth / 2.0, depth / 2.0):
                    dx, dy = _rotate_z(local_x, local_y, obj.yaw)
                    yield np.array(
                        [obj.x + dx, obj.y + dy, obj.z + local_z],
                        dtype=np.float64,
                    )

    @staticmethod
    def _model_center(obj: WorldObject) -> np.ndarray:
        _, _, height = OBJECT_SPECS[obj.label][0]
        return np.array([obj.x, obj.y, obj.z + height / 2.0])

    @staticmethod
    def _project(
        point: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> Optional[Tuple[float, float, float]]:
        # OAK-D optical frame: x right, y down, z forward.
        if point[2] <= 0.10:
            return None
        return (
            fx * point[0] / point[2] + cx,
            fy * point[1] / point[2] + cy,
            float(point[2]),
        )

    def _is_depth_occluded(
        self, u: float, v: float, expected_depth: float, width: int, height: int
    ) -> bool:
        if self.depth_image is None or expected_depth <= 0.0:
            return False
        depth = self.depth_image
        if depth.ndim < 2:
            return False
        du = int(round(u))
        dv = int(round(v))
        if not (0 <= du < depth.shape[1] and 0 <= dv < depth.shape[0]):
            return False
        # The RGB and depth preview streams normally have identical size. If
        # they do not, sample using normalized image coordinates.
        if depth.shape[1] != width or depth.shape[0] != height:
            du = min(depth.shape[1] - 1, max(0, int(u / width * depth.shape[1])))
            dv = min(depth.shape[0] - 1, max(0, int(v / height * depth.shape[0])))
        sample = depth[dv, du]
        if isinstance(sample, np.ndarray):
            sample = sample.flat[0]
        observed = float(sample)
        if not math.isfinite(observed) or observed <= 0.01:
            return False
        if depth.dtype == np.uint16 or '16U' in self.depth_encoding.upper():
            observed /= 1000.0
        return observed < expected_depth - self.occlusion_tolerance_m

    def _bbox_for_object(
        self,
        obj: WorldObject,
        transform,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        width: int,
        height: int,
    ) -> Optional[Tuple[int, int, int, int, float, float, float]]:
        corners = [
            self._project(_transform_point(point, transform), fx, fy, cx, cy)
            for point in self._model_corners(obj)
        ]
        visible = [item for item in corners if item is not None]
        if len(visible) < 2:
            return None
        xs = [item[0] for item in visible]
        ys = [item[1] for item in visible]
        depths = [item[2] for item in visible]
        raw_x0, raw_x1 = min(xs), max(xs)
        raw_y0, raw_y1 = min(ys), max(ys)
        if raw_x1 < 0 or raw_y1 < 0 or raw_x0 >= width or raw_y0 >= height:
            return None
        x0 = max(0, min(width - 1, int(math.floor(raw_x0))))
        y0 = max(0, min(height - 1, int(math.floor(raw_y0))))
        x1 = max(0, min(width - 1, int(math.ceil(raw_x1))))
        y1 = max(0, min(height - 1, int(math.ceil(raw_y1))))
        box_width, box_height = x1 - x0, y1 - y0
        area_ratio = (box_width * box_height) / float(width * height)
        if (box_width < self.min_bbox_pixels or box_height < self.min_bbox_pixels
                or area_ratio > self.max_bbox_area_ratio
                or box_width / float(width) > self.max_bbox_width_ratio):
            return None

        center_camera = _transform_point(self._model_center(obj), transform)
        center_projection = self._project(center_camera, fx, fy, cx, cy)
        if center_projection is None or center_projection[2] > self.max_range_m:
            return None
        if self._is_depth_occluded(
                center_projection[0], center_projection[1], center_projection[2],
                width, height):
            return None
        return x0, y0, x1, y1, center_projection[2], raw_x0, raw_y0

    def _lookup_transform(self, message: Image):
        source_frame = message.header.frame_id or self.camera_frame
        stamp = Time.from_msg(message.header.stamp)
        try:
            return self.tf_buffer.lookup_transform(
                source_frame,
                self.world_frame,
                stamp,
                timeout=Duration(seconds=0.08),
            )
        except TransformException:
            # During startup the exact image timestamp can be just outside the
            # short TF cache. A latest-transform fallback avoids a permanent
            # blank window while the robot is stationary.
            try:
                return self.tf_buffer.lookup_transform(
                    source_frame,
                    self.world_frame,
                    Time(),
                    timeout=Duration(seconds=0.08),
                )
            except TransformException as exc:
                now = time.monotonic()
                if now - self.last_tf_warning > 2.0:
                    self.get_logger().warning(
                        f'Chua co TF {self.world_frame} -> {source_frame}: {exc}')
                    self.last_tf_warning = now
                return None

    def image_callback(self, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as exc:
            now = time.monotonic()
            if now - self.last_image_warning > 2.0:
                self.get_logger().warning(f'Khong doi duoc anh camera: {exc}')
                self.last_image_warning = now
            return

        annotated = image.copy()
        transform = self._lookup_transform(message) if self.objects else None
        detections = []
        height, width = image.shape[:2]
        if transform is not None:
            fx, fy, cx, cy = self._camera_model(width, height)
            for obj in self.objects:
                box = self._bbox_for_object(
                    obj, transform, fx, fy, cx, cy, width, height)
                if box is not None:
                    detections.append((obj, box))

        for obj, box in sorted(detections, key=lambda item: item[1][4]):
            x0, y0, x1, y1, depth, _, _ = box
            color = OBJECT_SPECS[obj.label][1]
            cv2.rectangle(annotated, (x0, y0), (x1, y1), color, 2)
            text = f'{obj.label} {depth:.1f}m'
            text_y = max(18, y0 - 6)
            cv2.putText(
                annotated, text, (x0, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, color, 2, cv2.LINE_AA)

        try:
            output = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            output.header = message.header
            self.image_pub.publish(output)
        except CvBridgeError:
            pass

        now = time.monotonic()
        if detections and now - self.last_log_time >= self.log_period_sec:
            names = ', '.join(
                f'{obj.label}({obj.model_name})' for obj, _ in detections)
            self.get_logger().info(f'World boxes: {names}')
            self.last_log_time = now

        if self.show_window:
            cv2.imshow('TB4 Small House World Objects', annotated)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldObjectBBoxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
