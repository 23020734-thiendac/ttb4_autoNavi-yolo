#!/usr/bin/env python3
"""Run YOLO on the TurtleBot 4 RGB camera and show annotated frames."""

import time

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class YoloDetectorNode(Node):
    EXPECTED_CLASSES = {
        'sofa', 'trash_bin', 'table', 'bed', 'chandelier', 'refrigerator',
    }
    CLASS_ALIASES = {
        'trash_can': 'trash_bin',
        'trash can': 'trash_bin',
    }
    CLASS_COLORS = {
        'sofa': (255, 0, 255),
        'trash_bin': (80, 220, 80),
        'table': (255, 255, 0),
        'bed': (180, 80, 255),
        'chandelier': (80, 255, 180),
        'refrigerator': (255, 180, 100),
    }

    def __init__(self):
        super().__init__('week5_yolo_detector')
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('confidence', 0.25)
        self.declare_parameter('show_window', True)
        self.declare_parameter('strict_custom_model', True)
        self.declare_parameter(
            'output_topic', '/tb4_week5/yolo/detections_image')

        self.bridge = CvBridge()
        self.model = None
        self.last_detection_log = 0.0
        model_path = str(self.get_parameter('model').value)
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.validate_model_classes()
            self.get_logger().info(f'Da nap YOLO model: {model_path}')
        except Exception as exc:
            self.get_logger().error(
                f'Khong nap duoc YOLO model {model_path}: {exc}')

        image_topic = str(self.get_parameter('image_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.get_logger().info(f'Dang subscribe camera: {image_topic}')
        self.get_logger().info(f'Dang publish anh YOLO: {output_topic}')

    @classmethod
    def normalize_class_name(cls, raw_name):
        name = str(raw_name).strip().lower()
        return cls.CLASS_ALIASES.get(name, name)

    def validate_model_classes(self):
        names = self.model.names
        if isinstance(names, dict):
            raw_names = names.values()
        else:
            raw_names = names
        actual = {self.normalize_class_name(name) for name in raw_names}
        missing = self.EXPECTED_CLASSES - actual
        unexpected = actual - self.EXPECTED_CLASSES
        if not missing and not unexpected:
            self.get_logger().info(
                f'Model custom dung {len(actual)} class small-house: ' +
                ', '.join(sorted(actual)))
            return

        message = (
            'Sai bo class YOLO. Thieu=[' + ', '.join(sorted(missing)) +
            '], khac=[' + ', '.join(sorted(unexpected)) + ']. '
            'Hay dung models/small_house_best.pt, khong dung yolo11n.pt COCO.')
        if bool(self.get_parameter('strict_custom_model').value):
            self.model = None
            raise ValueError(message)
        self.get_logger().warning(message)

    @classmethod
    def display_class_name(cls, result, class_id):
        """Return the assignment label instead of the default COCO label."""
        names = result.names
        if isinstance(names, dict):
            raw_name = names.get(class_id, str(class_id))
        else:
            raw_name = names[class_id] if class_id < len(names) else str(class_id)
        return cls.normalize_class_name(raw_name)

    @classmethod
    def annotate_frame(cls, frame, result):
        """Draw boxes with the labels required by the practical exercise."""
        import cv2

        for box in result.boxes:
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            label = cls.display_class_name(result, class_id)
            x1, y1, x2, y2 = [int(value) for value in box.xyxy[0].tolist()]
            color = cls.CLASS_COLORS.get(label, (0, 200, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            text = f'{label} {confidence:.2f}'
            (text_width, text_height), baseline = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            top = max(y1 - text_height - baseline - 4, 0)
            cv2.rectangle(
                frame, (x1, top), (x1 + text_width + 4, y1), color, -1)
            cv2.putText(
                frame, text, (x1 + 2, y1 - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                cv2.LINE_AA)
        return frame

    def image_callback(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge loi: {exc}')
            return

        detections = []
        if self.model is not None:
            try:
                result = self.model.predict(
                    frame,
                    conf=float(self.get_parameter('confidence').value),
                    verbose=False,
                )[0]
                frame = self.annotate_frame(frame, result)
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    label = self.display_class_name(result, class_id)
                    x1, y1, x2, y2 = [
                        int(value) for value in box.xyxy[0].tolist()]
                    detections.append(
                        f'{label} conf={confidence:.2f} '
                        f'bbox=[{x1},{y1},{x2},{y2}]')
            except Exception as exc:
                self.get_logger().warning(f'YOLO inference loi: {exc}')

        if detections and time.monotonic() - self.last_detection_log > 1.0:
            self.get_logger().info('Phat hien: ' + ', '.join(detections))
            self.last_detection_log = time.monotonic()

        try:
            output = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            output.header = message.header
            self.publisher.publish(output)
        except CvBridgeError as exc:
            self.get_logger().error(f'Khong publish anh YOLO: {exc}')

        if bool(self.get_parameter('show_window').value):
            try:
                import cv2
                cv2.imshow('TB4 Week 5 YOLO', frame)
                cv2.waitKey(1)
            except Exception as exc:
                self.get_logger().warning(f'Khong mo duoc cua so camera: {exc}')
                self.set_parameters([Parameter(
                    'show_window', Parameter.Type.BOOL, False)])


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            import cv2
            cv2.destroyAllWindows()
        except ImportError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
