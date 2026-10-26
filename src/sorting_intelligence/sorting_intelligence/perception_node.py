import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from sorting_intelligence.color_detector import detect_color_blob


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback,
            qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(String, '/detected_object', 10)
        self.last_label = None
        self.frame_count = 0
        self.get_logger().info('Perception node started; waiting for camera images')

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count == 1:
            self.get_logger().info(
                f'First camera frame received: {msg.width}x{msg.height} {msg.encoding}'
            )
        try:
            detection = detect_color_blob(
                msg.data, msg.width, msg.height, msg.step, msg.encoding
            )
        except ValueError as error:
            self.get_logger().warning(str(error))
            return

        if detection is None:
            if self.last_label is not None:
                self.get_logger().info('No color blob in the pickup-region image area')
            elif self.frame_count % 50 == 0:
                self.get_logger().info(
                    f'Received {self.frame_count} camera frames; no clear blob in pickup ROI'
                )
            self.last_label = None
            return

        detection['image_width'] = msg.width
        detection['image_height'] = msg.height
        output = String()
        output.data = json.dumps(detection)
        self.publisher.publish(output)

        if detection['label'] != self.last_label:
            self.get_logger().info(
                f"Detected {detection['label']} at image pixels "
                f"({detection['x']}, {detection['y']}), "
                f"confidence={detection['confidence']:.3f}"
            )
        self.last_label = detection['label']


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
