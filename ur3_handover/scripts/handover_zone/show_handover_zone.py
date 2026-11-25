
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class HandoverZoneVisualizer(Node):
    def __init__(self):
        super().__init__("handover_zone_visualizer")

        self.publisher = self.create_publisher(
            Marker, "/ur3_handover/zone_marker", 10
        )

        self.timer = self.create_timer(1.0, self.publish_zone)

        # Initial example only — not a calibrated handover location.
        self.zone_center = (-0.25, 0.35, 0.20)
        self.zone_size = (0.20, 0.20, 0.15)

    def publish_zone(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "handover_zone"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = self.zone_center[0]
        marker.pose.position.y = self.zone_center[1]
        marker.pose.position.z = self.zone_center[2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.zone_size[0]
        marker.scale.y = self.zone_size[1]
        marker.scale.z = self.zone_size[2]

        marker.color.r = 0.1
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 0.25

        self.publisher.publish(marker)


def main():
    rclpy.init()
    node = HandoverZoneVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
