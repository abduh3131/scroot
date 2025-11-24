"""
Sensor interface node that subscribes to annotated images, LiDAR scans, and YOLO
results coming from a Jetson publisher. The node republishes the streams under a
consistent namespace that downstream planners can consume locally.

Expected upstream topics (remap as needed):
- /jetson/annotated_image (sensor_msgs/Image)
- /jetson/lidar_scan (sensor_msgs/LaserScan)
- /jetson/yolo_detections (vision_msgs/Detection2DArray)

Republished topics:
- /scooter/annotated_image (sensor_msgs/Image)
- /scooter/lidar_scan (sensor_msgs/LaserScan)
- /scooter/yolo_detections (vision_msgs/Detection2DArray)
"""

import rospy
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray


class SensorInterface:
    """Bridge that republishes Jetson sensor topics under the /scooter namespace."""

    def __init__(self) -> None:
        rospy.init_node("sensor_interface", anonymous=False)
        jetson_ns = rospy.get_param("~jetson_namespace", "jetson")

        # Subscribers to the Jetson topics.
        rospy.Subscriber(f"/{jetson_ns}/annotated_image", Image, self._image_cb)
        rospy.Subscriber(f"/{jetson_ns}/lidar_scan", LaserScan, self._lidar_cb)
        rospy.Subscriber(f"/{jetson_ns}/yolo_detections", Detection2DArray, self._yolo_cb)

        # Publishers under the local scooter namespace.
        self.image_pub = rospy.Publisher("/scooter/annotated_image", Image, queue_size=1)
        self.lidar_pub = rospy.Publisher("/scooter/lidar_scan", LaserScan, queue_size=1)
        self.yolo_pub = rospy.Publisher("/scooter/yolo_detections", Detection2DArray, queue_size=1)

        rospy.loginfo("sensor_interface: bridging Jetson topics from namespace '%s'", jetson_ns)

    def _image_cb(self, msg: Image) -> None:
        """Pass through annotated image frames."""
        self.image_pub.publish(msg)

    def _lidar_cb(self, msg: LaserScan) -> None:
        """Pass through LiDAR scans."""
        self.lidar_pub.publish(msg)

    def _yolo_cb(self, msg: Detection2DArray) -> None:
        """Pass through YOLO detection arrays."""
        self.yolo_pub.publish(msg)

    def spin(self) -> None:
        """Keep the bridge alive."""
        rospy.spin()


if __name__ == "__main__":
    bridge = SensorInterface()
    bridge.spin()
