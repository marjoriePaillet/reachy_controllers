"""TODO."""
import time

import cv_bridge
import rclpy
from rclpy.node import Node

import cv2 as cv
from cv_bridge import CvBridge

from sensor_msgs.msg._compressed_image import CompressedImage
from reachy_msgs.srv import SetCameraResolution


class CameraSubscriber(Node):
    """Class."""
    def __init__(self) -> None:

        super().__init__('camera_subscriber')

        self.cam_img = None

        self.cv_bridge = CvBridge()

        self.create_subscription(
            CompressedImage,
            'left_image',
            self.on_image_sub,
            1,
            )
        self.set_camera_resolution_client = self.create_client(
            SetCameraResolution,
            'set_camera_resolution',
        )
        # self.change_resolution('1080p')

    def on_image_sub(self, msg) -> None:
        self.cam_img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        print(self.cam_img.shape)

    def change_resolution(self, resolution: str) -> None:
        request = SetCameraResolution.Request(resolution=resolution)

        future = self.set_camera_resolution_client.call_async(request)
        for _ in range(1000):
            if future.done():
                success = future.result().success
                print('Request sent.')
                break
            time.sleep(0.001)


def main() -> None:
    rclpy.init()

    cam_sub = CameraSubscriber()
    # cam_sub.change_resolution('180p')
    rclpy.spin(cam_sub)

    rclpy.shutdown()


if __name__ == '__main__':
    main()