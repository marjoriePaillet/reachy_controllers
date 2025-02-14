"""
Camera Node.

- publish /left_image and /right_image at the specified rate (default: 30Hz)

Supported resolutions are: (1920,1080) at 15FPS, (1280,720) at 30FPS, (640,480) at 30FPS.
"""
import time
from functools import partial

import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg._compressed_image import CompressedImage


class CameraPublisher(Node):
    """Camera Publisher class."""

    def __init__(self,
                 left_port: str = '/dev/video4',
                 right_port: str = '/dev/video0',
                 resolution: tuple = (640, 480),
                 fps: float = 30.0) -> None:
        """Connect to both cameras, initialize the publishers."""
        super().__init__('camera_publisher')
        self.logger = self.get_logger()

        time.sleep(15)

        self.image_left = CompressedImage()
        self.cap_left = cv.VideoCapture(left_port, apiPreference=cv.CAP_V4L2)

        self.cap_left.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        self.cap_left.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap_left.set(cv.CAP_PROP_FPS, fps)
        self.cap_left.set(cv.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap_left.set(cv.CAP_PROP_FRAME_HEIGHT, resolution[1])

        self.image_right = CompressedImage()
        self.cap_right = cv.VideoCapture(right_port, apiPreference=cv.CAP_V4L2)

        self.cap_right.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        self.cap_right.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap_right.set(cv.CAP_PROP_FPS, fps)
        self.cap_right.set(cv.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap_right.set(cv.CAP_PROP_FRAME_HEIGHT, resolution[1])

        self.cap = {
            'left': self.cap_left,
            'right': self.cap_right
        }
        self.rot = {
            'left': 3,  # 3 * 90 = 270
            'right': 1,  # 1 * 90 = 90
        }

        self.clock = self.get_clock()
        self.camera_publisher_left = self.create_publisher(CompressedImage, 'left_image', 1)
        self.publish_timer_l = self.create_timer(
            timer_period_sec=1/fps,
            callback=partial(self.publish_img, 'left')
            )
        self.logger.info(f'Launching "{self.camera_publisher_left.topic_name}" publisher.')

        self.camera_publisher_right = self.create_publisher(CompressedImage, 'right_image', 1)
        self.publish_timer_r = self.create_timer(
             timer_period_sec=1/fps,
             callback=partial(self.publish_img, 'right')
             )
        self.logger.info(f'Launching "{self.camera_publisher_right.topic_name}" publisher.')

        self.publisher = {
            'left': self.camera_publisher_left,
            'right': self.camera_publisher_right
        }

        self.bridge = CvBridge()

        self.logger.info('Node ready!')

    def publish_img(self, side: str) -> None:
        """Read image from the requested side and publishes it."""
        b, img = self.cap[side].read()
        if not b:
            self.logger.warning('Failed to grab frame!')
            return
        img = np.rot90(img, self.rot[side])
        img_msg = self.bridge.cv2_to_compressed_imgmsg(img)
        self.publisher[side].publish(img_msg)


def main() -> None:
    """Run Camera publisher main loop."""
    rclpy.init()

    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
