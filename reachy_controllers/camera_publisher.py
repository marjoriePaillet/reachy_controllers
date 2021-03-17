"""
Camera Node.

- publish /left_image and /right_image at the specified rate (default: 30Hz)

"""
from functools import partial

import cv2 as cv

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg._compressed_image import CompressedImage
from reachy_msgs.srv import SetCameraResolution


class CameraPublisher(Node):
    """Camera Publisher class."""

    request_to_resolution = {
        '1080p': (1920,1080),
        '720p': (1280,720),
        '480p': (640,480),
        '360p': (640,360),
        '240p': (320,240),
        '180p': (320,180),
    }

    def __init__(self,
                 left_port: str = '/dev/video0',
                 # right_port: str = '/dev/video4',
                 size: tuple = (720, 576),
                 fps: float = 30.0) -> None:
        """Connect to both cameras, initialize the publishers."""
        super().__init__('camera_publisher')
        self.logger = self.get_logger()

        self.resolution = size # self.request_to_resolution['180p']

        self.image_left = CompressedImage()
        self.cap_left = cv.VideoCapture(left_port, apiPreference=cv.CAP_V4L2)

        self.cap_left.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        self.cap_left.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap_left.set(cv.CAP_PROP_FPS, fps)
        self.cap_left.set(cv.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap_left.set(cv.CAP_PROP_FRAME_HEIGHT, size[1])

        # self.image_right = CompressedImage()
        # self.cap_right = cv.VideoCapture(right_port, apiPreference=cv.CAP_V4L2)

        # self.cap_right.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        # self.cap_right.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        # self.cap_right.set(cv.CAP_PROP_FPS, fps)
        # self.cap_right.set(cv.CAP_PROP_FRAME_WIDTH, size[0])
        # self.cap_right.set(cv.CAP_PROP_FRAME_HEIGHT, size[1])

        self.cap = {
            'left': self.cap_left,
            # 'right': self.cap_right
        }

        self.clock = self.get_clock()
        self.camera_publisher_left = self.create_publisher(CompressedImage, 'left_image', 1)
        self.publish_timer_l = self.create_timer(
            timer_period_sec=1/fps,
            callback=partial(self.publish_img, 'left')
            )
        self.logger.info(f'Launching "{self.camera_publisher_left.topic_name}" publisher.')

        # self.camera_publisher_right = self.create_publisher(CompressedImage, 'right_image', 1)
        # self.publish_timer_r = self.create_timer(
        #      timer_period_sec=1/fps,
        #      callback=partial(self.publish_img, 'right')
        #      )
        # self.logger.info(f'Launching "{self.camera_publisher_right.topic_name}" publisher.')

        self.publisher = {
            'left': self.camera_publisher_left,
            # 'right': self.camera_publisher_right
        }

        self.bridge = CvBridge()

        self.client_resolution = self.create_service(
            srv_type=SetCameraResolution,
            srv_name='set_camera_resolution',
            callback=self.set_resolution,
            )

        self.logger.info('Node ready!')

    def publish_img(self, side: str) -> None:
        """Read image from the requested side and publishes it."""
        _, img = self.cap[side].read()
        print(img.shape)
        img_msg = self.bridge.cv2_to_compressed_imgmsg(img)
        self.publisher[side].publish(img_msg)

    def set_resolution(self,
                       request: SetCameraResolution.Request,
                       response: SetCameraResolution.Response,
                      ) -> SetCameraResolution.Response:
        self.logger.info('Received a resolution request.')
        res = self.request_to_resolution[request.resolution]
        success = self.cap_left.set(cv.CAP_PROP_FRAME_WIDTH, res[0])
        success = self.cap_left.set(cv.CAP_PROP_FRAME_HEIGHT, res[1]) and success
        # success = self.cap_right.set(cv.CAP_PROP_FRAME_WIDTH, res[0]) and success
        # success = self.cap_right.set(cv.CAP_PROP_FRAME_HEIGHT, res[1]) and success

        response.success = success
        self.logger.info(f'success: {success}')
        return response


def main() -> None:
    """Run Camera publisher main loop."""
    rclpy.init()

    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()