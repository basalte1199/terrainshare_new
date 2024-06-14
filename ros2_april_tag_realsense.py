import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
import copy
import time
import argparse
from pupil_apriltags import Detector

class RsSub(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(RGBD, '/camera/camera/rgbd', self.listener_callback, 10)
        self.debug_image = None
        self.tags = []
        self.elapsed_time = 0
        self.lastest_position = []
        self.lastest_center = []
        self.at_detector = self.setup_detector()
        #サブスクライブなどのROSの設定、apriltag認識のための設定を反映

    def get_args(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--device", type=int, default=0)
        parser.add_argument("--width", help='cap width', type=int, default=960)
        parser.add_argument("--height", help='cap height', type=int, default=540)
        parser.add_argument("--families", type=str, default='tag36h11')
        parser.add_argument("--nthreads", type=int, default=1)
        parser.add_argument("--quad_decimate", type=float, default=1.5)
        parser.add_argument("--quad_sigma", type=float, default=0.0)
        parser.add_argument("--refine_edges", type=int, default=1)
        parser.add_argument("--decode_sharpening", type=float, default=0.25)
        parser.add_argument("--debug", type=int, default=0)
        sets = parser.parse_args()
        return sets
        #april_tagの認識設定、上で読み込んでる

    def setup_detector(self):
        sets = self.get_args()
        at_detector = Detector(
            families=sets.families,
            nthreads=sets.nthreads,
            quad_decimate=sets.quad_decimate,
            quad_sigma=sets.quad_sigma,
            refine_edges=sets.refine_edges,
            decode_sharpening=sets.decode_sharpening,
            debug=sets.debug,
        )
        return at_detector

    def listener_callback(self, msg):
        start_time = time.time()
        self.debug_image = copy.deepcopy(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))

        image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"), cv2.COLOR_BGR2GRAY)
        self.tags = self.at_detector.detect(image, estimate_tag_pose=False, camera_params=None, tag_size=None)

        self.debug_image, self.lastest_position = self.draw_tags(msg, self.debug_image, self.tags, self.elapsed_time, self.lastest_position, self.lastest_center)

        self.elapsed_time = time.time() - start_time

    def draw_tags(self, msg, image, tags, elapsed_time, lastest_position, lastest_center):
        depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")

        for tag in tags:
            tag_family = tag.tag_family
            tag_id = tag.tag_id
            center = tag.center
            corners = tag.corners
            #タグの深度を測定
            depth_data = depth_image[int(center[0])][int(center[0])]

            center = (int(center[0]), int(center[1]))
            corner_01 = (int(corners[0][0]), int(corners[0][1]))
            corner_02 = (int(corners[1][0]), int(corners[1][1]))
            corner_03 = (int(corners[2][0]), int(corners[2][1]))
            corner_04 = (int(corners[3][0]), int(corners[3][1]))

            cv2.circle(image, center, 5, (0, 0, 255), 2)
            cv2.line(image, corner_01, corner_02, (255, 0, 0), 2)
            cv2.line(image, corner_02, corner_03, (255, 0, 0), 2)
            cv2.line(image, corner_03, corner_04, (0, 255, 0), 2)
            cv2.line(image, corner_04, corner_01, (0, 255, 0), 2)
            cv2.line(image, (0, center[1]), (1280, center[1]), (0, 0, 0), 2)
            cv2.line(image, (center[0], 0), (center[0], 720), (0, 0, 0), 2)
            cv2.putText(image, "id: " + str(tag_id), (center[0] - 10, center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(image, "Depth " + str(int(depth_data * 100)) + "cm", (center[0] - 40, center[1] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)

        return image, lastest_position

def main(args=None):
    rclpy.init(args=args)
    node = RsSub()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            if node.debug_image is not None:
                cv2.imshow('AprilTag Detect Demo', node.debug_image)
                key = cv2.waitKey(1)
                if key == 27:  # ESC
                    break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()