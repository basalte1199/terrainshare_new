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
import pyrealsense2 as rs
import math
import numpy as np


class RsSub(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(RGBD, '/camera/camera/rgbd', self.listener_callback, 10)

    def listener_callback(self, msg):
        cv2.imshow("Image window", self.mask_rgb(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"), self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")))
        cv2.waitKey(1)

    def get_args():
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

        set = parser.parse_args()

        return set

    def listener_callback(self, msg):
        # 引数解析 #################################################################
        sets = self.get_args()

        cap_device =    sets.device
        cap_width = sets.width
        cap_height =    sets.height

        families =  sets.families
        nthreads =  sets.nthreads
        quad_decimate = sets.quad_decimate
        quad_sigma =    sets.quad_sigma
        refine_edges =  sets.refine_edges
        decode_sharpening = sets.decode_sharpening
        debug = sets.debug

        # カメラ準備 ###############################################################


        # Detector準備 #############################################################
        at_detector = Detector(
            families=families,
            nthreads=nthreads,
            quad_decimate=quad_decimate,
            quad_sigma=quad_sigma,
            refine_edges=refine_edges,
            decode_sharpening=decode_sharpening,
            debug=debug,
        )

        elapsed_time = 0
        lastest_position = []
        lastest_center = []

        while True:
            start_time = time.time()


            # カメラキャプチャ #####################################################
            debug_image = copy.deepcopy(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"))

            # 検出実施 #############################################################
            image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8"), cv2.COLOR_BGR2GRAY)
            tags = at_detector.detect(
                image,
                estimate_tag_pose=False,
                camera_params=None,
                tag_size=None,
            )

            # 描画 ################################################################
            debug_image, lastest_position = self.draw_tags(debug_image, tags, elapsed_time, self.bridge.imgmsg_to_cv2(msg.depth, "passthrough"), lastest_position, lastest_center)

            print(lastest_position)

            elapsed_time = time.time() - start_time

            # キー処理(ESC：終了) #################################################
            key = cv2.waitKey(1)
            if key == 27:  # ESC
                break

        # 画面反映 #############################################################
            cv2.imshow('AprilTag Detect Demo', debug_image)

        cv2.destroyAllWindows()





    def draw_tags(
        self,
        msg,
        image,
        tags,
        elapsed_time,
        aligend_depth_frame,
        lastest_position,
        lastest_center
    ):
        for tag in tags:
            tag_family = tag.tag_family
            tag_id = tag.tag_id
            center = tag.center
            corners = tag.corners

            depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")
            depth_data = depth_image(int(center[0]),int(center[1]))

            center = (int(center[0]), int(center[1]))
            corner_01 = (int(corners[0][0]), int(corners[0][1]))
            corner_02 = (int(corners[1][0]), int(corners[1][1]))
            corner_03 = (int(corners[2][0]), int(corners[2][1]))
            corner_04 = (int(corners[3][0]), int(corners[3][1]))

            # 中心
            cv2.circle(image, (center[0], center[1]), 5, (0, 0, 255), 2)

            # 各辺
            cv2.line(image, (corner_01[0], corner_01[1]),
                (corner_02[0], corner_02[1]), (255, 0, 0), 2)
            cv2.line(image, (corner_02[0], corner_02[1]),
                (corner_03[0], corner_03[1]), (255, 0, 0), 2)
            cv2.line(image, (corner_03[0], corner_03[1]),
                (corner_04[0], corner_04[1]), (0, 255, 0), 2)
            cv2.line(image, (corner_04[0], corner_04[1]),
                (corner_01[0], corner_01[1]), (0, 255, 0), 2)


            cv2.line(image, (0,center[1]), (1280,center[1]), (0, 0, 0), 2)
            cv2.line(image, (center[0],0), (center[0],720), (0, 0, 0), 2)

            # タグファミリー、タグID
            # cv.putText(image,
            #            str(tag_family) + ':' + str(tag_id),
            #            (corner_01[0], corner_01[1] - 10), cv.FONT_HERSHEY_SIMPLEX,
            #            0.6, (0, 255, 0), 1, cv.LINE_AA)
            cv2.putText(image, "id: " + str(tag_id), (center[0] - 10, center[1] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.putText(image, "Depth " + str(int(depth_data * 100)) + "cm", (center[0] - 40, center[1] - 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
        




rclpy.init  (args=None)
rclpy.spin(RsSub())