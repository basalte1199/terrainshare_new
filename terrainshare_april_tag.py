#!/usr/bin/env python
# -*- coding: utf-8 -*-
import copy
import time
import argparse

import cv2 as cv
from pupil_apriltags import Detector
import pyrealsense2 as rs
import math
import numpy as np


pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)


config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

pipeline.start(config)


align_to =rs.stream.color
align = rs.align(align_to)



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

    args = parser.parse_args()

    return args


def main():
    # 引数解析 #################################################################
    args = get_args()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    families = args.families
    nthreads = args.nthreads
    quad_decimate = args.quad_decimate
    quad_sigma = args.quad_sigma
    refine_edges = args.refine_edges
    decode_sharpening = args.decode_sharpening
    debug = args.debug

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

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        aligend_depth_frame = aligned_frames.get_depth_frame()
        aligend_color_frame = aligned_frames.get_color_frame()

        aligend_depth_image = np.asanyarray(aligend_depth_frame.get_data())
        aligend_color_image = np.asanyarray(aligend_color_frame.get_data())

        # カメラキャプチャ #####################################################
        debug_image = copy.deepcopy(aligend_color_image)

        # 検出実施 #############################################################
        image = cv.cvtColor(aligend_color_image, cv.COLOR_BGR2GRAY)
        tags = at_detector.detect(
            image,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=None,
        )

        # 描画 ################################################################
        debug_image, lastest_position = draw_tags(debug_image, tags, elapsed_time, aligend_depth_frame, lastest_position, lastest_center)

        print(lastest_position)

        elapsed_time = time.time() - start_time

        # キー処理(ESC：終了) #################################################
        key = cv.waitKey(1)
        if key == 27:  # ESC
            break

        # 画面反映 #############################################################
        cv.imshow('AprilTag Detect Demo', debug_image)

    pipeline.stop()
    cv.destroyAllWindows()


def draw_tags(
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


        depth_data = aligend_depth_frame.get_distance(int(center[0]),int(center[1]))

        center = (int(center[0]), int(center[1]))
        corner_01 = (int(corners[0][0]), int(corners[0][1]))
        corner_02 = (int(corners[1][0]), int(corners[1][1]))
        corner_03 = (int(corners[2][0]), int(corners[2][1]))
        corner_04 = (int(corners[3][0]), int(corners[3][1]))

        # 中心
        cv.circle(image, (center[0], center[1]), 5, (0, 0, 255), 2)

        # 各辺
        cv.line(image, (corner_01[0], corner_01[1]),
                (corner_02[0], corner_02[1]), (255, 0, 0), 2)
        cv.line(image, (corner_02[0], corner_02[1]),
                (corner_03[0], corner_03[1]), (255, 0, 0), 2)
        cv.line(image, (corner_03[0], corner_03[1]),
                (corner_04[0], corner_04[1]), (0, 255, 0), 2)
        cv.line(image, (corner_04[0], corner_04[1]),
                (corner_01[0], corner_01[1]), (0, 255, 0), 2)


        cv.line(image, (0,center[1]), (1280,center[1]), (0, 0, 0), 2)
        cv.line(image, (center[0],0), (center[0],720), (0, 0, 0), 2)

        # タグファミリー、タグID
        # cv.putText(image,
        #            str(tag_family) + ':' + str(tag_id),
        #            (corner_01[0], corner_01[1] - 10), cv.FONT_HERSHEY_SIMPLEX,
        #            0.6, (0, 255, 0), 1, cv.LINE_AA)
        cv.putText(image, "id: " + str(tag_id), (center[0] - 10, center[1] - 10),
                   cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv.LINE_AA)

        cv.putText(image, "Depth " + str(int(depth_data * 100)) + "cm", (center[0] - 40, center[1] - 40),
                   cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv.LINE_AA)
        

        if tag_id == 1:
            lastest_center = (int(center[0]), int(center[1]))
            lastest_position = lastest_center
        else:
            lastest_center = lastest_position

        if tag_id == 2:

            cv.line(image, (lastest_position[0],lastest_position[1]), (center[0],center[1]), (0, 0, 0), 2)




    # 処理時間
    cv.putText(image,
               "Elapsed Time:" + '{:.1f}'.format(elapsed_time * 1000) + "ms",
               (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
               cv.LINE_AA)

    #print(lastest_center)
    return image, lastest_center


if __name__ == '__main__':
    main()