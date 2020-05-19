#! /usr/bin/python
#  capture and visualize WDR-Depth images using zense (range1 = {>0, e.g. 0}, range2 = {>0, e.g. 0}, rgb_image != 1)
import os
import sys
import toml
import grpc
import numpy as np
import cv2
import cvui

WORKING_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(WORKING_DIR, '../../scripts'))
import zense_pb2
import zense_pb2_grpc
from utils.convert_pb_ndarray import bytes_to_ndarray

WINDOW_NAME = "gRPC Test"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

options = [('grpc.max_send_message_length', 10 * 1024 * 1024),
           ('grpc.max_receive_message_length', 10 * 1024 * 1024)
           ]  # Message size is up to 10MB

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def is_wdr_enabled():
    cfg_path = "{}/../../cfg/camera.toml".format(SCRIPT_DIR)
    toml_dict = toml.load(open(cfg_path))
    isWDR = int(toml_dict["Camera0"]["range1"]) >= 0 and \
        int(toml_dict["Camera0"]["range2"]) >= 0
    isRGB = int(toml_dict["Camera0"]["rgb_image"]) == 1
    do_exit = False
    if not isWDR:
        print("Current camera setting is WDR disabled mode. This app can be executed under WDR enabled setting")
        do_exit = True
    if isRGB:
        print("Current camera setting is RGB enabled mode. This app can be executed under RGB disabled setting")
        do_exit = True
    if do_exit:
        assert False


class WDRImageManager:
    def __init__(self, options):
        self.rgb_img = None
        self.depth_img = None

    def update_depth_range1(self, response):
        w = response.image_depth_range1.width
        h = response.image_depth_range1.height
        if w == 0:
            return False
        self.img_depth_range1 = bytes_to_ndarray(
            response.image_depth_range1.data)
        return True

    def update_depth_range2(self, response):
        w = response.image_depth_range2.width
        h = response.image_depth_range2.height
        if w == 0:
            return False
        self.img_depth_range2 = bytes_to_ndarray(
            response.image_depth_range2.data)
        return True

    def update(self):
        with grpc.insecure_channel('localhost:50051', options=options) as self.channel:
            stub = zense_pb2_grpc.ZenseServiceStub(self.channel)
            response = stub.SendWDRDepthImage(zense_pb2.ImageRequest())
            status = self.update_depth_range1(response)
            status &= self.update_depth_range2(response)
        return status

    def depth_image_colorized(self, depth_img):
        depth_img_colorized = np.zeros(
            [depth_img.shape[0], depth_img.shape[1], 3]).astype(np.uint8)
        depth_img_colorized[:, :, 1] = 255
        depth_img_colorized[:, :, 2] = 255

        _depth_img_zense_hue = depth_img.copy().astype(np.float32)
        _depth_img_zense_hue[np.where(_depth_img_zense_hue > 2000)] = 0
        zero_idx = np.where((_depth_img_zense_hue > 2000)
                            | (_depth_img_zense_hue == 0))
        _depth_img_zense_hue *= 255.0 / 2000.0

        depth_img_colorized[:, :, 0] = _depth_img_zense_hue.astype(np.uint8)
        depth_img_colorized = cv2.cvtColor(depth_img_colorized,
                                           cv2.COLOR_HSV2RGB)
        depth_img_colorized[zero_idx[0], zero_idx[1], :] = 0
        return depth_img_colorized

    @property
    def depth_image_range1(self):
        return self.img_depth_range1

    @property
    def depth_image_range2(self):
        return self.img_depth_range2

    @property
    def depth_image_range1_colorized(self):
        return self.depth_image_colorized(self.img_depth_range1)

    @property
    def depth_image_range2_colorized(self):
        return self.depth_image_colorized(self.img_depth_range2)


#
# main rootine
#
is_wdr_enabled()
zense_mng = WDRImageManager(options)
cvui.init(WINDOW_NAME)

key = cv2.waitKey(10)
while ((key & 0xFF != ord('q')) or (key & 0xFF != 27)):
    status = zense_mng.update()
    if status:
        depth_img_r1_colorized = zense_mng.depth_image_range1_colorized
        depth_img_r2_colorized = zense_mng.depth_image_range2_colorized

        depth_img_r1_resized = cv2.resize(depth_img_r1_colorized,
                                          (IMAGE_WIDTH, IMAGE_HEIGHT))
        depth_img_r2_resized = cv2.resize(depth_img_r2_colorized,
                                          (IMAGE_WIDTH, IMAGE_HEIGHT))
        frame = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH * 2, 3), np.uint8)
        frame[0:IMAGE_HEIGHT, 0:IMAGE_WIDTH, :] = depth_img_r1_resized
        frame[0:IMAGE_HEIGHT,
              IMAGE_WIDTH:IMAGE_WIDTH * 2, :] = depth_img_r2_resized

        cvui.update()
        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(20)
        if key == 27:
            break

cv2.destroyAllWindows()
