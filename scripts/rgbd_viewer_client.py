#! /usr/bin/python
#  capture and visualize RGB-D images using zense (range1 = {>0, e.g. 0}, range2 = -1, rgb_image = 1)

import os
import toml
import grpc
import zense_pb2
import zense_pb2_grpc
import numpy as np
import cv2
import cvui
import pdb
from utils.convert_pb_ndarray import bytes_to_ndarray

WORKING_DIR = os.path.dirname(os.path.abspath(__file__))

WINDOW_NAME = "gRPC Test"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

options = [('grpc.max_send_message_length', 30 * 1024 * 1024),
           ('grpc.max_receive_message_length', 30 * 1024 * 1024)
           ]  # Message size is up to 10MB

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def is_rgbd_enabled():
    cfg_path = os.path.join(
        SCRIPT_DIR, "{}/../cfg/camera.toml".format(WORKING_DIR)
    )
    toml_dict = toml.load(open(cfg_path))
    isWDR = int(toml_dict["Camera0"]["range1"]) >= 0 and \
        int(toml_dict["Camera0"]["range2"]) >= 0
    isRGB = int(toml_dict["Camera0"]["rgb_image"]) >= 0
    do_exit = False
    if isWDR:
        print("Current camera setting is WDR mode. This app can be executed under WDR disabled setting")
        do_exit = True
    if not isRGB:
        print("Current camera setting is RGB disabled mode. This app can be executed under RGB enabled setting")
        do_exit = True
    if do_exit:
        assert False


class RGBDImageManager:
    def __init__(self, options):
        self.rgb_img = None
        self.depth_img = None

    def update_rgb(self, response):
        w = response.image_rgb.width
        h = response.image_rgb.height
        c = response.image_rgb.channel
        if w == 0:
            return False
        img_np = bytes_to_ndarray(response.image_rgb.data)
        self.img_rgb = img_np.reshape(h, w, c)
        return True

    def update_depth(self, response):
        w = response.image_depth.width
        h = response.image_depth.height
        if w == 0:
            return False
        img_np = bytes_to_ndarray(response.image_depth.data)
        self.img_depth = img_np.reshape(h, w)
        return True

    def update(self):
        with grpc.insecure_channel('localhost:50051', options=options) as self.channel:
            stub = zense_pb2_grpc.ZenseServiceStub(self.channel)
            response = stub.SendRGBDImage(zense_pb2.ImageRequest())
            status = self.update_rgb(response)
            status &= self.update_depth(response)
        return status

    @property
    def rgb_image(self):
        return self.img_rgb

    @property
    def depth_image(self):
        return self.img_depth

    @property
    def depth_image_colorized(self):
        depth_img_colorized = np.zeros(
            [self.img_depth.shape[0], self.img_depth.shape[1],
             3]).astype(np.uint8)
        depth_img_colorized[:, :, 1] = 255
        depth_img_colorized[:, :, 2] = 255

        _depth_img_zense_hue = self.img_depth.copy().astype(np.float32)
        _depth_img_zense_hue[np.where(_depth_img_zense_hue > 2000)] = 0
        zero_idx = np.where((_depth_img_zense_hue > 2000)
                            | (_depth_img_zense_hue == 0))
        _depth_img_zense_hue *= 255.0 / 2000.0

        depth_img_colorized[:, :, 0] = _depth_img_zense_hue.astype(np.uint8)
        depth_img_colorized = cv2.cvtColor(depth_img_colorized,
                                           cv2.COLOR_HSV2RGB)
        depth_img_colorized[zero_idx[0], zero_idx[1], :] = 0

        return depth_img_colorized


#
# main rootine
#

is_rgbd_enabled()
zense_mng = RGBDImageManager(options)

cvui.init(WINDOW_NAME)
key = cv2.waitKey(10)
while ((key & 0xFF != ord('q')) or (key & 0xFF != 27)):
    status = zense_mng.update()
    if status:
        rgb_img = zense_mng.rgb_image
        depth_img_colorized = zense_mng.depth_image_colorized

        rgb_img_resized = cv2.resize(rgb_img, (IMAGE_WIDTH, IMAGE_HEIGHT))
        depth_img_resized = cv2.resize(depth_img_colorized,
                                       (IMAGE_WIDTH, IMAGE_HEIGHT))
        frame = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH * 2, 3), np.uint8)
        frame[0:IMAGE_HEIGHT, 0:IMAGE_WIDTH, :] = rgb_img_resized
        frame[0:IMAGE_HEIGHT, IMAGE_WIDTH:IMAGE_WIDTH * 2, :] = depth_img_colorized
        cvui.update()
        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(20)
        if key == 27:
            break

cv2.destroyAllWindows()
