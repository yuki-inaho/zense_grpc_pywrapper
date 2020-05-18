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
import time
import pdb

WINDOW_NAME = "gRPC Test"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

options = [('grpc.max_send_message_length', 100 * 1024 * 1024),
           ('grpc.max_receive_message_length', 100 * 1024 * 1024)
           ]  # Message size is up to 10MB

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def is_rgbd_enabled():
    cfg_path = os.path.join(SCRIPT_DIR, "../cfg/camera.toml")
    toml_dict = toml.load(open(cfg_path))
    isWDR = int(toml_dict["Camera0"]["range1"]) >= 0 and \
        int(toml_dict["Camera0"]["range2"]) >= 0
    isRGB = int(toml_dict["Camera0"]["rgb_image"]) >= 0
    do_exit = False
    if isWDR:
        print(
            "Current camera setting is WDR mode. This app can be executed under WDR disabled setting"
        )
        do_exit = True
    if not isRGB:
        print(
            "Current camera setting is RGB disabled mode. This app can be executed under RGB enabled setting"
        )
        do_exit = True
    if do_exit:
        assert False


key = cv2.waitKey(10)
is_rgbd_enabled()
count = 0

while key & 0xFF != ord('q') or key & 0xFF != 27:

    with grpc.insecure_channel('localhost:50051', options=options) as channel:
        stub = zense_pb2_grpc.ZenseServiceStub(channel)
        _response = stub.SendRGBDImage(zense_pb2.ImageRequest())        
        # serverが生きてるか死んでるか
        if _response.result().image_rgb.width == 0:
            print("no")
            pass
        else:
            print("OK1")
    with grpc.insecure_channel('localhost:50051', options=options) as channel:
        stub = zense_pb2_grpc.ZenseServiceStub(channel)
        _response = stub.SendRGBDIRImage.future(zense_pb2.ImageRequest())
        if _response.result().image_rgb.width == 0:
            print("no2")
            pass
        else:
            print("OK2")
