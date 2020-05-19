import numpy as np
import time
from datetime import datetime

import zense_pb2_grpc
import zense_pb2 as Image
import zense_pb2
import grpc

import os
import sys

WORKING_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(WORKING_DIR, '..'))
from zense_grpc_pywrapper import PicoZenseGRPCServerImpl

from utils.convert_pb_ndarray import ndarray_to_bytes
from concurrent import futures

_ONE_DAY_IN_SECONDS = 60 * 60 * 24

import pdb
CFG_PARAM_PATH = "{}/../cfg/camera.toml".format(WORKING_DIR).encode('utf-8')
CAM_KEY = "Camera0".encode('utf-8')


# It is necesarry this object is unique.
zense = PicoZenseGRPCServerImpl(CFG_PARAM_PATH, CAM_KEY, 0)


class ZenseServiceServicer(zense_pb2_grpc.ZenseServiceServicer):
    def __init__(self):
        global zense
        self.is_rgb = zense.is_rgb
        self.is_ir = zense.is_ir
        self.is_wdr = zense.is_wdr

    # TODO : check inifinite roop
    # TODO : implement exception process
    def SendRGBDImage(self, request, context):
        global zense
        if not self.is_rgb:
            print("Current Configuration is not RGB enabled")
            return zense_pb2.ImageRGBDReply()
        print("test")
        while not zense.update():
            print("pass")
            pass

        self.rgb_image = zense.rgb_image
        self.depth_image = zense.depth_image_range1
        self.depth_range = zense.get_depth_range

        timestamp = self.get_timestamp_microseconds()        
        rgb_img_pb = zense_pb2.Image(
            height=self.rgb_image.shape[0],
            width=self.rgb_image.shape[1],
            timestamp=timestamp,
            channel=3,
            data = ndarray_to_bytes(self.rgb_image)
        )

        depth_img_pb = zense_pb2.Image(
            height=self.depth_image.shape[0],
            width=self.depth_image.shape[1],
            timestamp=timestamp,
            channel=1,
            depth_range=self.depth_range,
            data=ndarray_to_bytes(self.depth_image)
        )

        return zense_pb2.ImageRGBDReply(image_rgb=rgb_img_pb, image_depth=depth_img_pb)


    def SendRGBDIRImage(self, request, context):
        return zense_pb2.RGBDIRReply()
    '''
        if (not self.is_rgb) or (not self.is_ir):
            return zense_pb2.RGBDIRReply()
        if not self.zense.update():
            pass
        rgb_img_pb = zense_pb2.Image(
            data=ndarray_to_bytes(self.zense.rgb_image))
        ir_img_pb = zense_pb2.Image(data=ndarray_to_bytes(self.zense.ir_image))
        depth_img_pb = zense_pb2.Image(
            data=ndarray_to_bytes(self.zense.depth_image))
        return zense_pb2.RGBDIRReply(image_rgb=rgb_img_pb, image_ir=ir_img_pb, image_depth=depth_img_pb)
    '''

    def get_timestamp_microseconds(self):
        return int((datetime.now() - datetime.utcfromtimestamp(0)).total_seconds() * 1e6)

    def from_microseconds_to_timestamp(self, msec):
        return datetime.utcfromtimestamp(msec*1e-6)

    def serve(self):
        server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        zense_pb2_grpc.add_ZenseServiceServicer_to_server(
            ZenseServiceServicer(), server)
        server.add_insecure_port('localhost:50051')
        server.start()

        print('Zense Server Launched')
        try:
            while True:
                time.sleep(_ONE_DAY_IN_SECONDS)
        except KeyboardInterrupt:
            server.stop(0)


def main():
    servicer = ZenseServiceServicer()
    servicer.serve()


if __name__ == '__main__':
    main()
