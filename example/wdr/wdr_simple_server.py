
import os
import sys
import numpy as np
import time
from datetime import datetime
from concurrent import futures
import grpc

WORKING_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(WORKING_DIR, '../..'))
from zense_grpc_pywrapper import PicoZenseGRPCServerImpl

sys.path.append(os.path.join(WORKING_DIR, '../../scripts'))
import zense_pb2
import zense_pb2 as Image
import zense_pb2_grpc
from utils.convert_pb_ndarray import ndarray_to_bytes

_ONE_DAY_IN_SECONDS = 60 * 60 * 24

CFG_PARAM_PATH = "{}/../../cfg/camera.toml".format(WORKING_DIR).encode('utf-8')
CAM_KEY = "Camera0".encode('utf-8')


# It is necesarry this object is unique.
zense = PicoZenseGRPCServerImpl(CFG_PARAM_PATH, CAM_KEY, 0)


class ZenseServiceServicerWDR(zense_pb2_grpc.ZenseServiceServicer):
    def __init__(self):
        global zense
        self.is_rgb = zense.is_rgb
        self.is_ir = zense.is_ir
        self.is_wdr = zense.is_wdr

    # TODO : check inifinite roop
    # TODO : implement exception process
    def SendWDRDepthImage(self, request, context):
        global zense
        if (not self.is_wdr) or self.is_rgb:
            print("Current Configuration is not WDR enabled")
            return zense_pb2.ImageWDRDepthReply()
        while not zense.update():
            pass

        self.depth_image_range1 = zense.depth_image_range1.copy()
        self.depth_image_range2 = zense.depth_image_range2.copy()
        wdr_depth_range = zense.get_depth_range
        self.depth_range1 = wdr_depth_range[0]
        self.depth_range2 = wdr_depth_range[1]

        timestamp = self.get_timestamp_microseconds()

        depth_img_range1_pb = zense_pb2.Image(
            height=self.depth_image_range1.shape[0],
            width=self.depth_image_range1.shape[1],
            timestamp=timestamp,
            channel=1,
            depth_range=self.depth_range1,
            data=ndarray_to_bytes(self.depth_image_range1)
        )

        depth_img_range2_pb = zense_pb2.Image(
            height=self.depth_image_range2.shape[0],
            width=self.depth_image_range2.shape[1],
            timestamp=timestamp,
            channel=1,
            depth_range=self.depth_range2,
            data=ndarray_to_bytes(self.depth_image_range2)
        )
        return zense_pb2.ImageWDRDepthReply(image_depth_range1=depth_img_range1_pb, image_depth_range2=depth_img_range2_pb)

    def get_timestamp_microseconds(self):
        return int((datetime.now() - datetime.utcfromtimestamp(0)).total_seconds() * 1e6)

    def from_microseconds_to_timestamp(self, msec):
        return datetime.utcfromtimestamp(msec*1e-6)

    def serve(self):
        server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        zense_pb2_grpc.add_ZenseServiceServicer_to_server(
            ZenseServiceServicerWDR(), server)
        server.add_insecure_port('localhost:50051')
        server.start()

        print('Zense Server Launched')
        try:
            while True:
                time.sleep(_ONE_DAY_IN_SECONDS)
        except KeyboardInterrupt:
            server.stop(0)


def main():
    servicer = ZenseServiceServicerWDR()
    servicer.serve()


if __name__ == '__main__':
    main()
