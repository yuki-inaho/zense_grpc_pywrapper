
from io import BytesIO
import numpy as np
import time

import zense_pb2_grpc
import zense_pb2 as Image
import zense_pb2
import grpc
from zense_grpc_pywrapper import PicoZenseGRPCServerImpl
from concurrent import futures


_ONE_DAY_IN_SECONDS = 60 * 60 * 24

CFG_PARAM_PATH = "./cfg/camera.toml".encode('utf-8')
CAM_KEY = "Camera0".encode('utf-8')


def ndarray_to_bytes(nda: np.ndarray) -> bytes:
    nda_bytes = BytesIO()
    np.save(nda_bytes, nda, allow_pickle=False)
    return nda_bytes.getvalue()


def bytes_to_ndarray(nda_proto: bytes) -> np.ndarray:
    nda_bytes = BytesIO(nda_proto)
    return np.load(nda_bytes, allow_pickle=False)


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
            return zense_pb2.ImageRGBDReply()
        if not zense.update():
            pass

        self.rgb_image = zense.rgb_image
        self.depth_image = zense.depth_image_range1

        rgb_img_pb = zense_pb2.Image(
            data=ndarray_to_bytes(self.rgb_image))
        depth_img_pb = zense_pb2.Image(
            data=ndarray_to_bytes(self.depth_image))
        return zense_pb2.ImageRGBDReply(image_rgb=rgb_img_pb, image_depth=depth_img_pb)

    '''
    def SendRGBDIRImage(self, request, context):
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

    def serve(self):
        server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
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
