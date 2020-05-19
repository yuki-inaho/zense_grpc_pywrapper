#! /usr/bin/python
#  capture and visualize RGB-D-IR images using zense (range1 = {>0, e.g. 0}, range2 = -1, rgb_image = 1)

import os
import sys
import toml
import grpc
import numpy as np
import cv2
import toml
import PySimpleGUI as sg
from enum import Enum

WORKING_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(WORKING_DIR, '../../scripts'))
from utils.convert_pb_ndarray import bytes_to_ndarray
import zense_pb2_grpc
import zense_pb2


drange_dict = {-1: "Undefined", 0: "Near", 1: "Mid", 2: "Far"}
laser_dict = {"Undefined": 160, "Near": 160, "Mid": 280, "Far": 300}

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

WINDOW_NAME = "gRPC Test"
IMAGE_WIDTH = 400
IMAGE_HEIGHT = 300

options = [('grpc.max_send_message_length', 10 * 1024 * 1024),
           ('grpc.max_receive_message_length', 10 * 1024 * 1024)
           ]  # Message size is up to 10MB


def is_rgbdir_enabled():
    cfg_path = "{}/../../cfg/camera.toml".format(SCRIPT_DIR)
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


class RGBDIRImageManager:
    def __init__(self, options):
        self.rgb_img = None
        self.depth_img = None
        self.depth_range = "Undefined"
        self.laser_intensity = "Undefined"

    def update_rgb(self, response):
        w = response.image_rgb.width
        h = response.image_rgb.height
        c = response.image_rgb.channel
        if w == 0:
            return False
        self.img_rgb = bytes_to_ndarray(response.image_rgb.data)
        return True

    def update_ir(self, response):
        w = response.image_ir.width
        h = response.image_ir.height
        if w == 0:
            return False
        self.img_ir = bytes_to_ndarray(response.image_ir.data)
        return True

    def update_depth(self, response):
        w = response.image_depth.width

    def update_rgb(self, response):
        w = response.image_rgb.width
        h = response.image_rgb.height
        h = response.image_rgb.channel
        if w == 0:
            return False
        self.img_rgb = bytes_to_ndarray(response.image_rgb.data)
        return True

    def update_ir(self, response):
        w = response.image_ir.width
        h = response.image_ir.height
        if w == 0:
            return False
        self.img_ir = bytes_to_ndarray(response.image_ir.data)
        return True

    def update_depth(self, response):
        w = response.image_depth.width
        h = response.image_depth.height
        if w == 0:
            self.depth_range = drange_dict[-1]
            return False
        self.img_depth = bytes_to_ndarray(response.image_depth.data)
        self.depth_range = drange_dict[response.image_depth.depth_range]
        return True

    def update_laser_intensity(self, response):
        self.laser_intensity = int(response.intensity)
        return True

    def update(self):
        with grpc.insecure_channel('localhost:50051', options=options) as self.channel:
            stub = zense_pb2_grpc.ZenseServiceStub(self.channel)
            response = stub.SendRGBDIRImage(zense_pb2.ImageRequest())
            status = self.update_rgb(response)
            status &= self.update_ir(response)
            status &= self.update_depth(response)
            response_laser = stub.GetLaserIntensity(
                zense_pb2.GetLaserIntensityRequest())
            status &= self.update_laser_intensity(response_laser)
        return status

    def set_laser_intensity(self, intensity):
        with grpc.insecure_channel('localhost:50051', options=options) as self.channel:
            stub = zense_pb2_grpc.ZenseServiceStub(self.channel)
            request = zense_pb2.SetLaserIntensityRequest()
            request.intensity = intensity
            response = stub.SetLaserIntensity(request)
        return response.is_success

    def set_depth_range(self, given_depth_range):
        with grpc.insecure_channel('localhost:50051', options=options) as self.channel:
            if len(given_depth_range) == 0:
                return True
            stub = zense_pb2_grpc.ZenseServiceStub(self.channel)
            request = zense_pb2.SetDepthRangeRequest()
            request.given_range = given_depth_range
            response = stub.SetDepthRange(request)
        return response.is_success

    @ property
    def rgb_image(self):
        return self.img_rgb

    @ property
    def ir_image(self):
        return self.img_ir

    @ property
    def depth_image(self):
        return self.img_depth

    @ property
    def ir_image_normalized(self):
        ir_img_normalized = np.zeros(
            [self.img_ir.shape[0], self.img_ir.shape[1],
             3]).astype(np.uint8)

        _ir_img_raw = (self.img_ir.copy().astype(
            np.float32)/3840*255).astype(np.uint8)
        for c in range(3):
            ir_img_normalized[:, :, c] = _ir_img_raw

        return ir_img_normalized

    @ property
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


class RGBD_IR_GUIManager():
    def __init__(self):
        self.zense_mng = RGBDIRImageManager(options)
        self.zense_mng.set_laser_intensity(160)

        # define the window layout
        layout = [
            [
                sg.Image(filename='', key='-IMAGE-',
                         tooltip='Right click for exit menu')
            ],
            [
                sg.Text('Depth Range : ',
                        text_color='lightgreen', font='Default 14'),
                sg.Text('', text_color='lightgreen',
                        font='Default 14', key="drange", size=(10, 1)),
                sg.Text('Laser Intensity(Pulse Count) : ',
                        text_color='lightgreen', font='Default 14'),
                sg.Text('', text_color='lightgreen',
                        font='Default 14', key="intensity", size=(5, 1)),
            ],
            [
                sg.Button('Set Laser Intensity', size=(15, 1)),
                sg.Text('Given Intensity : ',
                        font='Default 14'),
                sg.Input(key='-INTENSITY-', size=(10, 1)),
                sg.Text(' Reset Depth Range: ',
                        font='Default 14'),
                sg.Combo(['Near', 'Mid', 'Far'],
                         enable_events=False, key='range_combo'),
                sg.Button('Set')
            ]
        ]

        # create the window and show it without the plot
        self.window = sg.Window('SingleZenseDetector', layout, location=(800, 200),
                                no_titlebar=False, grab_anywhere=True,
                                right_click_menu=['&Right', ['E&xit']], )  # if trying Qt, you will need to remove this right click menu

        # Initial drawing
        _, _ = self.window.read(timeout=5)
        self.depth_range = drange_dict[-1]
        self.laser_intensity = self.zense_mng.laser_intensity
        self.window['drange'].update(self.depth_range)
        self.window['intensity'].update(self.laser_intensity)
        self.update_rgbd_ir()
        self.draw()

    def update_rgbd_ir(self):
        status = self.zense_mng.update()
        self.depth_range = self.zense_mng.depth_range
        self.laser_intensity = self.zense_mng.laser_intensity
        frame = np.zeros((IMAGE_HEIGHT * 2, IMAGE_WIDTH * 2, 3), np.uint8)
        if status:
            rgb_img = self.zense_mng.rgb_image
            ir_img_normalized = self.zense_mng.ir_image_normalized
            depth_img_colorized = self.zense_mng.depth_image_colorized

            rgb_img_resized = cv2.resize(
                rgb_img, (int(IMAGE_HEIGHT/9*16), IMAGE_HEIGHT))
            ir_img_resized = cv2.resize(ir_img_normalized,
                                        (IMAGE_WIDTH, IMAGE_HEIGHT))
            depth_img_resized = cv2.resize(depth_img_colorized,
                                           (IMAGE_WIDTH, IMAGE_HEIGHT))

            frame[0: IMAGE_HEIGHT, 0: int(
                IMAGE_HEIGHT/9*16), :] = rgb_img_resized
            frame[IMAGE_HEIGHT: IMAGE_HEIGHT * 2,
                  0:IMAGE_WIDTH, :] = depth_img_resized
            frame[IMAGE_HEIGHT: IMAGE_HEIGHT * 2,
                  IMAGE_WIDTH: IMAGE_WIDTH * 2, :] = ir_img_resized
        self.draw_img = frame
        self.window['drange'].update(self.depth_range)
        self.window['intensity'].update(self.laser_intensity)

    def draw(self):
        imgbytes = cv2.imencode('.png', self.draw_img)[1].tobytes()
        self.window['-IMAGE-'].update(data=imgbytes)

    def update(self):
        event, values = self.window.read(timeout=5)
        if event in ('Exit', sg.WIN_CLOSED):
            return False
        elif event == 'Set Laser Intensity':
            given_intensity = values['-INTENSITY-']
            if given_intensity == None:
                print("No string is inputted in text box.")
                return True
            if not str.isnumeric(given_intensity):
                print("Inputted string is not numeric.")
                return True
            given_intensity = int(given_intensity)
            if given_intensity <= 0:
                print("Inputted number have to be > 0")
                return True
            self.given_intensity = given_intensity
            self.zense_mng.set_laser_intensity(self.given_intensity)
            return True
        elif event == 'Set':
            given_depth_range = values["range_combo"]
            self.zense_mng.set_depth_range(given_depth_range)
            return True
        else:
            self.update_rgbd_ir()
            self.draw()
            return True

    def run(self):
        while self.update():
            pass
        self.window.close()


def main():
    is_rgbdir_enabled()
    gui_mng = RGBD_IR_GUIManager()
    gui_mng.run()


if __name__ == "__main__":
    main()
