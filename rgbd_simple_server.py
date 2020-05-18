from zense_grpc_pywrapper import PicoZenseGRPCServerImpl

cfg_param_path = "./cfg/camera.toml".encode('utf-8')
cam_key = "Camera0".encode('utf-8')
zense = PicoZenseGRPCServerImpl(cfg_param_path, cam_key, 0)


zense.update()
is_rgb = zense.is_rgb
rgb_img = zense.rgb_image
