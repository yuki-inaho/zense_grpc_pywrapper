#include "pico_zense_server_impl.hpp"

namespace zense {

void PicoZenseServerImpl::setup(std::string cfgParamPath, std::string camKey,
                                int32_t device_index__) {
  device_index_ = device_index__;
  usleep(5 * 1e6);  // To avoid high frequent sensor open call from
                    // immediately after termination and rebooting

  if (!checkFileExistence(cfgParamPath)) {
    std::cerr << "Setting TOML file does not exist" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  ParameterManager cfgParam(cfgParamPath);

  sensor_id_ = cfgParam.readStringData("General", "sensor_id");

  if (!cfgParam.checkExistanceTable(camKey.c_str())) {
    std::cerr << "Camera name is invalid, please check setting toml file"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::string camera_name =
      cfgParam.readStringData(camKey.c_str(), "camera_name");
  serial_no_ = cfgParam.readStringData(camKey.c_str(), "serial_no");
  range1 = cfgParam.readIntData(camKey.c_str(), "range1");
  range2 = cfgParam.readIntData(camKey.c_str(), "range2");
  isRGB = cfgParam.readIntData(camKey.c_str(), "rgb_image") == 1;
  isWDR = (range1 >= 0) && (range2 >= 0);
  isIR = isRGB && !isWDR;
  // TODO: merge factory values and tuned values
  std::string camFactKey = camKey + "_Factory";

  CameraParameter camera_factory_param;
  camera_factory_param.fx = cfgParam.readFloatData(camFactKey.c_str(), "fx");
  camera_factory_param.fy = cfgParam.readFloatData(camFactKey.c_str(), "fy");
  camera_factory_param.cx = cfgParam.readFloatData(camFactKey.c_str(), "cx");
  camera_factory_param.cy = cfgParam.readFloatData(camFactKey.c_str(), "cy");
  camera_factory_param.p1 = cfgParam.readFloatData(camFactKey.c_str(), "p1");
  camera_factory_param.p2 = cfgParam.readFloatData(camFactKey.c_str(), "p2");
  camera_factory_param.k1 = cfgParam.readFloatData(camFactKey.c_str(), "k1");
  camera_factory_param.k2 = cfgParam.readFloatData(camFactKey.c_str(), "k2");
  camera_factory_param.k3 = cfgParam.readFloatData(camFactKey.c_str(), "k3");
  camera_factory_param.k4 = cfgParam.readFloatData(camFactKey.c_str(), "k4");
  camera_factory_param.k5 = cfgParam.readFloatData(camFactKey.c_str(), "k5");
  camera_factory_param.k6 = cfgParam.readFloatData(camFactKey.c_str(), "k6");

  std::string id_str;
  std::cout << "Serial number allocated to Zense Publisher : " << serial_no_
            << std::endl;

  std::string distortionKey = camKey + "_Undistortion";

  // If TOML configuration file doesn't contain any undistortion
  // description(table), undistortion process will be skip
  if (cfgParam.checkExistanceTable(distortionKey)) {
    undistortion_flag = cfgParam.readBoolData(distortionKey, "flag");
  } else {
    undistortion_flag = false;
  }
  if (undistortion_flag == true) {
    undistorter = PicoZenseUndistorter(cfgParam, distortionKey);
  }

  manager_.openDevice(device_index_);
  if (!manager_.setupDevice(device_index_, range1, range2, isRGB)) {
    close();
    std::cerr << "Could not setup device" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  if (range2 < 0) range2 = range1;

  camera_param = manager_.getCameraParameter(device_index_, 0);
  if (!(isWithinError(camera_param.k5, camera_factory_param.k5) &&
        isWithinError(camera_param.k6, camera_factory_param.k6))) {
    close();
    std::cerr << "Erroneous internal parameters. Exiting..." << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (!manager_.startDevice(device_index_)) {
    close();
    std::cerr << "Could not start device" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::string ns = "/" + camera_name;
  std::cout << "Camera setup is finished!" << std::endl;
}

void PicoZenseServerImpl::close() { manager_.closeDevice(device_index_); }

int PicoZenseServerImpl::getDepthRange() { return depth_range1; }

std::vector<int> PicoZenseServerImpl::getDepthRangeWDR() {
  return std::vector<int>{(int)depth_range1, (int)depth_range2};
}

bool PicoZenseServerImpl::monitoring_skip() {
  skip_counter_[range1]++;
  skip_counter_[range2]++;
  if (skip_counter_[range1] > MAX_SKIP_COUNTER ||
      skip_counter_[range2] > MAX_SKIP_COUNTER) {
    close();
    std::cerr << "Device not responding. Exiting..." << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (!manager_.updateDevice(device_index_)) {
    std::cout << "Device not updated. Skipping..." << std::endl;
    usleep(33333);
    return false;
  }
  return true;
}

template <>
bool PicoZenseServerImpl::_update<ZenseMode::RGBD>() {
  bool is_success = true;
  if (!monitoring_skip()) return false;
  rgb_image = manager_.getRgbImage(device_index_).clone();
  depth_range1 = (DepthRange)manager_.getDepthRange(device_index_);
  depth_image_range1 = manager_.getDepthImage(device_index_).clone();
  if (rgb_image.cols == 0 || depth_image_range1.cols == 0) is_success = false;
  skip_counter_[depth_range1] = 0;
  return is_success;
}

template <>
bool PicoZenseServerImpl::_update<ZenseMode::RGBDIR>() {
  bool is_success = true;
  if (!monitoring_skip()) return false;
  rgb_image = manager_.getRgbImage(device_index_).clone();
  ir_image = manager_.getIRImage(device_index_).clone();
  depth_image_range1 = manager_.getDepthImage(device_index_).clone();
  depth_range1 = (DepthRange)manager_.getDepthRange(device_index_);
  flag_wdr_range_updated_[depth_range1];
  if (is_success && (ir_image.cols == 0 || depth_image_range1.cols == 0))
    is_success = false;
  skip_counter_[depth_range1] = 0;
  return is_success;
}

template <>
bool PicoZenseServerImpl::_update<ZenseMode::WDR>() {
  bool is_success = true;
  if (!monitoring_skip()) return false;
  DepthRange _depth_range = (DepthRange)manager_.getDepthRange(device_index_);
  cv::Mat _depth_image = manager_.getDepthImage(device_index_).clone();
  if (_depth_range == range1) {
    depth_range1 = _depth_range;
    depth_image_range1 = _depth_image;
  } else if (_depth_range == range2) {
    depth_range2 = _depth_range;
    depth_image_range2 = _depth_image;
  } else {
    throw std::runtime_error("Unconfigurated depth informaiton aquired");
  }

  if (_depth_image.cols == 0) {
    is_success = false;
  }else{
    flag_wdr_range_updated_[_depth_range] = true;
  }
  skip_counter_[_depth_range] = 0;

  // if only double range depth image are updated, return true
  bool is_success_wdr;
  is_success_wdr = flag_wdr_range_updated_[range1] &&
                   flag_wdr_range_updated_[range2];
  if (is_success_wdr) {
    // if double depth info is correctly updated, reflesh
    flag_wdr_range_updated_[range1] = false;
    flag_wdr_range_updated_[range2] = false;
  }

  if(is_success_wdr && !is_success){
    throw "Undefined situation";
  }

  return is_success_wdr;
}

bool PicoZenseServerImpl::update() {
  // ToDo: Check coverage
  bool status = false;

  while (!status) {
    if (isRGB) {
      if (isIR) {
        status = _update<ZenseMode::RGBDIR>();
      } else {
        status = _update<ZenseMode::RGBD>();
      }
    } else {
      if (isWDR) {
        status = _update<ZenseMode::WDR>();
      } else {
        status = _update<ZenseMode::DepthIR>();
      }
    }
  }

  return status;
}

}  // namespace zense