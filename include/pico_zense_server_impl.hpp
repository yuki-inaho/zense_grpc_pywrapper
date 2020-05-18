#pragma once
#include <chrono>
#include <mutex>
#include <unordered_map>

#include "common.hpp"
#include "parameter_manager.hpp"
#include "pico_zense_manager.hpp"
#include "pico_zense_undistorter.hpp"

#define INIT_SKIP_COUNTER -200
#define MAX_SKIP_COUNTER 30
#define MAX_HEARTBEAT_COUNTER 10

namespace zense {
class PicoZenseServerImpl {
 public:
  ~PicoZenseServerImpl() { close(); }

  void setup(std::string cfgParamPath, std::string camKey,
             int32_t device_index__);
  bool isWithinError(float val, float ref) {
    int32_t depth_range = manager_.getDepthRange(device_index_);
    const double fract_err = 1e-5;
    return (std::fabs(val - ref) <= fract_err * std::fabs(ref));
  };
  void close() { manager_.closeDevice(device_index_); };
  bool update();

 private:
  // PicoZense custom API
  PicoZenseManager manager_;

  int range1;
  int range2;
  int skip_counter_[3] = {INIT_SKIP_COUNTER, INIT_SKIP_COUNTER,
                          INIT_SKIP_COUNTER};
  bool flag_wdr_range_updated_[3] = {
      false, false, false};  // assumed near = 0, mid = 1, far = 2

  int32_t device_index_;
  std::string sensor_id_;
  std::string serial_no_;

  bool isRGB, isWDR, isIR;

  cv::Mat rgb_image;
  cv::Mat ir_image;
  cv::Mat depth_image_range1;
  cv::Mat depth_image_range2;

  bool undistortion_flag;
  PicoZenseUndistorter undistorter;
  CameraParameter camera_param;
};

}  // namespace zense