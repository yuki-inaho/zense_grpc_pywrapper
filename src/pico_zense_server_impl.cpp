#include "pico_zense_server_impl.hpp"

/*
template <typename response_type>
class PicoZenseAsyncCall {
 public:
  PicoZenseAsyncCall(ZenseService::AsyncService *service,
                     ServerCompletionQueue *cq)
      : service_(service), cq_(cq), responder_(&ctx_), responder_rgbdir_(&ctx_),
status_(CREATE) { Proceed();
  }

  void SetZenseManager(std::shared_ptr<PicoZenseManager> &manager__) {
    manager_ = manager__;
  }
  uint64_t getTimeStamp() {
    uint64_t microsecondsUTC =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    return microsecondsUTC;
  }

  void SetRGBImage(const cv::Mat &__rgb_image);
  void SetIRImage(const cv::Mat &__ir_image);
  void SetDepthImage(const cv::Mat &__depth_image);
  template <typename ImageReply>
  void SetRGBImageReply(const ImageReply &reply);
  template <typename ImageReply>
  void SetDepthImageReply(const ImageReply &reply);
  template <typename ImageReply>
  void SetIRImageReply(const ImageReply &reply);

  Status SendRGBDImage(ServerContext *context, const ImageRequest *request,
                       ImageRGBDReply *reply);
  Status SendRGBDIRImage(ServerContext *context, const ImageRequest *request,
                         ImageRGBDIRReply *reply);

  void Proceed(void){};
  void Proceed2(void) {
    if (status_ == CREATE) {
      status_ = PROCESS;
      service_->RequestSendRGBDIRImage(&ctx_, &request_, &responder_rgbdir_,
cq_, cq_, this); } else if (status_ == PROCESS) { if (!manager_) { throw
std::domain_error("Null Pointer Exception (Zense Manager)");
      }
      new PicoZenseAsyncCall<ImageRGBDIRReply>(service_, cq_);
      SendRGBDIRImage(&ctx_, &request_, &reply_rgbdir_);
      status_ = FINISH;
      responder_rgbdir_.Finish(reply_rgbdir_, Status::OK, this);
    } else {
      std::cout << "fuck you" << std::endl;
      GPR_ASSERT(status_ == FINISH);
      delete this;
    }
  };

 private:
  std::shared_ptr<PicoZenseManager> manager_;
  ZenseService::AsyncService *service_;
  ServerCompletionQueue *cq_;
  ServerContext ctx_;
  ImageRequest request_;
  ImageRGBDReply reply_rgbd_;
  ImageRGBDIRReply reply_rgbdir_;

  cv::Mat _rgb_image;
  cv::Mat _ir_image;
  cv::Mat _depth_image_range1;
  cv::Mat _depth_image_range2;

  // The means to get back to the client.
  ServerAsyncResponseWriter<response_type> responder_;
  ServerAsyncResponseWriter<ImageRGBDIRReply> responder_rgbdir_;

  // Let's implement a tiny state machine with the following states.
  enum CallStatus { CREATE, PROCESS, FINISH };
  CallStatus status_;  // The current serving state.
};

template <>
void PicoZenseAsyncCall<ImageRGBDReply>::Proceed(void) {
  if (status_ == CREATE) {
    status_ = PROCESS;
    service_->RequestSendRGBDImage(&ctx_, &request_, &responder_, cq_, cq_,
                                     this);
  } else if (status_ == PROCESS) {
    if (!manager_) {
      throw std::domain_error("Null Pointer Exception (Zense Manager)");
    }
    new PicoZenseAsyncCall<ImageRGBDReply>(service_, cq_);
    SendRGBDImage(&ctx_, &request_, &reply_rgbd_);
    status_ = FINISH;
    responder_.Finish(reply_rgbd_, Status::OK, this);
  } else {
    GPR_ASSERT(status_ == FINISH);
    delete this;
  }
};

template <>
void PicoZenseAsyncCall<ImageRGBDIRReply>::Proceed(void) {
  if (status_ == CREATE) {
    status_ = PROCESS;
    service_->RequestSendRGBDIRImage(&ctx_, &request_, &responder_, cq_, cq_,
                                     this);
  } else if (status_ == PROCESS) {
    if (!manager_) {
      throw std::domain_error("Null Pointer Exception (Zense Manager)");
    }
    new PicoZenseAsyncCall<ImageRGBDIRReply>(service_, cq_);
    SendRGBDIRImage(&ctx_, &request_, &reply_rgbdir_);
    status_ = FINISH;
    responder_.Finish(reply_rgbdir_, Status::OK, this);
  } else {
    GPR_ASSERT(status_ == FINISH);
    delete this;
  }
};

template <typename response_type>
void PicoZenseAsyncCall<response_type>::SetRGBImage(
    const cv::Mat &__rgb_image) {
  _rgb_image = __rgb_image.clone();
}

template <typename response_type>
void PicoZenseAsyncCall<response_type>::SetIRImage(const cv::Mat &__ir_image) {
  _ir_image = __ir_image.clone();
}

template <typename response_type>
void PicoZenseAsyncCall<response_type>::SetDepthImage(
    const cv::Mat &__depth_image) {
  _depth_image_range1 = __depth_image.clone();
}

template <typename response_type>
template <typename ImageReply>
void PicoZenseAsyncCall<response_type>::SetRGBImageReply(
    const ImageReply &reply) {
  int width = _rgb_image.cols;
  int height = _rgb_image.rows;
  int channel = _rgb_image.channels();

  int image_size = _rgb_image.total() * _rgb_image.elemSize();
  char *image_char = new char[image_size];
  std::memcpy(image_char, _rgb_image.data, image_size * sizeof(char));
  std::string data_rgb_str(reinterpret_cast<char const *>(image_char),
                           image_size);

  zense::Image *rgb_image_pb = new zense::Image();
  rgb_image_pb->set_width(width);
  rgb_image_pb->set_height(height);
  rgb_image_pb->set_channel(channel);
  rgb_image_pb->set_data(data_rgb_str);
  reply.set_allocated_rgb_image(rgb_image_pb);
}

template <typename response_type>
template <typename ImageReply>
void PicoZenseAsyncCall<response_type>::SetIRImageReply(
    const ImageReply &reply) {
  int width = _ir_image.cols;
  int height = _ir_image.rows;
  int channel = _ir_image.channels();

  int image_size = _ir_image.total() * _ir_image.elemSize();
  char *image_char = new char[image_size];
  std::memcpy(image_char, _ir_image.data, image_size * sizeof(char));
  std::string data_ir_str(reinterpret_cast<char const *>(image_char),
                          image_size);

  zense::Image *ir_image_pb = new zense::Image();
  ir_image_pb->set_width(width);
  ir_image_pb->set_height(height);
  ir_image_pb->set_channel(channel);
  ir_image_pb->set_data(data_ir_str);
  reply.set_allocated_ir_image(ir_image_pb);
}

template <typename response_type>
template <typename ImageReply>
void PicoZenseAsyncCall<response_type>::SetDepthImageReply(
    const ImageReply &reply) {
  int width = _depth_image_range1.cols;
  int height = _depth_image_range1.rows;
  int channel = _depth_image_range1.channels();

  int image_size = _depth_image_range1.total() * _depth_image_range1.elemSize();
  char *image_char = new char[image_size];
  std::memcpy(image_char, _depth_image_range1.data, image_size * sizeof(char));
  std::string data_depth_str(reinterpret_cast<char const *>(image_char),
                             image_size);

  zense::Image *depth_image_pb = new zense::Image();
  depth_image_pb->set_width(width);
  depth_image_pb->set_height(height);
  depth_image_pb->set_channel(channel);
  depth_image_pb->set_data(data_depth_str);
  reply.set_allocated_ir_image(depth_image_pb);
}

void PicoZenseAsyncCall::_setWDRDepthImageReply(ImageWDRReply *reply,
                                                int &width, int &height,
                                                int &channel, int &image_size,
                                                std::string &data_depth_str,
                                                bool is_first_range) {
  char *image_char;
  if (is_first_range) {
    width = depth_image_range1.cols;
    height = depth_image_range1.rows;
    channel = depth_image_range1.channels();
    image_size = depth_image_range1.total() * depth_image_range1.elemSize();
    image_char = new char[image_size];
    std::memcpy(image_char, depth_image_range1.data, image_size *
sizeof(char)); } else { width = depth_image_range2.cols; height =
depth_image_range2.rows; channel = depth_image_range2.channels(); image_size =
depth_image_range2.total() * depth_image_range2.elemSize(); image_char = new
char[image_size]; std::memcpy(image_char, depth_image_range2.data, image_size
* sizeof(char));
  }
  std::string _data_depth_str(reinterpret_cast<char const *>(image_char),
                              image_size);
  data_depth_str = _data_depth_str;
  zense::Image *depth_image_pb = new zense::Image();
  depth_image_pb->set_width(width);
  depth_image_pb->set_height(height);
  depth_image_pb->set_channel(channel);
  depth_image_pb->set_data(data_depth_str);
  if (is_first_range) {
    reply_.set_allocated_depth_image_range1(depth_image_pb);
  } else {
    reply_.set_allocated_depth_image_range1(depth_image_pb);
  }
}

void PicoZenseAsyncCall::SetWDRDepthImageReply(ImageWDRReply *reply) {
  int width_r1, height_r1, channel_r1, image_size_r1;
  std::string data_depth_str_r1;
  _setWDRDepthImageReply(reply, width_r1, height_r1, channel_r1,
image_size_r1, data_depth_str_r1, true); int width_r2, height_r2, channel_r2,
image_size_r2; std::string data_depth_str_r2; _setWDRDepthImageReply(reply,
width_r2, height_r2, channel_r2, image_size_r2, data_depth_str_r2, false);
}

template <typename response_type>
Status PicoZenseAsyncCall<response_type>::SendRGBDImage(
    ServerContext *context, const ImageRequest *request,
    ImageRGBDReply *reply) {
  // If skip_counter_[*] exceeds MAX_SKIP_COUNTER in update(),
  // the sensor module will terminate itself. So, infinite loop can be avoided
  // in this process.
  int width = _rgb_image.cols;
  int height = _rgb_image.rows;
  int channel = _rgb_image.channels();

  int image_size = _rgb_image.total() * _rgb_image.elemSize();
  char *image_char = new char[image_size];
  std::memcpy(image_char, _rgb_image.data, image_size * sizeof(char));
  std::string data_rgb_str(reinterpret_cast<char const *>(image_char),
                           image_size);

  zense::Image image_rgb;
  image_rgb.set_width(width);
  image_rgb.set_height(height);
  image_rgb.set_channel(channel);
  image_rgb.set_image_size(image_size);
  image_rgb.set_data(data_rgb_str);
  image_rgb.set_timestamp(getTimeStamp());
  // reply->set_allocated_image_rgb(&image_rgb);
  reply->mutable_image_rgb()->MergeFrom(image_rgb);

  return Status::OK;
}

template <typename response_type>
Status PicoZenseAsyncCall<response_type>::SendRGBDIRImage(
    ServerContext *context, const ImageRequest *request,
    ImageRGBDIRReply *reply) {
  // If skip_counter_[*] exceeds MAX_SKIP_COUNTER in update(),
  // the sensor module will terminate itself. So, infinite loop can be avoided
  // in this process.
  int width = _rgb_image.cols;
  int height = _rgb_image.rows;
  int channel = _rgb_image.channels();

  int image_size = _rgb_image.total() * _rgb_image.elemSize();
  char *image_char = new char[image_size];
  std::memcpy(image_char, _rgb_image.data, image_size * sizeof(char));
  std::string data_rgb_str(reinterpret_cast<char const *>(image_char),
                           image_size);

  zense::Image image_rgb;
  image_rgb.set_width(width);
  image_rgb.set_height(height);
  image_rgb.set_channel(channel);
  image_rgb.set_image_size(image_size);
  image_rgb.set_data(data_rgb_str);
  image_rgb.set_timestamp(getTimeStamp());
  // reply->set_allocated_image_rgb(&image_rgb);
  reply->mutable_image_rgb()->MergeFrom(image_rgb);

  return Status::OK;
}

class PicoZenseAsyncServer final {
 public:
  ~PicoZenseAsyncServer() {
    close();
    server_rgbd_->Shutdown();
    //server_rgbdir_->Shutdown();
    cq_rgbd_->Shutdown();
    cq_rgbdir_->Shutdown();
    //cq_->Shutdown();
  }

  void Setup(std::string cfgParamPath, std::string camKey, int32_t
device_index__); bool isWithinError(float val, float ref) { int32_t depth_range
= manager_->getDepthRange(device_index_); const double fract_err = 1e-5; return
(std::fabs(val - ref) <= fract_err * std::fabs(ref));
  };
  void close() { manager_->closeDevice(device_index_); };
  bool update();

  // There is no shutdown handling in this code.
  void Run() {
    //_zense_publisher =
    //    PicoZenseSimplePublisher(cfgParamPath, camKey, device_index__);
    std::string server_address_rgbd("localhost:50051");
    ServerBuilder builder_rgbd;
    builder_rgbd.AddListeningPort(server_address_rgbd,
grpc::InsecureServerCredentials());
    builder_rgbd.RegisterService(&service_rgbd_);
    cq_rgbd_ = builder_rgbd.AddCompletionQueue();
    cq_rgbdir_ = builder_rgbd.AddCompletionQueue();
    server_rgbd_ = builder_rgbd.BuildAndStart();


    std::string server_address_rgbdir("localhost:50052");
    ServerBuilder builder_rgbdir;
    builder_rgbdir.AddListeningPort(server_address_rgbdir,
grpc::InsecureServerCredentials());
    builder_rgbdir.RegisterService(&service_rgbdir_);
    cq_rgbdir_ = builder_rgbdir.AddCompletionQueue();
    server_rgbdir_ = builder_rgbdir.BuildAndStart();


    //std::cout << "Server listening on " << server_address << std::endl;

    HandleRpcs();
  }

 private:
  // Class encompasing the state and logic needed to serve a request.
  void HandleRpcs() {
    // Spawn a new PicoZenseAsyncCall instance to serve new clients.
    while (!update()) {
      continue;
    }

    if (isWDR) {

    }else{
      if (isRGB) {
        if (isIR) {

        }
        //new PicoZenseAsyncCall<ImageRGBDIRReply>(&service_rgbd_,
cq_rgbdir_.get()); new PicoZenseAsyncCall<ImageRGBDReply>(&service_rgbd_,
cq_rgbd_.get());

      } else {

      }
    }

    void *tag_rgbd, *tag_rgbdir;
    bool ok_rgbd, ok_rgbdir;

    while (true) {
      update();
      if (isWDR) {

      } else {
        if (isRGB) {
            std::cout << "test2" << std::endl;
            GPR_ASSERT(cq_rgbdir_->Next(&tag_rgbdir, &ok_rgbdir));
            static_cast<PicoZenseAsyncCall<ImageRGBDIRReply> *>(tag_rgbdir)
                ->SetZenseManager(manager_);
            static_cast<PicoZenseAsyncCall<ImageRGBDIRReply> *>(tag_rgbdir)
                ->SetRGBImage(rgb_image);
            static_cast<PicoZenseAsyncCall<ImageRGBDIRReply> *>(tag_rgbdir)
                ->SetIRImage(ir_image);
            static_cast<PicoZenseAsyncCall<ImageRGBDIRReply> *>(tag_rgbdir)
                ->SetDepthImage(depth_image_range1);
            static_cast<PicoZenseAsyncCall<ImageRGBDIRReply> *>(tag_rgbdir)
                ->Proceed();

            std::cout << "test1" << std::endl;
            GPR_ASSERT(cq_rgbd_->Next(&tag_rgbd, &ok_rgbd));
            static_cast<PicoZenseAsyncCall<ImageRGBDReply> *>(tag_rgbd)
                ->SetZenseManager(manager_);
            static_cast<PicoZenseAsyncCall<ImageRGBDReply> *>(tag_rgbd)
                ->SetRGBImage(rgb_image);
            static_cast<PicoZenseAsyncCall<ImageRGBDReply> *>(tag_rgbd)
                ->SetDepthImage(depth_image_range1);
            static_cast<PicoZenseAsyncCall<ImageRGBDReply> *>(tag_rgbd)
                ->Proceed();
            static_cast<PicoZenseAsyncCall<ImageRGBDReply> *>(tag_rgbd)
                ->Proceed2();


        }
      }
      std::cout << rgb_image.cols << std::endl;
    }
  }

 private:
  // PicoZense custom API
  std::shared_ptr<PicoZenseManager> manager_;
  std::mutex lockData;

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

  std::shared_ptr<ServerCompletionQueue> cq_rgbd_;
  std::shared_ptr<ServerCompletionQueue> cq_rgbdir_;
  //std::shared_ptr<ServerCompletionQueue> cq_;
  ZenseService::AsyncService service_rgbd_;
  ZenseService::AsyncService service_rgbdir_;
  std::shared_ptr<Server> server_rgbd_;
  std::shared_ptr<Server> server_rgbdir_;
};

void PicoZenseAsyncServer::Setup(std::string cfgParamPath, std::string camKey,
                                 int32_t device_index__) {
  manager_ = std::shared_ptr<PicoZenseManager>(new PicoZenseManager);

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

  manager_->openDevice(device_index_);
  if (!manager_->setupDevice(device_index_, range1, range2, isRGB)) {
    close();
    std::cerr << "Could not setup device" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  if (range2 < 0) range2 = range1;

  camera_param = manager_->getCameraParameter(device_index_, 0);
  if (!(isWithinError(camera_param.k5, camera_factory_param.k5) &&
        isWithinError(camera_param.k6, camera_factory_param.k6))) {
    close();
    std::cerr << "Erroneous internal parameters. Exiting..." << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (!manager_->startDevice(device_index_)) {
    close();
    std::cerr << "Could not start device" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  std::string ns = "/" + camera_name;
  std::cout << "Camera setup is finished!" << std::endl;
}

bool PicoZenseAsyncServer::update() {
  // lockData.lock();
  skip_counter_[range1]++;
  skip_counter_[range2]++;
  if (skip_counter_[range1] > MAX_SKIP_COUNTER ||
      skip_counter_[range2] > MAX_SKIP_COUNTER) {
    close();
    std::cerr << "Device not responding. Exiting..." << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (!manager_->updateDevice(device_index_)) {
    std::cout << "Device not updated. Skipping..." << std::endl;
    usleep(33333);
    return false;
  }

  if (isRGB) {
    rgb_image = manager_->getRgbImage(device_index_).clone();
  }
  if (isIR) {
    ir_image = manager_->getIRImage(device_index_).clone();
  }
  int32_t depth_range = manager_->getDepthRange(device_index_);
  if (isWDR) {
    cv::Mat _depth_image = manager_->getDepthImage(device_index_).clone();
    if (depth_range == range1) {
      depth_image_range1 = _depth_image.clone();
      if (undistortion_flag) {
        undistorter.undistortion(depth_image_range1, camera_param, range1);
      }
    } else if (depth_range == range2) {
      depth_image_range2 = _depth_image.clone();
      if (undistortion_flag) {
        undistorter.undistortion(depth_image_range2, camera_param, range2);
      }
    } else {
      std::cerr << "Aquired depth range is invalid" << std::endl;
      close();
      std::exit(EXIT_FAILURE);
    }
  } else {
    depth_image_range1 = manager_->getDepthImage(device_index_).clone();
  }
  skip_counter_[depth_range] = 0;
  flag_wdr_range_updated_[depth_range] = true;
  // lockData.unlock();
  return true;
}
*/

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

bool PicoZenseServerImpl::update() {
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

  if (isRGB) {
    rgb_image = manager_.getRgbImage(device_index_).clone();
  }
  if (isIR) {
    ir_image = manager_.getIRImage(device_index_).clone();
  }
  int32_t depth_range = manager_.getDepthRange(device_index_);
  if (isWDR) {
    cv::Mat _depth_image = manager_.getDepthImage(device_index_).clone();
    if (depth_range == range1) {
      depth_image_range1 = _depth_image.clone();
      if (undistortion_flag) {
        undistorter.undistortion(depth_image_range1, camera_param, range1);
      }
    } else if (depth_range == range2) {
      depth_image_range2 = _depth_image.clone();
      if (undistortion_flag) {
        undistorter.undistortion(depth_image_range2, camera_param, range2);
      }
    } else {
      std::cerr << "Aquired depth range is invalid" << std::endl;
      close();
      std::exit(EXIT_FAILURE);
    }
  } else {
    depth_image_range1 = manager_.getDepthImage(device_index_).clone();
  }
  skip_counter_[depth_range] = 0;
  flag_wdr_range_updated_[depth_range] = true;
  return true;
}
}  // namespace zense