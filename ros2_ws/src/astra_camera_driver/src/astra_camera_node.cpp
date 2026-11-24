#include <OpenNI.h>

#include <camera_info_manager/camera_info_manager.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifndef OPENNI2_REDIST_DEFAULT
#define OPENNI2_REDIST_DEFAULT ""
#endif

class AstraCameraNode final : public rclcpp::Node {
 public:
  AstraCameraNode() : Node("astra_camera_node") {
    video_device_ = declare_parameter<std::string>("video_device", "/dev/video0");
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    fps_ = declare_parameter<double>("fps", 30.0);
    rgb_format_ = declare_parameter<std::string>("rgb_format", "MJPG");
    request_registration_ = declare_parameter<bool>("depth_to_color", true);
    depth_mirror_ = declare_parameter<bool>("depth_mirror", false);
    sync_tolerance_ms_ = declare_parameter<double>("sync_tolerance_ms", 60.0);
    color_frame_id_ = declare_parameter<std::string>(
        "color_frame_id", "camera_color_optical_frame");
    depth_frame_id_ = declare_parameter<std::string>(
        "depth_frame_id", "camera_depth_optical_frame");

    const auto color_topic = declare_parameter<std::string>(
        "color_topic", "/camera/color/image_raw");
    const auto color_info_topic = declare_parameter<std::string>(
        "color_info_topic", "/camera/color/camera_info");
    const auto depth_topic = declare_parameter<std::string>(
        "depth_topic", "/camera/depth/image_raw");
    const auto depth_info_topic = declare_parameter<std::string>(
        "depth_info_topic", "/camera/depth/camera_info");
    const auto color_info_url = declare_parameter<std::string>(
        "color_camera_info_url", "");
    const auto depth_info_url = declare_parameter<std::string>(
        "depth_camera_info_url", "");
    color_camera_matrix_ = declare_parameter<std::vector<double>>(
        "color_camera_matrix", std::vector<double>{});
    color_distortion_coefficients_ = declare_parameter<std::vector<double>>(
        "color_distortion_coefficients", std::vector<double>{});
    depth_camera_matrix_ = declare_parameter<std::vector<double>>(
        "depth_camera_matrix", std::vector<double>{});
    depth_distortion_coefficients_ = declare_parameter<std::vector<double>>(
        "depth_distortion_coefficients", std::vector<double>{});
    // Expose the SDK's Depth -> RGB transform for consumers that need to
    // publish/use the corresponding TF. Translation remains in SDK units.
    depth_to_color_rotation_ = declare_parameter<std::vector<double>>(
        "depth_to_color_rotation", std::vector<double>{});
    depth_to_color_translation_sdk_units_ = declare_parameter<std::vector<double>>(
        "depth_to_color_translation_sdk_units", std::vector<double>{});

    if (width_ <= 0 || height_ <= 0 || fps_ <= 0.0 || sync_tolerance_ms_ < 0.0) {
      throw std::invalid_argument("width, height, and fps must be positive; sync_tolerance_ms cannot be negative");
    }
    if (rgb_format_.size() != 4) {
      throw std::invalid_argument("rgb_format must be a four-character V4L2 format such as MJPG or YUYV");
    }
    validateCalibration(color_camera_matrix_, color_distortion_coefficients_, "color");
    validateCalibration(depth_camera_matrix_, depth_distortion_coefficients_, "depth");
    if (!depth_to_color_rotation_.empty() && depth_to_color_rotation_.size() != 9) {
      throw std::invalid_argument("depth_to_color_rotation must contain 9 row-major values");
    }
    if (!depth_to_color_translation_sdk_units_.empty() &&
        depth_to_color_translation_sdk_units_.size() != 3) {
      throw std::invalid_argument("depth_to_color_translation_sdk_units must contain 3 values");
    }

    color_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this, "astra_color");
    depth_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this, "astra_depth");
    if (!color_info_url.empty() && !color_info_manager_->loadCameraInfo(color_info_url)) {
      RCLCPP_WARN(get_logger(), "Could not load color calibration from '%s'", color_info_url.c_str());
    }
    if (!depth_info_url.empty() && !depth_info_manager_->loadCameraInfo(depth_info_url)) {
      RCLCPP_WARN(get_logger(), "Could not load depth calibration from '%s'", depth_info_url.c_str());
    }
    if (!color_info_manager_->isCalibrated() && color_camera_matrix_.empty()) {
      RCLCPP_WARN(get_logger(), "RGB CameraInfo is uncalibrated (K[0] = 0); set color_camera_info_url to a calibration YAML");
    }
    if (!depth_info_manager_->isCalibrated() && depth_camera_matrix_.empty()) {
      RCLCPP_WARN(get_logger(), "Depth CameraInfo is uncalibrated (K[0] = 0); set depth_camera_info_url to a calibration YAML");
    }

    const auto qos = rclcpp::SensorDataQoS();
    color_pub_ = create_publisher<sensor_msgs::msg::Image>(color_topic, qos);
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(color_info_topic, qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic, qos);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(depth_info_topic, qos);
    openOpenNI();
    openColorCamera();
    running_.store(true);
    capture_thread_ = std::thread(&AstraCameraNode::captureLoop, this);
  }

  ~AstraCameraNode() override {
    running_.store(false);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (color_capture_.isOpened()) {
      color_capture_.release();
    }
    if (depth_stream_.isValid()) {
      depth_stream_.stop();
      depth_stream_.destroy();
    }
    if (device_.isValid()) {
      device_.close();
    }
    if (openni_initialized_) {
      openni::OpenNI::shutdown();
    }
  }

 private:
  using SteadyClock = std::chrono::steady_clock;

  void openOpenNI() {
    if (OPENNI2_REDIST_DEFAULT[0] != '\0') {
      setenv("OPENNI2_REDIST", OPENNI2_REDIST_DEFAULT, 0);
    }
    const auto status = openni::OpenNI::initialize();
    if (status != openni::STATUS_OK) {
      throw std::runtime_error(std::string("OpenNI initialize failed: ") +
                               openni::OpenNI::getExtendedError());
    }
    openni_initialized_ = true;

    if (device_.open(openni::ANY_DEVICE) != openni::STATUS_OK) {
      throw std::runtime_error(std::string("Could not open Astra/OpenNI device: ") +
                               openni::OpenNI::getExtendedError());
    }
    if (!device_.getSensorInfo(openni::SENSOR_DEPTH)) {
      throw std::runtime_error("OpenNI device does not expose a depth sensor");
    }

    if (request_registration_ && device_.isImageRegistrationModeSupported(
                                     openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
      const auto registration_status = device_.setImageRegistrationMode(
          openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
      registration_active_ = registration_status == openni::STATUS_OK &&
          device_.getImageRegistrationMode() == openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR;
    }
    if (request_registration_ && !registration_active_) {
      RCLCPP_WARN(get_logger(), "Depth-to-color registration is unsupported; publishing native depth coordinates");
    } else if (registration_active_) {
      RCLCPP_INFO(get_logger(), "Depth-to-color registration enabled");
    }

    if (depth_stream_.create(device_, openni::SENSOR_DEPTH) != openni::STATUS_OK) {
      throw std::runtime_error(std::string("Could not create OpenNI depth stream: ") +
                               openni::OpenNI::getExtendedError());
    }
    if (depth_stream_.setMirroringEnabled(depth_mirror_) != openni::STATUS_OK) {
      throw std::runtime_error(std::string("Could not set OpenNI depth mirroring: ") +
                               openni::OpenNI::getExtendedError());
    }
    if (depth_stream_.start() != openni::STATUS_OK) {
      throw std::runtime_error(std::string("Could not start OpenNI depth stream: ") +
                               openni::OpenNI::getExtendedError());
    }
  }

  void openColorCamera() {
    color_capture_.open(video_device_, cv::CAP_V4L2);
    if (!color_capture_.isOpened()) {
      throw std::runtime_error("Could not open RGB device " + video_device_);
    }
    color_capture_.set(cv::CAP_PROP_FOURCC,
                       cv::VideoWriter::fourcc(rgb_format_[0], rgb_format_[1],
                                               rgb_format_[2], rgb_format_[3]));
    color_capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    color_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    color_capture_.set(cv::CAP_PROP_FPS, fps_);
    color_capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    RCLCPP_INFO(get_logger(), "Opened RGB %s at requested %dx%d %.1f FPS (%s)",
                video_device_.c_str(), width_, height_, fps_, rgb_format_.c_str());
  }

  static void validateCalibration(const std::vector<double>& matrix,
                                  const std::vector<double>& distortion,
                                  const char* stream_name) {
    if (matrix.empty() != distortion.empty()) {
      throw std::invalid_argument(std::string(stream_name) +
                                  " camera matrix and distortion coefficients must both be set");
    }
    if (!matrix.empty() && matrix.size() != 9) {
      throw std::invalid_argument(std::string(stream_name) +
                                  " camera matrix must contain 9 row-major values");
    }
    if (!distortion.empty() && distortion.size() != 5) {
      throw std::invalid_argument(std::string(stream_name) +
                                  " distortion coefficients must contain 5 values [k1,k2,p1,p2,k3]");
    }
  }

  sensor_msgs::msg::CameraInfo cameraInfo(
      camera_info_manager::CameraInfoManager& manager,
      const std_msgs::msg::Header& header, int width, int height,
      bool color_calibration) {
    auto info = manager.getCameraInfo();
    const auto& matrix = color_calibration ? color_camera_matrix_ : depth_camera_matrix_;
    const auto& distortion = color_calibration
        ? color_distortion_coefficients_ : depth_distortion_coefficients_;
    if (!matrix.empty()) {
      info.width = static_cast<std::uint32_t>(width);
      info.height = static_cast<std::uint32_t>(height);
      info.distortion_model = "plumb_bob";
      std::copy(matrix.begin(), matrix.end(), info.k.begin());
      info.d = distortion;
      info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      info.p.fill(0.0);
      info.p[0] = matrix[0];
      info.p[2] = matrix[2];
      info.p[5] = matrix[4];
      info.p[6] = matrix[5];
      info.p[10] = 1.0;
    } else if (!manager.isCalibrated()) {
      info.width = static_cast<std::uint32_t>(width);
      info.height = static_cast<std::uint32_t>(height);
      info.distortion_model = "plumb_bob";
      info.k.fill(0.0);
      info.d.clear();
      info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      info.p.fill(0.0);
    } else if (info.width != static_cast<std::uint32_t>(width) ||
               info.height != static_cast<std::uint32_t>(height)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "CameraInfo calibration resolution differs from image resolution; verify calibration YAML");
    }
    info.header = header;
    return info;
  }

  bool readDepth(cv::Mat& depth_mm, SteadyClock::time_point& received_at) {
    openni::VideoStream* streams[] = {&depth_stream_};
    int ready_index = -1;
    const auto status = openni::OpenNI::waitForAnyStream(
        streams, 1, &ready_index, 1000);
    if (status != openni::STATUS_OK) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for OpenNI depth frame failed: %s",
                           openni::OpenNI::getExtendedError());
      return false;
    }

    openni::VideoFrameRef frame;
    if (depth_stream_.readFrame(&frame) != openni::STATUS_OK || !frame.isValid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Reading OpenNI depth frame failed: %s",
                           openni::OpenNI::getExtendedError());
      return false;
    }
    received_at = SteadyClock::now();
    const int width = frame.getWidth();
    const int height = frame.getHeight();
    const int stride = frame.getStrideInBytes();
    if (width <= 0 || height <= 0 || stride < width * 2) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "OpenNI returned invalid depth dimensions or stride");
      return false;
    }

    const auto pixel_format = frame.getVideoMode().getPixelFormat();
    if (pixel_format != openni::PIXEL_FORMAT_DEPTH_1_MM &&
        pixel_format != openni::PIXEL_FORMAT_DEPTH_100_UM) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Unsupported OpenNI depth pixel format: %d",
                            static_cast<int>(pixel_format));
      return false;
    }

    depth_mm.create(height, width, CV_16UC1);
    const auto* bytes = static_cast<const std::uint8_t*>(frame.getData());
    for (int y = 0; y < height; ++y) {
      const auto* source = reinterpret_cast<const uint16_t*>(bytes + y * stride);
      auto* destination = depth_mm.ptr<uint16_t>(y);
      if (pixel_format == openni::PIXEL_FORMAT_DEPTH_1_MM) {
        std::copy(source, source + width, destination);
      } else {
        for (int x = 0; x < width; ++x) {
          destination[x] = static_cast<uint16_t>(std::lround(source[x] * 0.1));
        }
      }
    }
    return true;
  }

  void captureLoop() {
    while (running_.load() && rclcpp::ok()) {
      cv::Mat depth_mm;
      SteadyClock::time_point depth_received_at;
      if (!readDepth(depth_mm, depth_received_at)) {
        continue;
      }

      cv::Mat color_bgr;
      if (!color_capture_.read(color_bgr) || color_bgr.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "No RGB frame received from %s", video_device_.c_str());
        continue;
      }
      const auto color_received_at = SteadyClock::now();
      const auto skew = std::chrono::duration<double, std::milli>(
          color_received_at - depth_received_at).count();
      if (skew > sync_tolerance_ms_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Dropping RGB/depth pair with %.1f ms receive-time skew (limit %.1f ms)",
                             skew, sync_tolerance_ms_);
        continue;
      }

      if (registration_active_ && depth_mm.cols != color_bgr.cols) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Depth-to-color mode is active but RGB/depth widths differ (%d vs %d)",
                             color_bgr.cols, depth_mm.cols);
      }
      const bool aligned = registration_active_ &&
          depth_mm.cols == color_bgr.cols && depth_mm.rows == color_bgr.rows;
      const auto stamp = now();
      std_msgs::msg::Header color_header;
      color_header.stamp = stamp;
      color_header.frame_id = color_frame_id_;
      std_msgs::msg::Header depth_header;
      depth_header.stamp = stamp;
      depth_header.frame_id = aligned ? color_frame_id_ : depth_frame_id_;

      const auto color_message = cv_bridge::CvImage(
          color_header, sensor_msgs::image_encodings::BGR8, color_bgr).toImageMsg();
      const auto depth_message = cv_bridge::CvImage(
          depth_header, sensor_msgs::image_encodings::TYPE_16UC1, depth_mm).toImageMsg();
      auto color_info = cameraInfo(*color_info_manager_, color_header,
                                   color_bgr.cols, color_bgr.rows, true);
      auto depth_info = aligned
          ? cameraInfo(*color_info_manager_, depth_header, depth_mm.cols,
                       depth_mm.rows, true)
          : cameraInfo(*depth_info_manager_, depth_header, depth_mm.cols,
                       depth_mm.rows, false);

      color_pub_->publish(*color_message);
      color_info_pub_->publish(color_info);
      depth_pub_->publish(*depth_message);
      depth_info_pub_->publish(depth_info);
    }
  }

  std::string video_device_;
  std::string rgb_format_;
  std::string color_frame_id_;
  std::string depth_frame_id_;
  std::vector<double> color_camera_matrix_;
  std::vector<double> color_distortion_coefficients_;
  std::vector<double> depth_camera_matrix_;
  std::vector<double> depth_distortion_coefficients_;
  std::vector<double> depth_to_color_rotation_;
  std::vector<double> depth_to_color_translation_sdk_units_;
  int width_{640};
  int height_{480};
  double fps_{30.0};
  double sync_tolerance_ms_{60.0};
  bool request_registration_{true};
  bool registration_active_{false};
  bool depth_mirror_{false};
  bool openni_initialized_{false};

  openni::Device device_;
  openni::VideoStream depth_stream_;
  cv::VideoCapture color_capture_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> color_info_manager_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> depth_info_manager_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  std::atomic<bool> running_{false};
  std::thread capture_thread_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<AstraCameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    std::cerr << "astra_camera_node failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
