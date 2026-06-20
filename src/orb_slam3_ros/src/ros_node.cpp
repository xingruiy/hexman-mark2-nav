#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <orb_slam3/System.h>
#include <orb_slam3/ImuTypes.h>
#include <orb_slam3/ORBVocabulary.h>

using namespace std::chrono_literals;

namespace {

enum class SensorMode {
  MONOCULAR,
  MONOCULAR_IMU,
  STEREO,
  STEREO_IMU,
  RGBD,
  RGBD_IMU
};

SensorMode parseSensorMode(const std::string &s) {
  if (s == "monocular")     return SensorMode::MONOCULAR;
  if (s == "monocular_imu") return SensorMode::MONOCULAR_IMU;
  if (s == "stereo")        return SensorMode::STEREO;
  if (s == "stereo_imu")    return SensorMode::STEREO_IMU;
  if (s == "rgbd")          return SensorMode::RGBD;
  if (s == "rgbd_imu")      return SensorMode::RGBD_IMU;
  throw std::invalid_argument("Unknown sensor_type: " + s);
}

ORB_SLAM3::System::eSensor toOrbSensor(SensorMode m) {
  switch (m) {
    case SensorMode::MONOCULAR:     return ORB_SLAM3::System::MONOCULAR;
    case SensorMode::MONOCULAR_IMU: return ORB_SLAM3::System::IMU_MONOCULAR;
    case SensorMode::STEREO:        return ORB_SLAM3::System::STEREO;
    case SensorMode::STEREO_IMU:    return ORB_SLAM3::System::IMU_STEREO;
    case SensorMode::RGBD:          return ORB_SLAM3::System::RGBD;
    case SensorMode::RGBD_IMU:      return ORB_SLAM3::System::IMU_RGBD;
  }
  return ORB_SLAM3::System::RGBD;
}

bool isStereo(SensorMode m) {
  return m == SensorMode::STEREO || m == SensorMode::STEREO_IMU;
}

bool isRGBD(SensorMode m) {
  return m == SensorMode::RGBD || m == SensorMode::RGBD_IMU;
}

bool isIMU(SensorMode m) {
  return m == SensorMode::MONOCULAR_IMU || m == SensorMode::STEREO_IMU || m == SensorMode::RGBD_IMU;
}

}  // namespace

class SlamNode : public rclcpp::Node {
public:
  SlamNode()
  : Node("orb_slam3"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_) {

    const std::string default_voc =
        ament_index_cpp::get_package_share_directory("orb_slam3") +
        "/vocabulary/ORBvoc.bin";

    voc_file_          = declare_parameter<std::string>("vocabulary_file", default_voc);
    // If set, the final (loop-closure-corrected) trajectory is saved on shutdown
    // in TUM format, identical to the native ORB-SLAM3 examples, for benchmarking.
    trajectory_file_   = declare_parameter<std::string>("trajectory_file", "");
    world_frame_id_    = declare_parameter<std::string>("world_frame_id",  "odom");
    camera_frame_id_   = declare_parameter<std::string>("camera_frame_id", "camera_link");
    publish_tf_        = declare_parameter<bool>  ("publish_tf", false);
    image_slop_        = declare_parameter<double>("image_sync_slop", 0.03);
    tf_lookup_timeout_ = declare_parameter<double>("tf_lookup_timeout", 10.0);

    const std::string sensor_str = declare_parameter<std::string>("sensor_type", "rgbd_imu");
    try {
      sensor_mode_ = parseSensorMode(sensor_str);
    } catch (const std::invalid_argument &e) {
      RCLCPP_FATAL(get_logger(), "%s", e.what());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Sensor mode: %s", sensor_str.c_str());

    // Camera params (all modes)
    camera_fps_ = declare_parameter<int> ("camera.fps", 30);
    camera_rgb_ = declare_parameter<bool>("camera.rgb", true);

    // IMU params
    if (isIMU(sensor_mode_)) {
      imu_frame_id_               = declare_parameter<std::string>("imu_frame_id",
                                              "camera_imu_optical_frame");
      imu_noise_gyro_             = declare_parameter<double>("imu.noise_gyro", 1e-2);
      imu_noise_acc_              = declare_parameter<double>("imu.noise_acc", 1e-1);
      imu_gyro_walk_              = declare_parameter<double>("imu.gyro_walk", 1e-6);
      imu_acc_walk_               = declare_parameter<double>("imu.acc_walk",  1e-4);
      imu_frequency_              = declare_parameter<double>("imu.frequency", 200.0);
      imu_insert_kfs_when_lost_   = declare_parameter<bool>  ("imu.insert_kfs_when_lost", false);
    }

    // RGB-D params
    if (isRGBD(sensor_mode_)) {
      rgbd_depth_map_factor_ = declare_parameter<double>("rgbd.depth_map_factor", 1000.0);
    }

    // Stereo params (shared by stereo and RGBD)
    if (isStereo(sensor_mode_) || isRGBD(sensor_mode_)) {
      stereo_baseline_ = declare_parameter<double>("stereo.baseline", 0.05);
      stereo_th_depth_ = declare_parameter<double>("stereo.th_depth", 40.0);
    }
    if (isStereo(sensor_mode_)) {
      right_camera_frame_id_ = declare_parameter<std::string>(
          "right_camera_frame_id", "camera_right_optical_frame");
    }

    // ORB params (all modes)
    orb_n_features_  = declare_parameter<int>   ("orb.n_features", 1250);
    orb_scale_factor_ = declare_parameter<double>("orb.scale_factor", 1.2);
    orb_n_levels_    = declare_parameter<int>   ("orb.n_levels", 8);
    orb_ini_th_fast_ = declare_parameter<int>   ("orb.ini_th_fast", 20);
    orb_min_th_fast_ = declare_parameter<int>   ("orb.min_th_fast", 7);

    // System params (all modes)
    system_th_far_points_ = declare_parameter<double>("system.th_far_points", 0.0);

    // Callback groups
    image_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    if (isIMU(sensor_mode_)) {
      imu_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }

    // Publishers
    odom_pub_         = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    local_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/local_points", 5);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Start IMU subscription early for IMU modes
    if (isIMU(sensor_mode_)) {
      startImuSubscription();
    }

    // Start vocabulary preload
    voc_loader_ = std::thread([this]() { loadVocabularyAsync(); });

    // Subscribe to primary CameraInfo (triggers initialization)
    auto qos = rclcpp::SensorDataQoS();
    rclcpp::SubscriptionOptions info_opts;
    info_opts.callback_group = image_cb_group_;
    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "~/rgb/camera_info", qos,
        std::bind(&SlamNode::cameraInfoCallback, this, std::placeholders::_1),
        info_opts);

    RCLCPP_INFO(get_logger(),
                "Waiting for CameraInfo on %s ...",
                cam_info_sub_->get_topic_name());

    // For stereo modes, also subscribe to right CameraInfo.
    // Use a SEPARATE callback group: the left cameraInfoCallback blocks waiting for
    // right_cam_info_, so the right callback must be able to run concurrently (under
    // the MultiThreadedExecutor) or it deadlocks and stereo falls back to wrong intrinsics.
    if (isStereo(sensor_mode_)) {
      right_info_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions right_opts;
      right_opts.callback_group = right_info_cb_group_;
      right_cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
          "~/right/camera_info", qos,
          std::bind(&SlamNode::rightCameraInfoCallback, this, std::placeholders::_1),
          right_opts);
    }

    rclcpp::on_shutdown([this]() {
      if (slam_) {
        RCLCPP_INFO(get_logger(), "Shutting down ORB_SLAM3");
        slam_->Shutdown();
        saveTrajectory();
      }
    });
  }

  // Save the final (loop-closure-corrected) trajectory in TUM format, matching
  // the native ORB-SLAM3 examples. Must be called after slam_->Shutdown().
  // Pure-monocular has no metric full trajectory, so only keyframes are saved.
  void saveTrajectory() {
    if (traj_saved_ || trajectory_file_.empty() || !slam_) {
      return;
    }
    traj_saved_ = true;
    try {
      if (sensor_mode_ == SensorMode::MONOCULAR) {
        slam_->SaveKeyFrameTrajectoryTUM(trajectory_file_ + ".txt");
      } else {
        slam_->SaveTrajectoryTUM(trajectory_file_ + ".txt");
        slam_->SaveKeyFrameTrajectoryTUM(trajectory_file_ + "_kf.txt");
      }
      RCLCPP_INFO(get_logger(), "Saved trajectory to %s.txt", trajectory_file_.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to save trajectory: %s", e.what());
    }
  }

  ~SlamNode() override {
    if (voc_loader_.joinable()) {
      voc_loader_.join();
    }
    if (slam_) {
      slam_->Shutdown();
      saveTrajectory();
      slam_.reset();
    }
  }

private:
  static double stamp_to_sec(const builtin_interfaces::msg::Time &t) {
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
  }

  static bool endsWith(const std::string &s, const std::string &suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
  }

  // ---- Vocabulary loading ---------------------------------------------------

  void loadVocabularyAsync() {
    auto voc = std::make_unique<ORB_SLAM3::ORBVocabulary>();
    const bool useBinary = endsWith(voc_file_, ".bin");
    RCLCPP_INFO(get_logger(),
                "Pre-loading ORB vocabulary (%s) from %s ...",
                useBinary ? "binary" : "text",
                voc_file_.c_str());
    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = useBinary ? voc->loadFromBinaryFile(voc_file_)
                              : voc->loadFromTextFile(voc_file_);
    const auto t1 = std::chrono::steady_clock::now();
    if (!ok) {
      RCLCPP_FATAL(get_logger(),
                   "Failed to load ORB vocabulary from %s",
                   voc_file_.c_str());
      voc_failed_.store(true);
      return;
    }
    const double ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    RCLCPP_INFO(get_logger(),
                "ORB vocabulary loaded (%u words) in %.0f ms.",
                voc->size(), ms);
    preloaded_voc_ = std::move(voc);
    voc_ready_.store(true);
  }

  // ---- CameraInfo callbacks ------------------------------------------------

  void rightCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (right_cam_info_) return;  // already have it
    right_cam_info_ = msg;
    RCLCPP_INFO(get_logger(),
                "Got right CameraInfo: fx=%.3f fy=%.3f %ux%u frame=%s",
                msg->k[0], msg->k[4], msg->width, msg->height,
                msg->header.frame_id.c_str());
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (initialized_) return;

    if (!msg->distortion_model.empty() && msg->distortion_model != "plumb_bob") {
      RCLCPP_FATAL(get_logger(),
                   "Unsupported distortion_model='%s'; only plumb_bob (PinHole) is supported.",
                   msg->distortion_model.c_str());
      rclcpp::shutdown();
      return;
    }
    if (msg->k[0] == 0.0 || msg->k[4] == 0.0) {
      RCLCPP_FATAL(get_logger(), "CameraInfo has zero fx/fy; refusing to initialize.");
      rclcpp::shutdown();
      return;
    }
    const std::string cam_optical_frame = msg->header.frame_id;
    if (cam_optical_frame.empty()) {
      RCLCPP_FATAL(get_logger(), "CameraInfo header.frame_id is empty; cannot look up transform.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(),
                "Got CameraInfo: fx=%.3f fy=%.3f cx=%.3f cy=%.3f %ux%u frame=%s model=%s",
                msg->k[0], msg->k[4], msg->k[2], msg->k[5],
                msg->width, msg->height,
                cam_optical_frame.c_str(),
                msg->distortion_model.c_str());

    // For stereo modes, wait for right CameraInfo with timeout
    if (isStereo(sensor_mode_)) {
      const auto deadline = now() + rclcpp::Duration::from_seconds(tf_lookup_timeout_);
      while (rclcpp::ok() && now() < deadline) {
        if (right_cam_info_) break;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Waiting for right CameraInfo on %s ...",
                             right_cam_info_sub_->get_topic_name());
        std::this_thread::sleep_for(50ms);
      }
      if (!right_cam_info_) {
        RCLCPP_WARN(get_logger(),
                    "Right CameraInfo not received after %.1fs; reusing left camera intrinsics.",
                    tf_lookup_timeout_);
      }
    }

    // Build Settings object
    auto settings = std::make_unique<ORB_SLAM3::Settings>();
    settings->setSensor(toOrbSensor(sensor_mode_));
    settings->setCameraType(ORB_SLAM3::Settings::PinHole);

    // Camera1 intrinsics
    settings->setCamera1Intrinsics(
        static_cast<float>(msg->k[0]), static_cast<float>(msg->k[4]),
        static_cast<float>(msg->k[2]), static_cast<float>(msg->k[5]));
    if (msg->d.size() >= 4) {
      std::vector<float> d = {static_cast<float>(msg->d[0]), static_cast<float>(msg->d[1]),
                              static_cast<float>(msg->d[2]), static_cast<float>(msg->d[3])};
      if (msg->d.size() >= 5 && msg->d[4] != 0.0)
        d.push_back(static_cast<float>(msg->d[4]));
      settings->setCamera1Distortion(d);
    }

    // Camera2 intrinsics (stereo only)
    if (isStereo(sensor_mode_)) {
      if (right_cam_info_ && right_cam_info_->k[0] != 0.0 && right_cam_info_->k[4] != 0.0) {
        settings->setCamera2Intrinsics(
            static_cast<float>(right_cam_info_->k[0]),
            static_cast<float>(right_cam_info_->k[4]),
            static_cast<float>(right_cam_info_->k[2]),
            static_cast<float>(right_cam_info_->k[5]));
        if (right_cam_info_->d.size() >= 4) {
          std::vector<float> d = {static_cast<float>(right_cam_info_->d[0]),
                                  static_cast<float>(right_cam_info_->d[1]),
                                  static_cast<float>(right_cam_info_->d[2]),
                                  static_cast<float>(right_cam_info_->d[3])};
          if (right_cam_info_->d.size() >= 5 && right_cam_info_->d[4] != 0.0)
            d.push_back(static_cast<float>(right_cam_info_->d[4]));
          settings->setCamera2Distortion(d);
        }
      } else {
        // Reuse left camera intrinsics for right
        settings->setCamera2Intrinsics(
            static_cast<float>(msg->k[0]), static_cast<float>(msg->k[4]),
            static_cast<float>(msg->k[2]), static_cast<float>(msg->k[5]));
      }

      // Look up Tlr (left-to-right camera transform)
      {
        Sophus::SE3f Tlr;
        bool got_tlr = false;
        const auto tlr_deadline = now() + rclcpp::Duration::from_seconds(tf_lookup_timeout_);
        while (rclcpp::ok() && now() < tlr_deadline) {
          try {
            auto tf = tf_buffer_.lookupTransform(
                cam_optical_frame, right_camera_frame_id_, tf2::TimePointZero,
                tf2::durationFromSec(0.5));
            Eigen::Vector3f t(tf.transform.translation.x,
                              tf.transform.translation.y,
                              tf.transform.translation.z);
            Eigen::Quaternionf q(tf.transform.rotation.w,
                                 tf.transform.rotation.x,
                                 tf.transform.rotation.y,
                                 tf.transform.rotation.z);
            Tlr = Sophus::SE3f(q.normalized(), t);
            got_tlr = true;
            RCLCPP_INFO(get_logger(),
                        "Tlr from tf (%s -> %s): t=[%.4f %.4f %.4f] q=[%.4f %.4f %.4f %.4f]",
                        cam_optical_frame.c_str(), right_camera_frame_id_.c_str(),
                        t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
            break;
          } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for tf %s -> %s: %s",
                                 cam_optical_frame.c_str(),
                                 right_camera_frame_id_.c_str(), e.what());
          }
        }
        if (got_tlr) {
          settings->setTlr(Tlr);
        } else {
          RCLCPP_WARN(get_logger(), "Tlr not available; stereo may use default.");
        }
      }
    }

    settings->setImageInfo(static_cast<int>(msg->width), static_cast<int>(msg->height),
                           camera_fps_, camera_rgb_);

    // IMU params
    if (isIMU(sensor_mode_)) {
      Sophus::SE3f T_b_c1;
      if (!lookupTransform(imu_frame_id_, cam_optical_frame, T_b_c1, "T_b_c1")) {
        rclcpp::shutdown();
        return;
      }
      settings->setIMUParams(
          static_cast<float>(imu_noise_gyro_), static_cast<float>(imu_noise_acc_),
          static_cast<float>(imu_gyro_walk_), static_cast<float>(imu_acc_walk_),
          static_cast<float>(imu_frequency_), T_b_c1, imu_insert_kfs_when_lost_);
    }

    // RGB-D params
    if (isRGBD(sensor_mode_)) {
      settings->setRGBDParams(static_cast<float>(rgbd_depth_map_factor_));
    }

    // Stereo/RGB-D shared params
    if (isStereo(sensor_mode_) || isRGBD(sensor_mode_)) {
      settings->setStereoParams(static_cast<float>(stereo_baseline_),
                                static_cast<float>(stereo_th_depth_));
    }

    // ORB params
    settings->setORBParams(orb_n_features_, static_cast<float>(orb_scale_factor_),
                           orb_n_levels_, orb_ini_th_fast_, orb_min_th_fast_);

    // System params
    settings->setThFarPoints(static_cast<float>(system_th_far_points_));
    settings->setActiveLoopClosing(true);
    settings->initialize();

    // Wait for vocabulary preload
    while (!voc_ready_.load() && !voc_failed_.load() && rclcpp::ok()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Waiting for ORB vocabulary preload to finish ...");
      std::this_thread::sleep_for(50ms);
    }
    if (voc_failed_.load() || !preloaded_voc_) {
      RCLCPP_FATAL(get_logger(), "ORB vocabulary preload failed; aborting.");
      rclcpp::shutdown();
      return;
    }
    if (voc_loader_.joinable()) {
      voc_loader_.join();
    }

    // Construct ORB-SLAM3 System
    try {
      slam_ = std::make_unique<ORB_SLAM3::System>(
          preloaded_voc_.release(),
          voc_file_,
          settings.release(),
          toOrbSensor(sensor_mode_));
    } catch (const std::exception &e) {
      RCLCPP_FATAL(get_logger(), "ORB_SLAM3::System ctor threw: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    slam_->SetKeyFrameInsertedCallback(
        [this](const std::vector<Eigen::Vector3f> &pts) {
          publishLocalPoints(pts);
        });

    // Unsubscribe CameraInfo topics
    cam_info_sub_.reset();
    right_cam_info_sub_.reset();

    // Start image subscriptions based on modality
    startImageSubscriptions();

    initialized_ = true;
    RCLCPP_INFO(get_logger(), "ORB_SLAM3 initialized; tracking with sensor_type=%s.",
                sensor_str_from_mode(sensor_mode_));
  }

  // ---- TF lookup -----------------------------------------------------------

  bool lookupTransform(const std::string &from_frame,
                       const std::string &to_frame,
                       Sophus::SE3f &out,
                       const char *label) {
    const auto deadline = now() + rclcpp::Duration::from_seconds(tf_lookup_timeout_);
    while (rclcpp::ok() && now() < deadline) {
      try {
        auto tf = tf_buffer_.lookupTransform(
            from_frame, to_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
        Eigen::Vector3f t(tf.transform.translation.x,
                          tf.transform.translation.y,
                          tf.transform.translation.z);
        Eigen::Quaternionf q(tf.transform.rotation.w,
                             tf.transform.rotation.x,
                             tf.transform.rotation.y,
                             tf.transform.rotation.z);
        out = Sophus::SE3f(q.normalized(), t);
        RCLCPP_INFO(get_logger(),
                    "%s from tf (%s -> %s): t=[%.4f %.4f %.4f] q=[%.4f %.4f %.4f %.4f]",
                    label, from_frame.c_str(), to_frame.c_str(),
                    t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
        return true;
      } catch (const tf2::TransformException &e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waiting for tf %s -> %s: %s",
                             from_frame.c_str(), to_frame.c_str(), e.what());
      }
    }
    RCLCPP_FATAL(get_logger(),
                 "Timed out after %.1fs waiting for tf %s -> %s.",
                 tf_lookup_timeout_, from_frame.c_str(), to_frame.c_str());
    return false;
  }

  // ---- IMU -----------------------------------------------------------------

  void startImuSubscription() {
    auto qos = rclcpp::SensorDataQoS();
    rclcpp::SubscriptionOptions imu_opts;
    imu_opts.callback_group = imu_cb_group_;

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "~/imu", qos,
        std::bind(&SlamNode::imuCallback, this, std::placeholders::_1),
        imu_opts);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const double t = stamp_to_sec(msg->header.stamp);
    ORB_SLAM3::IMU::Point p(
        static_cast<float>(msg->linear_acceleration.x),
        static_cast<float>(msg->linear_acceleration.y),
        static_cast<float>(msg->linear_acceleration.z),
        static_cast<float>(msg->angular_velocity.x),
        static_cast<float>(msg->angular_velocity.y),
        static_cast<float>(msg->angular_velocity.z),
        t);
    std::lock_guard<std::mutex> lk(imu_mutex_);
    imu_buf_.push_back(p);
  }

  std::vector<ORB_SLAM3::IMU::Point> drainImuUpTo(double t_image) {
    std::vector<ORB_SLAM3::IMU::Point> out;
    std::lock_guard<std::mutex> lk(imu_mutex_);
    while (!imu_buf_.empty() && imu_buf_.front().t <= t_image) {
      out.push_back(imu_buf_.front());
      imu_buf_.pop_front();
    }
    return out;
  }

  // ---- Image subscriptions -------------------------------------------------

  void startImageSubscriptions() {
    auto qos = rclcpp::SensorDataQoS();

    if (isStereo(sensor_mode_)) {
      // Stereo: ApproximateTime sync of left + right
      stereo_left_sub_.subscribe(this, "~/rgb/image_raw", qos.get_rmw_qos_profile());
      stereo_right_sub_.subscribe(this, "~/right/image_raw", qos.get_rmw_qos_profile());

      using SyncPolicy = message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
      stereo_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
          SyncPolicy(10), stereo_left_sub_, stereo_right_sub_);
      stereo_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(image_slop_));
      stereo_sync_->registerCallback(
          std::bind(&SlamNode::stereoImageCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
    } else if (isRGBD(sensor_mode_)) {
      // RGB-D: ApproximateTime sync of rgb + depth
      rgbd_rgb_sub_.subscribe(this, "~/rgb/image_raw", qos.get_rmw_qos_profile());
      rgbd_depth_sub_.subscribe(this, "~/depth/image_raw", qos.get_rmw_qos_profile());

      using SyncPolicy = message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
      rgbd_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
          SyncPolicy(10), rgbd_rgb_sub_, rgbd_depth_sub_);
      rgbd_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(image_slop_));
      rgbd_sync_->registerCallback(
          std::bind(&SlamNode::rgbdImageCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
    } else {
      // Monocular: simple subscription
      mono_sub_ = create_subscription<sensor_msgs::msg::Image>(
          "~/rgb/image_raw", qos,
          std::bind(&SlamNode::monoImageCallback, this, std::placeholders::_1));
    }
  }

  // ---- Image callbacks -----------------------------------------------------

  void monoImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg) {
    if (!slam_ || slam_->isShutDown()) return;

    cv::Mat rgb;
    try {
      rgb = cv_bridge::toCvShare(rgb_msg)->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    const double t = stamp_to_sec(rgb_msg->header.stamp);
    auto vImu = isIMU(sensor_mode_) ? drainImuUpTo(t) : std::vector<ORB_SLAM3::IMU::Point>{};

    Sophus::SE3f Tcw;
    try {
      Tcw = slam_->TrackMonocular(rgb, t, vImu);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "TrackMonocular threw: %s", e.what());
      return;
    }

    if (Tcw.matrix().isZero(1e-9)) return;
    publishOdom(rgb_msg->header.stamp, Tcw);
  }

  void stereoImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                           const sensor_msgs::msg::Image::ConstSharedPtr &right_msg) {
    if (!slam_ || slam_->isShutDown()) return;

    cv::Mat left, right;
    try {
      left  = cv_bridge::toCvShare(left_msg)->image;
      right = cv_bridge::toCvShare(right_msg)->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    const double t = stamp_to_sec(left_msg->header.stamp);
    auto vImu = isIMU(sensor_mode_) ? drainImuUpTo(t) : std::vector<ORB_SLAM3::IMU::Point>{};

    Sophus::SE3f Tcw;
    try {
      Tcw = slam_->TrackStereo(left, right, t, vImu);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "TrackStereo threw: %s", e.what());
      return;
    }

    if (Tcw.matrix().isZero(1e-9)) return;
    publishOdom(left_msg->header.stamp, Tcw);
  }

  void rgbdImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
                         const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg) {
    if (!slam_ || slam_->isShutDown()) return;

    cv::Mat rgb, depth;
    try {
      rgb   = cv_bridge::toCvShare(rgb_msg)->image;
      depth = cv_bridge::toCvShare(depth_msg)->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    const double t = stamp_to_sec(rgb_msg->header.stamp);
    auto vImu = isIMU(sensor_mode_) ? drainImuUpTo(t) : std::vector<ORB_SLAM3::IMU::Point>{};

    Sophus::SE3f Tcw;
    try {
      Tcw = slam_->TrackRGBD(rgb, depth, t, vImu);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "TrackRGBD threw: %s", e.what());
      return;
    }

    if (Tcw.matrix().isZero(1e-9)) return;
    publishOdom(rgb_msg->header.stamp, Tcw);
  }

  // ---- Publishing ----------------------------------------------------------

  void publishOdom(const builtin_interfaces::msg::Time &stamp,
                   const Sophus::SE3f &Tcw) {
    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Vector3f t = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = world_frame_id_;
    odom.child_frame_id  = camera_frame_id_;
    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom);

    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = world_frame_id_;
      tf.child_frame_id  = camera_frame_id_;
      tf.transform.translation.x = t.x();
      tf.transform.translation.y = t.y();
      tf.transform.translation.z = t.z();
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void publishLocalPoints(const std::vector<Eigen::Vector3f> &pts) {
    if (!local_points_pub_ || local_points_pub_->get_subscription_count() == 0) {
      return;
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = world_frame_id_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(pts.size());
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
    for (const auto &p : pts) {
      *ix = p.x(); *iy = p.y(); *iz = p.z();
      ++ix; ++iy; ++iz;
    }
    local_points_pub_->publish(cloud);
  }

  static const char *sensor_str_from_mode(SensorMode m) {
    switch (m) {
      case SensorMode::MONOCULAR:     return "monocular";
      case SensorMode::MONOCULAR_IMU: return "monocular_imu";
      case SensorMode::STEREO:        return "stereo";
      case SensorMode::STEREO_IMU:    return "stereo_imu";
      case SensorMode::RGBD:          return "rgbd";
      case SensorMode::RGBD_IMU:      return "rgbd_imu";
    }
    return "unknown";
  }

  // ---- Member variables ----------------------------------------------------

  SensorMode sensor_mode_;

  // Common params
  std::string voc_file_;
  std::string trajectory_file_;
  bool        traj_saved_{false};
  std::string world_frame_id_;
  std::string camera_frame_id_;
  bool        publish_tf_{false};
  double      image_slop_{0.03};
  double      tf_lookup_timeout_{10.0};

  // Camera params
  int  camera_fps_;
  bool camera_rgb_;

  // IMU params
  std::string imu_frame_id_;
  double imu_noise_gyro_, imu_noise_acc_;
  double imu_gyro_walk_, imu_acc_walk_;
  double imu_frequency_;
  bool   imu_insert_kfs_when_lost_;

  // RGB-D params
  double rgbd_depth_map_factor_;

  // Stereo params
  double stereo_baseline_;
  double stereo_th_depth_;
  std::string right_camera_frame_id_;

  // ORB params
  int orb_n_features_;
  double orb_scale_factor_;
  int orb_n_levels_, orb_ini_th_fast_, orb_min_th_fast_;

  // System params
  double system_th_far_points_;

  // State
  bool initialized_{false};
  std::unique_ptr<ORB_SLAM3::System> slam_;

  // Vocabulary
  std::unique_ptr<ORB_SLAM3::ORBVocabulary> preloaded_voc_;
  std::thread voc_loader_;
  std::atomic<bool> voc_ready_{false};
  std::atomic<bool> voc_failed_{false};

  // TF
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr image_cb_group_;
  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
  rclcpp::CallbackGroup::SharedPtr right_info_cb_group_;

  // Camera info subscriptions
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_cam_info_sub_;
  sensor_msgs::msg::CameraInfo::SharedPtr right_cam_info_;

  // IMU subscription
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Monocular subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_sub_;

  // Stereo subscriptions
  message_filters::Subscriber<sensor_msgs::msg::Image> stereo_left_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> stereo_right_sub_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> stereo_sync_;

  // RGB-D subscriptions
  message_filters::Subscriber<sensor_msgs::msg::Image> rgbd_rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgbd_depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> rgbd_sync_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_points_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // IMU buffer
  std::mutex imu_mutex_;
  std::deque<ORB_SLAM3::IMU::Point> imu_buf_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlamNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
