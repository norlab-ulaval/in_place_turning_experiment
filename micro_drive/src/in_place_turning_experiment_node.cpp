#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>

using namespace std::chrono_literals;

// An internal RK4 substep is taken whenever the bristle relaxation rate times the output step
// would exceed this. 0.5 is comfortably inside RK4's ~2.78 stability limit. Mirrors
// SUBSTEP_SAFETY / MAX_SUBSTEPS in preprocess/fitting_dahl.py, which the parameters come from.
constexpr double kDahlSubstepSafety = 0.5;
// Cap on the substepping, so a bad parameter set cannot stall the IMU callback.
constexpr int kDahlMaxSubsteps = 64;

enum class State { IDLE, ROTATING, WAITING, DONE, LOCKED };
// IDLE: waiting for a start_experiment call, commanding zero velocity.
// ROTATING: commanding the current run's angular velocity, accumulating rotation.
// WAITING: velocity zeroed after reaching this run's target rotation (current velocity *
//          min_run_time_seconds), holding for wait_time_seconds so the recoil transient can be
//          observed before the next run starts.
// DONE: every velocity step up to target_angular_velocity_rad has been run.
// LOCKED: paused by an external /teleop/lock_autonomy command received during ROTATING or
//         WAITING; on unlock, redoes the current run from scratch (current_angular_velocity_
//         unchanged) rather than resuming where it left off.

class InPlaceTurningExperimentNode : public rclcpp::Node {
 public:
  InPlaceTurningExperimentNode() : Node("in_place_turning_experiment_node") {
    // Declare params
    declare_parameter("start_angular_velocity_rad", 0.6);
    declare_parameter("angular_velocity_increment_rad", 0.1);
    declare_parameter("target_angular_velocity_rad", 3.0);
    declare_parameter("min_run_time_seconds", 2.0);
    declare_parameter("wait_time_seconds", 3.0);
    declare_parameter("publish_frequency_hz", 20.0);
    declare_parameter("command_delay_seconds", 0.25);
    declare_parameter<std::string>("stopping_model", "deadtime");
    // Dahl friction model, normalized by the yaw inertia J: only sigma_0/J, sigma_1/J,
    // sigma_2/J and tau_c/J are identifiable, so J never appears here. Defaults are the pooled
    // fit of the warthog front_workshop_35kpa dataset (dahl_fit.json, 107 steps, 9.1% relative
    // RMS), produced by preprocess/fitting_dahl.py.
    declare_parameter("dahl_omega_n", 21.6647);
    declare_parameter("dahl_zeta_b", 0.0265);
    declare_parameter("dahl_a_c", 6.0965);
    declare_parameter("dahl_alpha", 1.0881);
    declare_parameter("dahl_s2", 4.0646);
    declare_parameter("dahl_horizon_seconds", 1.2);
    declare_parameter("dahl_integration_rate_hz", 400.0);
    declare_parameter<std::string>("imu_frame", "imu_link");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter("gyro_window_size", 20);
    declare_parameter("max_imu_delta_seconds", 0.1);
    declare_parameter("invert_rotation", false);
    declare_parameter("use_twist_stamped", true);

    // Getting params
    start_angular_velocity_rad_ = get_parameter("start_angular_velocity_rad").as_double();
    angular_velocity_increment_rad_ = get_parameter("angular_velocity_increment_rad").as_double();
    target_angular_velocity_rad_ = get_parameter("target_angular_velocity_rad").as_double();
    min_run_time_seconds_ = get_parameter("min_run_time_seconds").as_double();
    wait_time_seconds_ = get_parameter("wait_time_seconds").as_double();
    publish_frequency_hz_ = get_parameter("publish_frequency_hz").as_double();
    command_delay_seconds_ = get_parameter("command_delay_seconds").as_double();
    stopping_model_ = get_parameter("stopping_model").as_string();
    dahl_omega_n_ = get_parameter("dahl_omega_n").as_double();
    dahl_zeta_b_ = get_parameter("dahl_zeta_b").as_double();
    dahl_a_c_ = get_parameter("dahl_a_c").as_double();
    dahl_alpha_ = get_parameter("dahl_alpha").as_double();
    dahl_s2_ = get_parameter("dahl_s2").as_double();
    dahl_horizon_seconds_ = get_parameter("dahl_horizon_seconds").as_double();
    dahl_integration_rate_hz_ = get_parameter("dahl_integration_rate_hz").as_double();
    imu_frame_ = get_parameter("imu_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    gyro_window_size_ = get_parameter("gyro_window_size").as_int();
    max_imu_delta_seconds_ = get_parameter("max_imu_delta_seconds").as_double();
    invert_rotation_ = get_parameter("invert_rotation").as_bool();
    use_twist_stamped_ = get_parameter("use_twist_stamped").as_bool();

    // Validation
    if (stopping_model_ != "baseline" && stopping_model_ != "deadtime" && stopping_model_ != "dahl") {
      throw std::runtime_error("Unknown or unimplemented stopping_model '" + stopping_model_ +
                               "'. Valid options: baseline, deadtime, dahl.");
    }
    if (wait_time_seconds_ < 0.0) {
      throw std::runtime_error("wait_time_seconds must be non-negative, got " + std::to_string(wait_time_seconds_) +
                               ".");
    }
    if (publish_frequency_hz_ <= 0.0) {
      throw std::runtime_error("publish_frequency_hz must be positive, got " + std::to_string(publish_frequency_hz_) +
                               ".");
    }
    if (command_delay_seconds_ < 0.0) {
      throw std::runtime_error("command_delay_seconds must be non-negative, got " +
                               std::to_string(command_delay_seconds_) + ".");
    }
    if (angular_velocity_increment_rad_ <= 0.0) {
      throw std::runtime_error(
          "angular_velocity_increment_rad must be positive, otherwise the ramp never reaches "
          "target_angular_velocity_rad. Got " +
          std::to_string(angular_velocity_increment_rad_) + ".");
    }
    if (start_angular_velocity_rad_ > target_angular_velocity_rad_) {
      throw std::runtime_error("start_angular_velocity_rad (" + std::to_string(start_angular_velocity_rad_) +
                               ") must not exceed target_angular_velocity_rad (" +
                               std::to_string(target_angular_velocity_rad_) + ").");
    }
    // All angle and speed parameters are magnitudes; direction is applied only at publish time
    // via invert_rotation, so nothing upstream needs to reason about sign.
    if (start_angular_velocity_rad_ <= 0.0) {
      throw std::runtime_error("start_angular_velocity_rad must be positive, got " +
                               std::to_string(start_angular_velocity_rad_) + ".");
    }
    if (target_angular_velocity_rad_ <= 0.0) {
      throw std::runtime_error("target_angular_velocity_rad must be positive, got " +
                               std::to_string(target_angular_velocity_rad_) + ".");
    }
    if (gyro_window_size_ <= 0) {
      throw std::runtime_error("gyro_window_size must be positive, got " + std::to_string(gyro_window_size_) + ".");
    }
    if (max_imu_delta_seconds_ <= 0.0) {
      throw std::runtime_error("max_imu_delta_seconds must be positive, got " + std::to_string(max_imu_delta_seconds_) +
                               ".");
    }
    if (min_run_time_seconds_ <= 0.0) {
      throw std::runtime_error("min_run_time_seconds must be positive, got " + std::to_string(min_run_time_seconds_) +
                               ".");
    }
    // Checked whatever the stopping model is: the defaults are valid, so a bad value here is a
    // typo in the config rather than an unused parameter.
    if (dahl_omega_n_ <= 0.0) {
      throw std::runtime_error("dahl_omega_n must be positive, got " + std::to_string(dahl_omega_n_) + ".");
    }
    if (dahl_zeta_b_ < 0.0) {
      throw std::runtime_error("dahl_zeta_b must be non-negative, got " + std::to_string(dahl_zeta_b_) + ".");
    }
    if (dahl_a_c_ <= 0.0) {
      throw std::runtime_error("dahl_a_c is the Coulomb level the bristle saturates against and must be positive, got " +
                               std::to_string(dahl_a_c_) + ".");
    }
    if (dahl_alpha_ <= 0.0) {
      throw std::runtime_error("dahl_alpha must be positive, got " + std::to_string(dahl_alpha_) + ".");
    }
    if (dahl_s2_ < 0.0) {
      throw std::runtime_error("dahl_s2 must be non-negative, got " + std::to_string(dahl_s2_) + ".");
    }
    if (dahl_horizon_seconds_ <= 0.0) {
      throw std::runtime_error("dahl_horizon_seconds must be positive, got " + std::to_string(dahl_horizon_seconds_) +
                               ".");
    }
    if (dahl_integration_rate_hz_ <= 0.0) {
      throw std::runtime_error("dahl_integration_rate_hz must be positive, got " +
                               std::to_string(dahl_integration_rate_hz_) + ".");
    }

    // Imu init
    init_up_axis(imu_frame_, base_frame_);

    // Subscribers
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_unbiased", 200, std::bind(&InPlaceTurningExperimentNode::imu_callback, this, std::placeholders::_1));

    lock_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/teleop/lock_autonomy", 10,
        std::bind(&InPlaceTurningExperimentNode::lock_callback, this, std::placeholders::_1));

    // Publishers
    if (use_twist_stamped_) {
      cmd_vel_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/controller/cmd_vel", 10);
    } else {
      cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/controller/cmd_vel", 10);
    }
    state_pub_ = create_publisher<std_msgs::msg::Int32>("~/state", 10);
    current_yaw_pub_ = create_publisher<std_msgs::msg::Float64>("~/current_yaw", 200);
    estimated_yaw_vel_pub_ = create_publisher<std_msgs::msg::Float64>("~/estimated_yaw_vel", 200);
    run_yaw_overshoot_pub_ = create_publisher<std_msgs::msg::Float64>("~/run_yaw_overshoot", 200);
    run_target_rotation_pub_ = create_publisher<std_msgs::msg::Float64>("~/run_target_rotation", 10);
    parameters_pub_ = create_publisher<std_msgs::msg::String>("~/parameters", 10);

    // Services
    start_service_ = create_service<std_srvs::srv::Trigger>(
        "~/start_experiment",
        std::bind(&InPlaceTurningExperimentNode::start_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Timers
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_frequency_hz_));
    cmd_vel_timer_ = create_wall_timer(period_ns, std::bind(&InPlaceTurningExperimentNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Ready. Publishing at %.0f Hz. Call 'start_experiment' to begin.", publish_frequency_hz_);
  }

 private:
  // Looks up the static imu_frame -> base_frame transform and caches the rotation needed to
  // read yaw and yaw-rate about base_link's z axis regardless of how the IMU is physically
  // mounted.
  void init_up_axis(const std::string& imu_frame, const std::string& base_frame) {
    auto tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    RCLCPP_INFO(get_logger(), "Waiting for transform between '%s' and '%s'...", imu_frame.c_str(), base_frame.c_str());
    unsigned int attempts = 0;
    constexpr unsigned int kMaxAttempts = 100;
    constexpr auto kSleepDuration = 100ms;
    while (!tf_buffer->_frameExists(imu_frame) || !tf_buffer->_frameExists(base_frame)) {
      if (++attempts >= kMaxAttempts) {
        throw tf2::TimeoutException("tf2 lookup timeout after " +
                                    std::to_string(kMaxAttempts * kSleepDuration.count()) + " ms. Frame '" + imu_frame +
                                    "' or '" + base_frame + "' does not exist.");
      }
      rclcpp::sleep_for(kSleepDuration);
    }

    geometry_msgs::msg::TransformStamped tf_imu_from_base;
    try {
      tf_imu_from_base = tf_buffer->lookupTransform(imu_frame, base_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(get_logger(), "Unable to get tf between '%s' and '%s': %s", imu_frame.c_str(), base_frame.c_str(),
                   ex.what());
      throw;
    }

    const auto& tq = tf_imu_from_base.transform.rotation;
    const tf2::Quaternion imu_from_base_quat(tq.x, tq.y, tq.z, tq.w);
    up_in_imu_ = tf2::Matrix3x3(imu_from_base_quat).getColumn(2);
    RCLCPP_INFO(get_logger(), "Up axis (base_link z) expressed in '%s' frame: [%.3f, %.3f, %.3f]", imu_frame.c_str(),
                up_in_imu_.x(), up_in_imu_.y(), up_in_imu_.z());
  }

  void start_rotation_run() {
    commanded_velocity_ = current_angular_velocity_;
    unwrapped_yaw_ = 0.0;
    current_run_target_rotation_rad_ = current_angular_velocity_ * min_run_time_seconds_;

    std_msgs::msg::Float64 target_msg;
    target_msg.data = current_run_target_rotation_rad_;
    run_target_rotation_pub_->publish(target_msg);

    state_ = State::ROTATING;
  }

  void start_callback(const std_srvs::srv::Trigger::Request::SharedPtr,
                      std_srvs::srv::Trigger::Response::SharedPtr response) {
    if (state_ != State::IDLE && state_ != State::DONE) {
      response->success = false;
      response->message = "Experiment already running.";
      return;
    }
    if (!has_imu_) {
      response->success = false;
      response->message = "No IMU data received yet.";
      return;
    }

    current_angular_velocity_ = start_angular_velocity_rad_;
    start_rotation_run();

    std_msgs::msg::String params_msg;
    params_msg.data = parameters_to_json();
    parameters_pub_->publish(params_msg);

    RCLCPP_INFO(get_logger(), "Experiment started. First velocity: %.3f rad/s", current_angular_velocity_);
    response->success = true;
    response->message = "Experiment started.";
  }

  std::string parameters_to_json() const {
    std::ostringstream json;
    json << std::setprecision(17);
    json << "{"
         << "\"start_angular_velocity_rad\":" << start_angular_velocity_rad_ << ","
         << "\"angular_velocity_increment_rad\":" << angular_velocity_increment_rad_ << ","
         << "\"target_angular_velocity_rad\":" << target_angular_velocity_rad_ << ","
         << "\"min_run_time_seconds\":" << min_run_time_seconds_ << ","
         << "\"wait_time_seconds\":" << wait_time_seconds_ << ","
         << "\"publish_frequency_hz\":" << publish_frequency_hz_ << ","
         << "\"command_delay_seconds\":" << command_delay_seconds_ << ","
         << "\"stopping_model\":\"" << stopping_model_ << "\","
         << "\"dahl_omega_n\":" << dahl_omega_n_ << ","
         << "\"dahl_zeta_b\":" << dahl_zeta_b_ << ","
         << "\"dahl_a_c\":" << dahl_a_c_ << ","
         << "\"dahl_alpha\":" << dahl_alpha_ << ","
         << "\"dahl_s2\":" << dahl_s2_ << ","
         << "\"dahl_horizon_seconds\":" << dahl_horizon_seconds_ << ","
         << "\"dahl_integration_rate_hz\":" << dahl_integration_rate_hz_ << ","
         << "\"imu_frame\":\"" << imu_frame_ << "\","
         << "\"base_frame\":\"" << base_frame_ << "\","
         << "\"gyro_window_size\":" << gyro_window_size_ << ","
         << "\"max_imu_delta_seconds\":" << max_imu_delta_seconds_ << ","
         << "\"invert_rotation\":" << (invert_rotation_ ? "true" : "false") << ","
         << "\"use_twist_stamped\":" << (use_twist_stamped_ ? "true" : "false") << "}";
    return json.str();
  }

  // Publishes the current commanded velocity and state at a fixed rate.
  // Zero velocity is published when not rotating, keeping the controller fed.
  void timer_callback() {
    const double angular_z = invert_rotation_ ? -commanded_velocity_ : commanded_velocity_;

    if (use_twist_stamped_) {
      geometry_msgs::msg::TwistStamped cmd_msg;
      cmd_msg.header.stamp = now();
      cmd_msg.header.frame_id = base_frame_;
      cmd_msg.twist.angular.z = angular_z;
      cmd_vel_stamped_pub_->publish(cmd_msg);
    } else {
      geometry_msgs::msg::Twist cmd_msg;
      cmd_msg.angular.z = angular_z;
      cmd_vel_pub_->publish(cmd_msg);
    }

    std_msgs::msg::Int32 state_msg;
    state_msg.data = static_cast<int>(state_);
    state_pub_->publish(state_msg);
  }

  void lock_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && (state_ == State::ROTATING || state_ == State::WAITING)) {
      commanded_velocity_ = 0.0;
      state_ = State::LOCKED;
      RCLCPP_WARN(get_logger(), "Autonomy locked. Pausing at %.3f rad/s.", current_angular_velocity_);
    } else if (!msg->data && state_ == State::LOCKED) {
      start_rotation_run();
      RCLCPP_INFO(get_logger(), "Autonomy unlocked. Redoing run at %.3f rad/s.", current_angular_velocity_);
    }
  }

  // Updates unwrapped_yaw_ and the gyro-based angular velocity estimate by integrating the
  // gyro rate about the "up" axis directly. Also drives state
  // transitions. Does not publish directly — sets commanded_velocity_ instead.
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const tf2::Vector3 gyro_in_imu(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    const double gyro_up = up_in_imu_.dot(gyro_in_imu);

    gyro_up_window_.push_back(gyro_up);
    if (gyro_up_window_.size() > static_cast<std::size_t>(gyro_window_size_)) {
      gyro_up_window_.pop_front();
    }
    double sum = 0.0;
    for (double v : gyro_up_window_) sum += v;
    estimated_angular_velocity_ = sum / static_cast<double>(gyro_up_window_.size());

    const rclcpp::Time stamp(msg->header.stamp);
    if (has_imu_) {
      const double dt = (stamp - last_imu_stamp_).seconds();
      if (dt < 0.0 || dt > max_imu_delta_seconds_) {
        RCLCPP_WARN(get_logger(),
                    "Skipped a beat: IMU delta time %.6f s is out of range (expected (0, %.2f] s). Discarding "
                    "integration step.",
                    dt, max_imu_delta_seconds_);
      } else {
        unwrapped_yaw_ += gyro_up * dt;
      }
    }
    last_imu_stamp_ = stamp;

    if (!has_imu_) {
      has_imu_ = true;
      return;
    }

    switch (state_) {
      case State::ROTATING:
        handle_rotating();
        break;
      case State::WAITING:
        handle_waiting();
        break;
      default:
        break;
    }

    std_msgs::msg::Float64 f;
    f.data = unwrapped_yaw_;
    current_yaw_pub_->publish(f);

    f.data = estimated_angular_velocity_;
    estimated_yaw_vel_pub_->publish(f);
  }

  // One state of the Dahl rollout, and its derivative: the tires' bristle deflection z, the
  // body's yaw rate w, and the yaw accumulated since the command was released.
  struct DahlState {
    double z;
    double w;
    double yaw;
  };

  static double sign(double value) { return static_cast<double>(value > 0.0) - static_cast<double>(value < 0.0); }

  static DahlState advance(const DahlState& s, const DahlState& d, double h) {
    return {s.z + h * d.z, s.w + h * d.w, s.yaw + h * d.yaw};
  }

  // The Dahl friction model of preprocess/fitting_dahl.py, in torsional form and already
  // divided through by the yaw inertia J:
  //
  //   b       = 1 - (sigma_0*z/tau_c)*sign(w)
  //   dz/dt   = sign(b)*|b|^alpha * w
  //   dw/dt   = -( sigma_0*z + sigma_1*dz/dt + sigma_2*w ) / J
  //
  // The equations are odd in (z, w), so feeding a signed yaw rate in gives a signed yaw out and
  // no separate sign bookkeeping is needed here.
  DahlState dahl_derivative(const DahlState& s) const {
    const double k_over_j = dahl_omega_n_ * dahl_omega_n_;
    const double c_over_j = 2.0 * dahl_zeta_b_ * dahl_omega_n_;
    const double b = 1.0 - (k_over_j * s.z / dahl_a_c_) * sign(s.w);
    const double dz = sign(b) * std::pow(std::abs(b), dahl_alpha_) * s.w;
    return {dz, -(k_over_j * s.z + c_over_j * dz + dahl_s2_ * s.w), s.w};
  }

  // Yaw still to come if the command is released now, at the current estimated yaw rate.
  //
  // The deadtime is a pure output shift in the fit, so the body holds its rate for
  // command_delay_seconds_ and only then starts the transient — the deadtime model's whole
  // prediction is this function's initial condition. What Dahl adds is the transient itself:
  // the robot slides to a stop against the Coulomb level, then the loaded bristle springs it
  // back, and the recoil returns roughly a third of the yaw the slide gave away. The integral
  // is therefore signed and lands well short of the peak overshoot.
  double dahl_predicted_coast(double w0) const {
    DahlState state{0.0, w0, w0 * command_delay_seconds_};

    const double dt_out = 1.0 / dahl_integration_rate_hz_;
    // The bristle relaxes at sigma_0*|w|/tau_c, fastest at the release rate since the motion
    // only decays from there. At the fitted parameters this is ~115 1/s and n_sub stays 1; the
    // substepping is here so an untuned parameter set degrades in accuracy, not stability.
    const double relaxation_rate = dahl_omega_n_ * dahl_omega_n_ * std::abs(w0) / dahl_a_c_;
    const int n_sub = std::clamp(static_cast<int>(std::ceil(relaxation_rate * dt_out / kDahlSubstepSafety)), 1,
                                 kDahlMaxSubsteps);
    const double dt = dt_out / n_sub;
    const long n_steps = std::lround(std::ceil(dahl_horizon_seconds_ * dahl_integration_rate_hz_)) * n_sub;

    for (long i = 0; i < n_steps; ++i) {
      const DahlState k1 = dahl_derivative(state);
      const DahlState k2 = dahl_derivative(advance(state, k1, 0.5 * dt));
      const DahlState k3 = dahl_derivative(advance(state, k2, 0.5 * dt));
      const DahlState k4 = dahl_derivative(advance(state, k3, dt));
      state = {state.z + (dt / 6.0) * (k1.z + 2.0 * k2.z + 2.0 * k3.z + k4.z),
               state.w + (dt / 6.0) * (k1.w + 2.0 * k2.w + 2.0 * k3.w + k4.w),
               state.yaw + (dt / 6.0) * (k1.yaw + 2.0 * k2.yaw + 2.0 * k3.yaw + k4.yaw)};
    }
    return state.yaw;
  }

  void handle_rotating() {
    const double rotation = unwrapped_yaw_;

    double predicted_rotation = rotation;
    if (stopping_model_ == "baseline") {
      predicted_rotation = rotation;
    } else if (stopping_model_ == "deadtime") {
      predicted_rotation = rotation + estimated_angular_velocity_ * command_delay_seconds_;
    } else if (stopping_model_ == "dahl") {
      predicted_rotation = rotation + dahl_predicted_coast(estimated_angular_velocity_);
    }

    if (std::abs(predicted_rotation) < current_run_target_rotation_rad_) {
      return;
    }

    commanded_velocity_ = 0.0;
    RCLCPP_INFO(get_logger(), "Rotated %.3f rad (projected %.3f rad) at %.3f rad/s. Waiting...", rotation,
                predicted_rotation, current_angular_velocity_);
    wait_start_time_ = now();
    state_ = State::WAITING;
  }

  void handle_waiting() {
    if ((now() - wait_start_time_).seconds() < wait_time_seconds_) {
      return;
    }

    const double next_vel = current_angular_velocity_ + angular_velocity_increment_rad_;
    const double target_vel = target_angular_velocity_rad_;

    const double overshoot = std::abs(unwrapped_yaw_) - current_run_target_rotation_rad_;

    std_msgs::msg::Float64 f;
    f.data = overshoot;
    run_yaw_overshoot_pub_->publish(f);

    if (next_vel > target_vel + 1e-9) {
      RCLCPP_INFO(get_logger(), "Experiment complete.");
      commanded_velocity_ = 0.0;
      current_angular_velocity_ = 0.0;
      unwrapped_yaw_ = 0.0;
      current_run_target_rotation_rad_ = 0.0;
      state_ = State::DONE;
      return;
    }

    current_angular_velocity_ = next_vel;
    start_rotation_run();

    RCLCPP_INFO(get_logger(), "Starting next run at %.3f rad/s.", current_angular_velocity_);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lock_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr estimated_yaw_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr run_yaw_overshoot_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr run_target_rotation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parameters_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::TimerBase::SharedPtr cmd_vel_timer_;

  State state_{State::IDLE};
  bool has_imu_{false};
  double unwrapped_yaw_{0.0};
  double current_angular_velocity_{0.0};
  double commanded_velocity_{0.0};
  double current_run_target_rotation_rad_{0.0};
  rclcpp::Time wait_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};

  double start_angular_velocity_rad_{0.0};
  double angular_velocity_increment_rad_{0.0};
  double target_angular_velocity_rad_{0.0};
  double min_run_time_seconds_{0.0};
  double wait_time_seconds_{0.0};
  double publish_frequency_hz_{0.0};
  double command_delay_seconds_{0.0};
  std::string stopping_model_;
  double dahl_omega_n_{0.0};
  double dahl_zeta_b_{0.0};
  double dahl_a_c_{0.0};
  double dahl_alpha_{0.0};
  double dahl_s2_{0.0};
  double dahl_horizon_seconds_{0.0};
  double dahl_integration_rate_hz_{0.0};
  std::string imu_frame_;
  std::string base_frame_;
  int64_t gyro_window_size_{0};
  double max_imu_delta_seconds_{0.0};
  bool invert_rotation_{false};
  bool use_twist_stamped_{true};
  tf2::Vector3 up_in_imu_{0.0, 0.0, 1.0};

  std::deque<double> gyro_up_window_;
  double estimated_angular_velocity_{0.0};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<InPlaceTurningExperimentNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("in_place_turning_experiment_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}