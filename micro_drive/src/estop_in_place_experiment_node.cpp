#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
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

enum class State { IDLE, RUNNING, ESTOPPED, DONE };
// IDLE: waiting for a start_experiment call, commanding zero velocity.
// RUNNING: e-stop released, commanding the current run's angular velocity and accumulating
//          rotation. The operator lets the robot settle into steady state, then presses the
//          e-stop to end the run.
// ESTOPPED: e-stop engaged. The motors are cut in hardware, so the robot coasts to a stop and
//           recoils on its own — the node never commands zero. The command has already been
//           incremented to the next run's velocity and keeps being published, so releasing the
//           e-stop snaps the robot straight into the next run.
// DONE: the run whose velocity would exceed target_angular_velocity_rad was never started; the
//       command is zeroed, so releasing the e-stop leaves the robot still.
//
// Unlike in_place_turning_experiment_node, nothing here predicts or triggers the stop: the
// operator does, with the e-stop, and the whole point is to observe the resulting transient
// without the commanded velocity having anything to do with it.

class EstopInPlaceExperimentNode : public rclcpp::Node {
 public:
  EstopInPlaceExperimentNode() : Node("estop_in_place_experiment_node") {
    // Declare params
    declare_parameter("start_angular_velocity_rad", 0.6);
    declare_parameter("angular_velocity_increment_rad", 0.1);
    declare_parameter("target_angular_velocity_rad", 3.0);
    declare_parameter("publish_frequency_hz", 20.0);
    declare_parameter("invert_estop", false);
    declare_parameter<std::string>("imu_frame", "imu_link");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter("gyro_window_size", 20);
    declare_parameter("max_imu_delta_seconds", 0.1);
    declare_parameter("invert_rotation", false);

    // Getting params
    start_angular_velocity_rad_ = get_parameter("start_angular_velocity_rad").as_double();
    angular_velocity_increment_rad_ = get_parameter("angular_velocity_increment_rad").as_double();
    target_angular_velocity_rad_ = get_parameter("target_angular_velocity_rad").as_double();
    publish_frequency_hz_ = get_parameter("publish_frequency_hz").as_double();
    invert_estop_ = get_parameter("invert_estop").as_bool();
    imu_frame_ = get_parameter("imu_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    gyro_window_size_ = get_parameter("gyro_window_size").as_int();
    max_imu_delta_seconds_ = get_parameter("max_imu_delta_seconds").as_double();
    invert_rotation_ = get_parameter("invert_rotation").as_bool();

    // Validation
    if (publish_frequency_hz_ <= 0.0) {
      throw std::runtime_error("publish_frequency_hz must be positive, got " + std::to_string(publish_frequency_hz_) +
                               ".");
    }
    if (angular_velocity_increment_rad_ <= 0.0) {
      throw std::runtime_error(
          "angular_velocity_increment_rad must be positive, otherwise the sweep never reaches "
          "target_angular_velocity_rad. Got " +
          std::to_string(angular_velocity_increment_rad_) + ".");
    }
    if (start_angular_velocity_rad_ > target_angular_velocity_rad_) {
      throw std::runtime_error("start_angular_velocity_rad (" + std::to_string(start_angular_velocity_rad_) +
                               ") must not exceed target_angular_velocity_rad (" +
                               std::to_string(target_angular_velocity_rad_) + ").");
    }
    // All speed parameters are magnitudes; direction is applied only at publish time via
    // invert_rotation, so nothing upstream needs to reason about sign.
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

    // Imu init
    init_up_axis(imu_frame_, base_frame_);

    // Subscribers
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_unbiased", 200, std::bind(&EstopInPlaceExperimentNode::imu_callback, this, std::placeholders::_1));

    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/estop", 10, std::bind(&EstopInPlaceExperimentNode::estop_callback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/controller/cmd_vel", 10);
    state_pub_ = create_publisher<std_msgs::msg::Int32>("~/state", 10);
    current_yaw_pub_ = create_publisher<std_msgs::msg::Float64>("~/current_yaw", 200);
    estimated_yaw_vel_pub_ = create_publisher<std_msgs::msg::Float64>("~/estimated_yaw_vel", 200);
    run_command_at_estop_pub_ = create_publisher<std_msgs::msg::Float64>("~/run_command_at_estop", 10);
    run_yaw_vel_at_estop_pub_ = create_publisher<std_msgs::msg::Float64>("~/run_yaw_vel_at_estop", 10);
    run_coast_yaw_pub_ = create_publisher<std_msgs::msg::Float64>("~/run_coast_yaw", 10);
    parameters_pub_ = create_publisher<std_msgs::msg::String>("~/parameters", 10);

    // Services
    start_service_ = create_service<std_srvs::srv::Trigger>(
        "~/start_experiment",
        std::bind(&EstopInPlaceExperimentNode::start_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Timers
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_frequency_hz_));
    cmd_vel_timer_ = create_wall_timer(period_ns, std::bind(&EstopInPlaceExperimentNode::timer_callback, this));

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
    state_ = State::RUNNING;
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
    if (!has_estop_) {
      response->success = false;
      response->message = "No e-stop data received yet.";
      return;
    }
    // Starting while engaged would command a nonzero velocity the operator can't see coming, and
    // the first press would be missed since the state is already engaged.
    if (estop_engaged_) {
      response->success = false;
      response->message = "E-stop is engaged. Release it before starting.";
      return;
    }

    current_angular_velocity_ = start_angular_velocity_rad_;
    start_rotation_run();

    std_msgs::msg::String params_msg;
    params_msg.data = parameters_to_json();
    parameters_pub_->publish(params_msg);

    RCLCPP_INFO(get_logger(),
                "Experiment started. First velocity: %.3f rad/s. Press the e-stop once the robot is at steady state.",
                current_angular_velocity_);
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
         << "\"publish_frequency_hz\":" << publish_frequency_hz_ << ","
         << "\"invert_estop\":" << (invert_estop_ ? "true" : "false") << ","
         << "\"imu_frame\":\"" << imu_frame_ << "\","
         << "\"base_frame\":\"" << base_frame_ << "\","
         << "\"gyro_window_size\":" << gyro_window_size_ << ","
         << "\"max_imu_delta_seconds\":" << max_imu_delta_seconds_ << ","
         << "\"invert_rotation\":" << (invert_rotation_ ? "true" : "false") << "}";
    return json.str();
  }

  // Publishes the current commanded velocity and state at a fixed rate.
  // The command is held through the e-stop, so the controller keeps being fed the velocity the
  // next run will start with; only IDLE and DONE publish zero.
  void timer_callback() {
    geometry_msgs::msg::TwistStamped cmd_msg;
    cmd_msg.header.stamp = now();
    cmd_msg.header.frame_id = base_frame_;
    cmd_msg.twist.angular.z = invert_rotation_ ? -commanded_velocity_ : commanded_velocity_;
    cmd_vel_pub_->publish(cmd_msg);

    std_msgs::msg::Int32 state_msg;
    state_msg.data = static_cast<int>(state_);
    state_pub_->publish(state_msg);
  }

  // Drives every state transition: the operator's e-stop is the only thing that ends a run.
  // Acts on edges only, so a topic republishing an unchanged value is harmless.
  void estop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    const bool engaged = invert_estop_ ? !msg->data : msg->data;
    const bool was_engaged = estop_engaged_;
    estop_engaged_ = engaged;

    if (!has_estop_) {
      has_estop_ = true;
      return;
    }
    if (engaged == was_engaged) {
      return;
    }

    if (engaged && state_ == State::RUNNING) {
      handle_estop_pressed();
    } else if (!engaged && state_ == State::ESTOPPED) {
      handle_estop_released();
    }
  }

  // Rising edge: the run just ended. The command jumps to the next run's velocity right away,
  // so the coast that follows is labelled here with the velocity that actually produced it and
  // with the steady-state yaw rate measured at the press.
  void handle_estop_pressed() {
    yaw_at_estop_ = unwrapped_yaw_;

    std_msgs::msg::Float64 f;
    f.data = current_angular_velocity_;
    run_command_at_estop_pub_->publish(f);

    f.data = estimated_angular_velocity_;
    run_yaw_vel_at_estop_pub_->publish(f);

    const double next_vel = current_angular_velocity_ + angular_velocity_increment_rad_;
    if (next_vel > target_angular_velocity_rad_ + 1e-9) {
      RCLCPP_INFO(get_logger(), "E-stop pressed at %.3f rad/s (measured %.3f rad/s). Experiment complete.",
                  current_angular_velocity_, estimated_angular_velocity_);
      commanded_velocity_ = 0.0;
      current_angular_velocity_ = 0.0;
      state_ = State::DONE;
      return;
    }

    current_angular_velocity_ = next_vel;
    commanded_velocity_ = current_angular_velocity_;
    state_ = State::ESTOPPED;
    RCLCPP_INFO(get_logger(),
                "E-stop pressed (measured %.3f rad/s). Commanding %.3f rad/s; release the e-stop to run it.",
                estimated_angular_velocity_, current_angular_velocity_);
  }

  // Falling edge: the coast window closes and the next run begins immediately, since the new
  // command has been on the wire the whole time the e-stop was engaged.
  void handle_estop_released() {
    const double coast_yaw = unwrapped_yaw_ - yaw_at_estop_;

    std_msgs::msg::Float64 f;
    f.data = coast_yaw;
    run_coast_yaw_pub_->publish(f);

    start_rotation_run();
    RCLCPP_INFO(get_logger(), "E-stop released after coasting %.3f rad. Running at %.3f rad/s.", coast_yaw,
                current_angular_velocity_);
  }

  // Updates unwrapped_yaw_ and the gyro-based angular velocity estimate by integrating the gyro
  // rate about the "up" axis directly. Unlike in_place_turning_experiment_node this drives no
  // state transitions — the e-stop does — it only measures. Integration runs through the e-stop
  // too, so the coast and recoil land in unwrapped_yaw_ as well.
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

    std_msgs::msg::Float64 f;
    f.data = unwrapped_yaw_;
    current_yaw_pub_->publish(f);

    f.data = estimated_angular_velocity_;
    estimated_yaw_vel_pub_->publish(f);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr estimated_yaw_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr run_command_at_estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr run_yaw_vel_at_estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr run_coast_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parameters_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::TimerBase::SharedPtr cmd_vel_timer_;

  State state_{State::IDLE};
  bool has_imu_{false};
  bool has_estop_{false};
  bool estop_engaged_{false};
  double unwrapped_yaw_{0.0};
  double yaw_at_estop_{0.0};
  double current_angular_velocity_{0.0};
  double commanded_velocity_{0.0};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};

  double start_angular_velocity_rad_{0.0};
  double angular_velocity_increment_rad_{0.0};
  double target_angular_velocity_rad_{0.0};
  double publish_frequency_hz_{0.0};
  bool invert_estop_{false};
  std::string imu_frame_;
  std::string base_frame_;
  int64_t gyro_window_size_{0};
  double max_imu_delta_seconds_{0.0};
  bool invert_rotation_{false};
  tf2::Vector3 up_in_imu_{0.0, 0.0, 1.0};

  std::deque<double> gyro_up_window_;
  double estimated_angular_velocity_{0.0};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<EstopInPlaceExperimentNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("estop_in_place_experiment_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
