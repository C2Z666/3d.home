// D1 SDK bridge for low-speed path execution. d1-navigation-v6.5 (1Hz RobotState).
// Readiness is true only with fresh robot state, Walk posture and held control.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_sdk/sdk_client.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using robot_sdk::MotionStatus;
using robot_sdk::SDKClient;

namespace {
// SDK 0.2.1 sdk_callback.hpp: OnRobotStateData is actively reported at 1Hz.
// Allow a two-second state window; velocity watchdog stays 0.3s.
constexpr double kRobotStateTimeoutSec = 2.0;

const char * MotionName(MotionStatus status)
{
  switch (status) {
    case MotionStatus::MOTION_STATUS_STAND_UP: return "StandUp";
    case MotionStatus::MOTION_STATUS_WALK: return "Walk";
    case MotionStatus::MOTION_STATUS_BALANCE_STAND: return "BalanceStand";
    case MotionStatus::MOTION_STATUS_LIE_DOWN: return "LieDown";
    case MotionStatus::MOTION_STATUS_CRAWL: return "Crawl";
    case MotionStatus::MOTION_STATUS_CRAWL_WALK: return "CrawlWalk";
    case MotionStatus::MOTION_STATUS_LOCKED: return "Locked";
    case MotionStatus::MOTION_STATUS_CLIMB: return "Climb";
    case MotionStatus::MOTION_STATUS_STAIR: return "Stair";
    case MotionStatus::MOTION_STATUS_SLIM: return "Slim";
    case MotionStatus::MOTION_STATUS_GAIT: return "Gait";
    case MotionStatus::MOTION_STATUS_DSB: return "DSB";
    case MotionStatus::MOTION_STATUS_POS_CONTROL: return "PosControl";
    case MotionStatus::MOTION_STATUS_SK_WALK: return "SkWalk";
    case MotionStatus::MOTION_STATUS_SAND: return "Sand";
    default: return "Unknown";
  }
}

struct SDKEvents {
  std::mutex mutex;
  std::optional<robot_sdk::RobotState> state;
  std::optional<robot_sdk::TakeControlAck> control_ack;
  std::optional<std::chrono::steady_clock::time_point> gait_ack;
  std::optional<uint64_t> gait_write_failed;
  std::chrono::steady_clock::time_point state_received;
  std::atomic<uint64_t> move_write_ok{0}, move_write_failed{0};
  uint64_t state_sequence{0}, safety_epoch{0};
  bool connected{false};
  bool control_lost{false};
  bool control_available{false};
};

class DataCallback : public robot_sdk::IDataCallback {
public:
  DataCallback(std::shared_ptr<SDKEvents> events, rclcpp::Logger logger)
  : events_(std::move(events)), logger_(logger) {}

  void OnRobotStateData(const robot_sdk::RobotState & data) override
  {
    std::lock_guard<std::mutex> lock(events_->mutex);
    events_->state = data;
    events_->state_received = std::chrono::steady_clock::now();
    ++events_->state_sequence;
    if (data.hardware_emergency_status != robot_sdk::EmergencyStatus::EMERGENCY_STATUS_RECOVER ||
      data.software_emergency_status != robot_sdk::EmergencyStatus::EMERGENCY_STATUS_RECOVER)
    {
      ++events_->safety_epoch;
    }
  }

  void OnFaultData(const robot_sdk::FaultDatas & data) override
  {
    for (const auto & fault : data) {
      RCLCPP_WARN(logger_, "SDK fault level=%d code=%d: %s",
        static_cast<int>(fault.level), static_cast<int>(fault.code), fault.message.c_str());
    }
  }

  void OnControlLost(const robot_sdk::ControlLostInfo &) override
  {
    std::lock_guard<std::mutex> lock(events_->mutex);
    events_->control_lost = true;
    ++events_->safety_epoch;
  }

  void OnControlAvailable(const robot_sdk::ControlAvailableInfo &) override
  {
    std::lock_guard<std::mutex> lock(events_->mutex);
    events_->control_available = true;
  }

private:
  std::shared_ptr<SDKEvents> events_;
  rclcpp::Logger logger_;
};

class ControlCallback : public robot_sdk::IControlCallback {
public:
  explicit ControlCallback(std::shared_ptr<SDKEvents> events) : events_(std::move(events)) {}

  void OnTakeControlAck(const robot_sdk::TakeControlAck & ack) override
  {
    std::lock_guard<std::mutex> lock(events_->mutex);
    events_->control_ack = ack;
  }

  void OnGait() override
  {
    std::lock_guard<std::mutex> lock(events_->mutex);
    events_->gait_ack = std::chrono::steady_clock::now();
  }

private:
  std::shared_ptr<SDKEvents> events_;
};
}  // namespace

class D1Bridge : public rclcpp::Node {
public:
  D1Bridge() : Node("d1_bridge")
  {
    max_lin_vel_ = declare_parameter<double>("max_lin_vel", 0.8);
    max_ang_vel_ = declare_parameter<double>("max_ang_vel", 1.2);
    max_lin_accel_ = declare_parameter<double>("max_lin_accel", 1.0);
    max_ang_accel_ = declare_parameter<double>("max_ang_accel", 2.0);
    watchdog_timeout_ = declare_parameter<double>("watchdog_timeout", 0.3);
    speed_level_ = declare_parameter<int>("speed_level", 1);
    trace_control_ = declare_parameter<bool>("trace_control", false);
    allow_terminal_reverse_ = declare_parameter<bool>("allow_terminal_reverse", false);
    const auto ip = declare_parameter<std::string>("robot_ip", "192.168.168.168");
    const int port = declare_parameter<int>("robot_port", 8082);
    if (port <= 0 || port > 65535 || speed_level_ < 1 || speed_level_ > 3 ||
      !Positive(max_lin_vel_) || !Positive(max_ang_vel_) ||
      !Positive(max_lin_accel_) || !Positive(max_ang_accel_) ||
      !Positive(watchdog_timeout_) || watchdog_timeout_ >= 1.0)
    {
      throw std::invalid_argument("Invalid limits, speed level, port, or watchdog (must be < 1s)");
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
          !std::isfinite(msg->angular.z) || !std::isfinite(msg->linear.z) ||
          !std::isfinite(msg->angular.x) || !std::isfinite(msg->angular.y) ||
          (msg->linear.x < 0.0 && !ReversePermitted()) ||
          msg->linear.y != 0.0 || msg->linear.z != 0.0 ||
          msg->angular.x != 0.0 || msg->angular.y != 0.0)
        {
          RCLCPP_WARN(get_logger(), "Rejecting /cmd_vel: unsupported axis or reverse lease unavailable");
          StopMotion();
          return;
        }
        last_cmd_ = Clock::now();
        have_cmd_ = true;
        watchdog_lost_ = false;
        target_.linear.x = Clamp(msg->linear.x, max_lin_vel_);
        target_.linear.x = std::max(target_.linear.x, -0.06);
        target_.linear.y = Clamp(msg->linear.y, std::min(max_lin_vel_, 0.5));
        target_.angular.z = Clamp(msg->angular.z, max_ang_vel_);
        // An explicit stop bypasses the acceleration ramp. The timer continues
        // refreshing Move(0, 0, 0) while the confirmed posture permits Move.
        if (target_.linear.x == 0.0 && target_.linear.y == 0.0 &&
          target_.angular.z == 0.0)
        {
          StopMotion();
        }
      });
    reverse_sub_ = create_subscription<std_msgs::msg::Bool>("terminal_reverse_permit", rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        reverse_permit_ = allow_terminal_reverse_ && msg->data;
        reverse_permit_time_ = Clock::now();
        if (!reverse_permit_ && (target_.linear.x < 0.0 || current_.linear.x < 0.0)) {
          StopMotion();
        }
      });
    speed_sub_ = create_subscription<std_msgs::msg::Int32>("d1_speed_level", 10,
      [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
        if (msg->data < 1 || msg->data > 3) {
          RCLCPP_WARN(get_logger(), "Speed level must be 1, 2, or 3");
          return;
        }
        if (Controlled()) {
          Check(sdk_->SetSpeed(msg->data), "SetSpeed");
          // Conversion follows RobotState.speed_level after the robot confirms it.
        }
      });
    action_sub_ = create_subscription<std_msgs::msg::String>("d1_action", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {Action(msg->data);});
    posture_sub_ = create_subscription<std_msgs::msg::Int32>("d1_posture", 10,
      [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
        if (msg->data < 1 || msg->data > 2) {
          RCLCPP_WARN(get_logger(), "Posture must be 1=stand, 2=balance stand");
          return;
        }
        Action(msg->data == 1 ? "standup" : "balance_standup");
      });

    ready_pub_ = create_publisher<std_msgs::msg::Bool>("d1_motion_ready", rclcpp::QoS(1));
    status_pub_ = create_publisher<std_msgs::msg::String>("d1_motion_status", rclcpp::QoS(1));
    sdk_ = std::make_unique<SDKClient>();
    sdk_->SetDataCallback(std::make_shared<DataCallback>(events_, get_logger()));
    sdk_->SetControlCallback(std::make_shared<ControlCallback>(events_));
    const auto events = events_;
    const auto logger = get_logger();
    RCLCPP_INFO(logger, "RobotSDK %s, connecting directly to %s:%d (UDP)",
      sdk_->Version().c_str(), ip.c_str(), port);
    Check(sdk_->Connect(ip, std::to_string(port), false,
      [events, logger](const std::error_code & ec) {
        if (ec) {
          RCLCPP_ERROR(logger, "SDK connection failed: %s", ec.message().c_str());
          return;
        }
        std::lock_guard<std::mutex> lock(events->mutex);
        events->connected = true;
      }), "Connect");
    last_tick_ = Clock::now();
    timer_ = create_wall_timer(20ms, [this]() {Tick();});
    RCLCPP_INFO(logger, "Topics: /cmd_vel /d1_action /d1_speed_level /d1_posture; watchdog %.2fs",
      watchdog_timeout_);
    RCLCPP_INFO(logger, "Control build: d1-navigation-v6.5; trace_control=%d; RobotState=1Hz timeout=2.0s; recovery status enabled", trace_control_);
  }

  ~D1Bridge() override
  {
    if (timer_) {timer_->cancel();}
    if (sdk_) {
      // Wait briefly for the zero command's transport write before disconnecting.
      // Write completion does not prove that the physical robot has stopped.
      if (Controlled() && CanMove() && !pose_pending_) {
        const auto completion = std::make_shared<std::promise<std::error_code>>();
        auto written = completion->get_future();
        const auto error = sdk_->Move(0, 0, 0, 500,
          [completion](const std::error_code & ec, std::size_t) {completion->set_value(ec);});
        if (Check(error, "Shutdown zero Move")) {
          if (written.wait_for(500ms) == std::future_status::ready) {
            Check(written.get(), "Shutdown zero Move write");
          } else {
            RCLCPP_WARN(get_logger(), "Shutdown zero Move write was not confirmed within 500ms");
          }
        }
      }
      // Also cancel a pending connection; callbacks never access this Node.
      sdk_->Disconnect(true);
    }
  }

private:
  using Clock = std::chrono::steady_clock;
  enum class Chain {NONE, STAND, CRAWL, LIE};

  static bool Positive(double value) {return std::isfinite(value) && value > 0.0;}
  static double Clamp(double value, double limit) {return std::clamp(value, -limit, limit);}
  static double Approach(double current, double target, double delta)
  {
    return current + Clamp(target - current, delta);
  }
  static double Age(Clock::time_point since)
  {
    return std::chrono::duration<double>(Clock::now() - since).count();
  }

  bool Check(const std::error_code & ec, const char * command)
  {
    if (!ec) {return true;}
    RCLCPP_ERROR(get_logger(), "%s failed: %s", command, ec.message().c_str());
    return false;
  }

  bool Controlled() const {return sdk_->IsConnected() && control_held_;}

  bool ReversePermitted() const
  {
    return allow_terminal_reverse_ && reverse_permit_ && Age(reverse_permit_time_) < 0.2;
  }

  void SendMove(float lr, float fb, float yaw)
  {
    last_sdk_move_ = Clock::now();
    have_sdk_move_ = true;
    sdk_lr_ = lr;
    sdk_fb_ = fb;
    sdk_yaw_ = yaw;
    const auto sequence = ++move_sequence_;
    const auto events = events_;
    const auto logger = get_logger();
    // The return code reports enqueueing. The WriteHandler reports transport
    // completion; neither proves that the robot executed the command.
    Check(sdk_->Move(lr, fb, yaw, 0,
      [events, logger, sequence](const std::error_code & ec, std::size_t) {
        if (ec) {
          const auto failures = ++events->move_write_failed;
          if (failures <= 3 || failures % 50 == 0) {
            RCLCPP_ERROR(logger, "Move write #%llu failed: %s",
              static_cast<unsigned long long>(sequence), ec.message().c_str());
          }
        } else {
          ++events->move_write_ok;
        }
      }), "Move");
  }

  void TraceControl()
  {
    if (!trace_control_ || Age(last_trace_) < 1.0) {return;}
    last_trace_ = Clock::now();
    RCLCPP_INFO(get_logger(),
      "CTRL motion=%s pending=%d control=%d source=%d gait_selected=%d gait_ack=%d gait_fault=%d "
      "target_vx=%.3f current_vx=%.3f target_vy=%.3f current_vy=%.3f "
      "target_wz=%.3f current_wz=%.3f speed_level=%d sdk=[%.3f,%.3f,%.3f] sdk_age_ms=%.0f cmd_rx_age_ms=%.0f state_rx_age_ms=%.0f "
      "reported_vx=%.3f write_ok=%llu write_fail=%llu",
      MotionName(motion_), pose_pending_, Controlled(), static_cast<int>(control_source_),
      gait_selected_, gait_acknowledged_, gait_fault_,
      target_.linear.x, current_.linear.x, target_.linear.y, current_.linear.y,
      target_.angular.z, current_.angular.z, speed_level_, sdk_lr_, sdk_fb_, sdk_yaw_,
      have_sdk_move_ ? Age(last_sdk_move_) * 1000.0 : -1.0,
      have_cmd_ ? Age(last_cmd_) * 1000.0 : -1.0,
      have_state_ ? Age(last_state_) * 1000.0 : -1.0, reported_vx_,
      static_cast<unsigned long long>(events_->move_write_ok.load()),
      static_cast<unsigned long long>(events_->move_write_failed.load()));
  }
  bool CanMove() const
  {
    // SDK 0.2.1's posture matrix lists CrawlWalk and DSB as Move-capable.
    // Use reported state, never the requested posture, to permit motion.
    return motion_ == MotionStatus::MOTION_STATUS_WALK ||
           motion_ == MotionStatus::MOTION_STATUS_CRAWL_WALK ||
           motion_ == MotionStatus::MOTION_STATUS_CLIMB ||
           motion_ == MotionStatus::MOTION_STATUS_GAIT ||
           motion_ == MotionStatus::MOTION_STATUS_SLIM ||
           motion_ == MotionStatus::MOTION_STATUS_SAND ||
           motion_ == MotionStatus::MOTION_STATUS_DSB;
  }
  bool MovingPose() const
  {
    return motion_ == MotionStatus::MOTION_STATUS_CLIMB ||
           motion_ == MotionStatus::MOTION_STATUS_CRAWL_WALK ||
           motion_ == MotionStatus::MOTION_STATUS_SLIM ||
           motion_ == MotionStatus::MOTION_STATUS_GAIT ||
           motion_ == MotionStatus::MOTION_STATUS_DSB ||
           motion_ == MotionStatus::MOTION_STATUS_SAND;
  }

  void SendGaitMove(float lr, float fb, float yaw)
  {
    // Gait is a device-side locomotion controller. SDK 0.2.1 exposes no
    // individual leg/wheel command here; its example uses Gait() then Move().
    // Do not fall through to movement in Walk if the requested gait is lost.
    if (!gait_selected_ || !gait_acknowledged_ || gait_fault_ || pose_pending_ ||
      motion_ != MotionStatus::MOTION_STATUS_GAIT)
    {
      StopMotion();
      return;
    }
    SendMove(lr, fb, yaw);
  }

  void StopMotion()
  {
    target_ = geometry_msgs::msg::Twist();
    current_ = geometry_msgs::msg::Twist();
    if (Controlled() && CanMove() && !pose_pending_) {
      SendMove(0, 0, 0);
    }
  }

  void Pose(const std::string & action)
  {
    ++pose_request_id_;
    pose_request_state_sequence_ = state_sequence_;
    StopMotion();
    const auto request_started = Clock::now();
    gait_selected_ = action == "gait";
    gait_fault_ = false;
    gait_acknowledged_ = false;
    std::error_code ec;
    if (action == "standup") {
      expected_ = MotionStatus::MOTION_STATUS_WALK;
      ec = sdk_->StandUp();
    } else if (action == "balance_standup") {
      expected_ = MotionStatus::MOTION_STATUS_BALANCE_STAND;
      ec = sdk_->BalanceStandUp();

    } else if (action == "liedown") {
      expected_ = MotionStatus::MOTION_STATUS_LIE_DOWN;
      ec = sdk_->LieDown();
    } else if (action == "crawl") {
      expected_ = MotionStatus::MOTION_STATUS_CRAWL;
      ec = sdk_->Crawl();
    } else if (action == "crawl_walk") {
      expected_ = MotionStatus::MOTION_STATUS_CRAWL_WALK;
      ec = sdk_->CrawlWalk();
    } else if (action == "climb") {
      expected_ = MotionStatus::MOTION_STATUS_CLIMB;
      ec = sdk_->Climb();
    } else if (action == "gait") {
      expected_ = MotionStatus::MOTION_STATUS_GAIT;
      const auto generation = ++gait_generation_;
      const auto events = events_;
      const auto logger = get_logger();
      {
        std::lock_guard<std::mutex> lock(events->mutex);
        events->gait_ack.reset();
        events->gait_write_failed.reset();
      }
      ec = sdk_->Gait(0,
        [events, logger, generation](const std::error_code & error, std::size_t) {
          if (!error) {return;}
          {
            std::lock_guard<std::mutex> lock(events->mutex);
            events->gait_write_failed = generation;
          }
          RCLCPP_ERROR(logger, "Gait write failed: %s", error.message().c_str());
        });
    } else if (action == "slim") {
      expected_ = MotionStatus::MOTION_STATUS_SLIM;
      ec = sdk_->Slim();
    } else if (action == "sand") {
      // SDK 0.2.1 sends action/snow and parses the reported snow state as SAND.
      expected_ = MotionStatus::MOTION_STATUS_SAND;
      ec = sdk_->Sand();
    } else {
      expected_ = MotionStatus::MOTION_STATUS_DSB;
      ec = sdk_->DSB();
    }
    // An ACK means the command was received, not that the posture is complete.
    pose_pending_ = true;
    pose_sent_ = gait_selected_ ? request_started : Clock::now();
    pose_warned_ = false;
    if (!Check(ec, action.c_str())) {
      chain_ = Chain::NONE;
      if (gait_selected_) {gait_fault_ = true;}
    }
    RCLCPP_INFO(get_logger(), "Pose request: %s; awaiting RobotState=%s",
      action.c_str(), MotionName(expected_));
  }

  void Action(const std::string & action)
  {
    if (!Controlled()) {
      RCLCPP_WARN(get_logger(), "SDK control unavailable; ignoring action '%s'", action.c_str());
      return;
    }
    if (action == "front_light_on" || action == "front_light_off") {
      Check(sdk_->FrontLight(action == "front_light_on"), "FrontLight");
    } else if (action == "back_light_on" || action == "back_light_off") {
      Check(sdk_->BackLight(action == "back_light_on"), "BackLight");
    } else if (action == "auto_light_on" || action == "auto_light_off") {
      Check(sdk_->AutoModeLight(action == "auto_light_on"), "AutoModeLight");
    } else if (action == "soft_stop_on" || action == "soft_stop_off") {
      chain_ = Chain::NONE;
      StopMotion();
      const bool on = action == "soft_stop_on";
      if (on) {soft_stop_ = true;}
      Check(sdk_->SoftEmergencyStop(on), "SoftEmergencyStop");
      // Clearing the local stop waits for RobotState confirmation.
    } else if (soft_stop_ || hardware_stop_) {
      RCLCPP_WARN(get_logger(), "Emergency stop active; ignoring '%s'", action.c_str());
    } else if (action == "reverse_headtail") {
      StopMotion();
      RCLCPP_WARN(get_logger(), "Head-tail reversal is disabled in this navigation release");
    } else if (action == "locked") {
      chain_ = Chain::NONE;
      StopMotion();
      pose_pending_ = true;
      expected_ = MotionStatus::MOTION_STATUS_LOCKED;
      pose_sent_ = Clock::now();
      pose_warned_ = false;
      Check(sdk_->Locked(), "Locked");
    } else if (action == "standup" || action == "balance_standup" ||
      action == "liedown" || action == "crawl" || action == "crawl_walk" ||
      action == "climb" || action == "gait" ||
      action == "slim" || action == "dsb" || action == "sand")
    {
      if (action == "gait" && gait_selected_ && !gait_fault_ &&
        ((pose_pending_ && expected_ == MotionStatus::MOTION_STATUS_GAIT) ||
        (!pose_pending_ && motion_ == MotionStatus::MOTION_STATUS_GAIT)))
      {
        StopMotion();
        RCLCPP_INFO(get_logger(), "Gait already selected; duplicate request ignored");
        return;
      }
      chain_ = action == "liedown" && MovingPose() ? Chain::STAND : Chain::NONE;
      Pose(chain_ == Chain::STAND ? "standup" : action);
    } else {
      RCLCPP_WARN(get_logger(), "Unknown action '%s'", action.c_str());
    }
  }

  void ProcessEvents()
  {
    std::optional<robot_sdk::RobotState> state;
    std::optional<robot_sdk::TakeControlAck> ack;
    std::optional<Clock::time_point> gait_ack;
    std::optional<uint64_t> gait_write_failed;
    Clock::time_point state_received;
    bool connected, lost, available;
    {
      std::lock_guard<std::mutex> lock(events_->mutex);
      state.swap(events_->state);
      state_received = events_->state_received;
      state_sequence_ = events_->state_sequence;
      safety_epoch_ = events_->safety_epoch;
      ack.swap(events_->control_ack);
      gait_ack.swap(events_->gait_ack);
      gait_write_failed.swap(events_->gait_write_failed);
      connected = std::exchange(events_->connected, false);
      lost = std::exchange(events_->control_lost, false);
      available = std::exchange(events_->control_available, false);
    }
    if (lost || !sdk_->IsConnected()) {
      if (control_held_) {RCLCPP_WARN(get_logger(), "Control lost; pausing motion");}
      control_held_ = false;
      chain_ = Chain::NONE;
      target_ = geometry_msgs::msg::Twist();
      current_ = geometry_msgs::msg::Twist();
    }
    if (connected || (available && !control_held_)) {
      RCLCPP_INFO(get_logger(), "SDK connected/control available; requesting TakeControl");
      Check(sdk_->TakeControl(), "TakeControl");
    }
    // Loss wins over a success ACK delivered within the same executor tick.
    if (ack && !lost && sdk_->IsConnected()) {
      control_held_ = ack->error_code == 0;
      if (control_held_) {
        RCLCPP_INFO(get_logger(), "TakeControl succeeded");
        RCLCPP_INFO(get_logger(), "Robot versions: protocol=%s system=%s",
          sdk_->ProtocolVersion().c_str(), sdk_->SystemVersion().c_str());
          Check(sdk_->SetSpeed(speed_level_), "SetSpeed");
          // Emergency stop reset must be explicit.
          if (gait_selected_) {
            // Keep the explicit gait selection across control reacquisition.
            // A lost control session must not automatically resume in Walk.
            gait_fault_ = true;
            gait_acknowledged_ = false;
            pose_pending_ = false;
            StopMotion();
            RCLCPP_WARN(get_logger(),
              "Control reacquired during Gait selection; motion held. Select b to retry or 7 for Walk");
          } else {
            // Reacquisition never selects a posture automatically.
            StopMotion();
          }
      } else {
        RCLCPP_WARN(get_logger(), "TakeControl denied: %s", ack->reason.c_str());
      }
    }
    if (gait_selected_ && gait_ack && *gait_ack >= pose_sent_) {
      gait_acknowledged_ = true;
      gait_ack_received_ = *gait_ack;
      RCLCPP_INFO(get_logger(), "SDK acknowledged Gait receipt; actual posture still decides motion");
    }
    if (gait_selected_ && gait_write_failed && *gait_write_failed == gait_generation_) {
      gait_fault_ = true;
      StopMotion();
    }
    if (state) {
      last_state_ = state_received;
      have_state_ = true;
      if (state->motion_status != motion_) {
        RCLCPP_INFO(get_logger(), "RobotState: %s -> %s",
          MotionName(motion_), MotionName(state->motion_status));
        target_ = geometry_msgs::msg::Twist();
        current_ = geometry_msgs::msg::Twist();
      }
      motion_ = state->motion_status;
      reported_vx_ = std::isfinite(state->speed.line) ? state->speed.line : 999.0;
      reported_vy_ = std::isfinite(state->speed.translation) ? state->speed.translation : 999.0;
      reported_wz_ = std::isfinite(state->speed.angle) ? state->speed.angle : 999.0;
      control_source_ = state->control_source;
      soft_stop_ = state->software_emergency_status !=
        robot_sdk::EmergencyStatus::EMERGENCY_STATUS_RECOVER;
      hardware_stop_ = state->hardware_emergency_status !=
        robot_sdk::EmergencyStatus::EMERGENCY_STATUS_RECOVER;
      const int level = static_cast<int>(state->speed_level);
      if (level >= 1 && level <= 3) {speed_level_ = level;}
      if (pose_pending_ && motion_ == expected_ &&
        (state_sequence_ > pose_request_state_sequence_ && state_received >= pose_sent_) &&
        (expected_ != MotionStatus::MOTION_STATUS_GAIT ||
        (gait_acknowledged_ && state_received >= gait_ack_received_)))
      {
        pose_pending_ = false;
        pose_confirmed_id_ = pose_request_id_;
        RCLCPP_INFO(get_logger(), "Pose confirmed: %s", MotionName(motion_));
      }
      if (gait_selected_ && !pose_pending_ && !gait_fault_ && !soft_stop_ && !hardware_stop_ &&
        motion_ != MotionStatus::MOTION_STATUS_GAIT)
      {
        gait_fault_ = true;
        StopMotion();
        RCLCPP_WARN(get_logger(),
          "Requested Gait lost: actual=%s; movement stopped. Select b to retry or 7 for Walk",
          MotionName(motion_));
      }
    }
  }

  const char * MotionReadyReason() const
  {
    if (!sdk_->IsConnected()) {return "disconnected";}
    if (!control_held_) {return "control_not_held";}
    if (!have_state_) {return "waiting_for_RobotState";}
    if (hardware_stop_) {return "hardware_stop";}
    if (soft_stop_) {return "soft_stop";}
    if (Age(last_state_) >= kRobotStateTimeoutSec) {return "RobotState_timeout";}
    if (pose_pending_) {return "posture_pending";}
    if (motion_ != MotionStatus::MOTION_STATUS_WALK) {return "posture_not_Walk";}
    return "ready";
  }

  bool BridgeMotionPermitted() const
  {
    return Controlled() && CanMove() &&
           !pose_pending_ && !soft_stop_ && !hardware_stop_ &&
           have_state_ && Age(last_state_) < kRobotStateTimeoutSec &&
           (!gait_selected_ ||
           (gait_acknowledged_ && !gait_fault_ && motion_ == MotionStatus::MOTION_STATUS_GAIT));
  }

  void Tick()
  {
    ProcessEvents();
    std_msgs::msg::Bool readiness;
    const std::string reason = MotionReadyReason();
    readiness.data = reason == "ready";
    if (reason != last_ready_reason_) {
      RCLCPP_INFO(get_logger(),
        "SDK readiness: ready=%d reason=%s motion=%s control=%d soft_stop=%d "
        "hardware_stop=%d state_age_ms=%.0f state_timeout=%.1fs",
        readiness.data, reason.c_str(), MotionName(motion_), Controlled(), soft_stop_,
        hardware_stop_, have_state_ ? Age(last_state_) * 1000.0 : -1.0,
        kRobotStateTimeoutSec);
      last_ready_reason_ = reason;
    }
    ready_pub_->publish(readiness);
    // Monotonic sequences distinguish a new RobotState from repeated 50Hz status.
    // safety_epoch preserves control-loss/stop events across a missed ROS message.
    std::ostringstream status_json;
    const bool state_fresh = have_state_ && Age(last_state_) < kRobotStateTimeoutSec;
    status_json << "{\"version\":\"d1-navigation-v6.5\",\"ready\":"
                << (readiness.data ? "true" : "false")
                << ",\"reason\":\"" << reason << "\",\"state_age_sec\":"
                << (have_state_ ? Age(last_state_) : -1.0)
                << ",\"state_fresh\":" << (state_fresh ? "true" : "false")
                << ",\"state_sequence\":" << state_sequence_
                << ",\"safety_epoch\":" << safety_epoch_
                << ",\"reported_forward_speed\":" << reported_vx_
                << ",\"reported_lateral_speed\":" << reported_vy_
                << ",\"reported_yaw_rate\":" << reported_wz_
                << ",\"motion\":\"" << MotionName(motion_) << "\""
                << ",\"sdk_connected\":" << (sdk_->IsConnected() ? "true" : "false")
                << ",\"control_held\":" << (Controlled() ? "true" : "false")
                << ",\"soft_stop\":" << (soft_stop_ ? "true" : "false")
                << ",\"hardware_stop\":" << (hardware_stop_ ? "true" : "false")
                << ",\"motion_pending\":" << (pose_pending_ ? "true" : "false")
                << ",\"bridge_motion_permitted\":"
                << (BridgeMotionPermitted() ? "true" : "false")
                << ",\"bridge_session\":\"" << bridge_session_ << "\""
                << ",\"pose_request_id\":" << pose_request_id_
                << ",\"pose_confirmed_id\":" << pose_confirmed_id_
                << ",\"command_zero\":"
                << ((target_.linear.x == 0.0 && target_.linear.y == 0.0 && target_.angular.z == 0.0 &&
                  current_.linear.x == 0.0 && current_.linear.y == 0.0 && current_.angular.z == 0.0) ? "true" : "false")
                << "}";
    std_msgs::msg::String status;
    status.data = status_json.str();
    status_pub_->publish(status);
    if (pose_pending_ && Age(pose_sent_) > 5.0 && !pose_warned_) {
      RCLCPP_WARN(get_logger(), "Pose confirmation timed out; motion blocked, chain cancelled");
      pose_warned_ = true;
      chain_ = Chain::NONE;
      if (gait_selected_) {
        gait_fault_ = true;
        RCLCPP_WARN(get_logger(), "Gait receipt/state not confirmed; select b to retry");
      }
    }
    if (Controlled() && !pose_pending_ && !soft_stop_ && !hardware_stop_) {
      if (chain_ == Chain::STAND) {chain_ = Chain::CRAWL; Pose("crawl");}
      else if (chain_ == Chain::CRAWL) {chain_ = Chain::LIE; Pose("liedown");}
      else if (chain_ == Chain::LIE) {chain_ = Chain::NONE;}
    }

    const auto now = Clock::now();
    const double dt = std::min(std::chrono::duration<double>(now - last_tick_).count(), 0.1);
    last_tick_ = now;
    if ((target_.linear.x < 0.0 || current_.linear.x < 0.0) && !ReversePermitted()) {
      StopMotion();
    }
    if (have_cmd_ && Age(last_cmd_) > watchdog_timeout_) {
      target_ = geometry_msgs::msg::Twist();
      current_ = geometry_msgs::msg::Twist();
      if (!watchdog_lost_) {
        RCLCPP_WARN(get_logger(), "No /cmd_vel for %.2fs; stopping", watchdog_timeout_);
        watchdog_lost_ = true;
      }
    }
    if (!BridgeMotionPermitted())
    {
      StopMotion();
      TraceControl();
      return;
    }
    current_.linear.x = Approach(current_.linear.x, target_.linear.x, max_lin_accel_ * dt);
    current_.linear.y = Approach(current_.linear.y, target_.linear.y, max_lin_accel_ * dt);
    current_.angular.z = Approach(current_.angular.z, target_.angular.z, max_ang_accel_ * dt);
    const double fb_scale = speed_level_ == 3 ? 3.0 : speed_level_ == 2 ? 2.0 : 1.0;
    double lr = Clamp(current_.linear.y / 0.5, 1.0);
    double yaw_limit = 1.5;
    if (speed_level_ >= 2 && std::fabs(current_.linear.x) > 1.0) {
      lr = 0.0;
      yaw_limit = speed_level_ == 3 && std::fabs(current_.linear.x) > 2.0 ? 0.5 : 1.0;
    }
    const auto fb = static_cast<float>(Clamp(current_.linear.x / fb_scale, 1.0));
    const auto yaw = static_cast<float>(Clamp(Clamp(current_.angular.z, yaw_limit) / 1.5, 1.0));
    if (gait_selected_) {SendGaitMove(static_cast<float>(lr), fb, yaw);}
    else {SendMove(static_cast<float>(lr), fb, yaw);}
    TraceControl();
  }

  std::shared_ptr<SDKEvents> events_{std::make_shared<SDKEvents>()};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr speed_sub_, posture_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reverse_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  uint64_t state_sequence_{0}, safety_epoch_{0};
  uint64_t pose_request_id_{0}, pose_confirmed_id_{0}, pose_request_state_sequence_{0};
  const std::string bridge_session_{std::to_string(Clock::now().time_since_epoch().count())};
  rclcpp::TimerBase::SharedPtr timer_;
  double max_lin_vel_, max_ang_vel_, max_lin_accel_, max_ang_accel_, watchdog_timeout_;
  int speed_level_{1};
  std::string last_ready_reason_;
  bool control_held_{false}, soft_stop_{true}, hardware_stop_{true};
  bool have_cmd_{false}, have_state_{false}, watchdog_lost_{false};
  bool pose_pending_{false}, pose_warned_{false};
  bool trace_control_{false}, have_sdk_move_{false};
  bool allow_terminal_reverse_{false}, reverse_permit_{false};
  Clock::time_point reverse_permit_time_;
  bool gait_selected_{false}, gait_acknowledged_{false}, gait_fault_{false};
  uint64_t gait_generation_{0};
  uint64_t move_sequence_{0};
  float sdk_lr_{0.0F}, sdk_fb_{0.0F}, sdk_yaw_{0.0F};
  double reported_vx_{0.0}, reported_vy_{0.0}, reported_wz_{0.0};
  robot_sdk::CtrlSource control_source_{robot_sdk::CtrlSource::CTRL_SOURCE_UNKNOWN};
  MotionStatus motion_{MotionStatus::MOTION_STATUS_UNKNOWN};
  MotionStatus expected_{MotionStatus::MOTION_STATUS_UNKNOWN};
  Chain chain_{Chain::NONE};
  geometry_msgs::msg::Twist target_, current_;
  Clock::time_point last_tick_, last_cmd_, last_state_, pose_sent_;
  Clock::time_point last_sdk_move_, last_trace_;
  Clock::time_point gait_ack_received_;
  // Destroy the SDK before events/subscriptions, after Disconnect joins its worker.
  std::unique_ptr<SDKClient> sdk_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<D1Bridge>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("d1_bridge"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
