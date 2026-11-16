// Offline ABI-compatible SDK double. Used only by smoke_control.py via LD_PRELOAD.
// It never creates a socket or sends anything to a robot.
#include "robot_sdk/sdk_client.hpp"
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <thread>

namespace robot_sdk {
class SDKClient::Impl {
public:
  std::mutex mutex;
  std::shared_ptr<IDataCallback> data;
  std::shared_ptr<IControlCallback> control;
  RobotState state{};
  std::atomic<bool> connected{false}, running{true};
  std::thread worker;
  std::ofstream log;

  Impl() : log(std::getenv("D1_TEST_SDK_LOG"))
  {
    state.motion_status = MotionStatus::MOTION_STATUS_LIE_DOWN;
    state.speed_level = SpeedLevel::SPEED_LEVEL_SLOW;
    state.software_emergency_status = EmergencyStatus::EMERGENCY_STATUS_RECOVER;
    state.hardware_emergency_status = EmergencyStatus::EMERGENCY_STATUS_RECOVER;
    state.control_source = CtrlSource::CTRL_SOURCE_SDK;
    worker = std::thread([this]() {
      std::ifstream commands(std::getenv("D1_TEST_SDK_COMMANDS"));
      bool reports_enabled = true;
      int report_period_ms = 100;
      auto last_report = std::chrono::steady_clock::now() - std::chrono::seconds(2);
      while (running) {
        std::string command;
        commands.clear();
        while (std::getline(commands, command)) {
          std::shared_ptr<IDataCallback> callback;
          {
            std::lock_guard<std::mutex> lock(mutex);
            callback = data;
            if (command == "walk") {state.motion_status = MotionStatus::MOTION_STATUS_WALK;}
            if (command == "climb") {state.motion_status = MotionStatus::MOTION_STATUS_CLIMB;}
            if (command == "gait") {state.motion_status = MotionStatus::MOTION_STATUS_GAIT;}
            if (command == "slim") {state.motion_status = MotionStatus::MOTION_STATUS_SLIM;}
            if (command == "sand") {state.motion_status = MotionStatus::MOTION_STATUS_SAND;}
            if (command == "crawl_walk") {state.motion_status = MotionStatus::MOTION_STATUS_CRAWL_WALK;}
            if (command == "dsb") {state.motion_status = MotionStatus::MOTION_STATUS_DSB;}
            if (command == "report_1hz") {
              report_period_ms = 1000;
              last_report = std::chrono::steady_clock::now() - std::chrono::seconds(2);
              log << "ReportMode1Hz" << std::endl;
            }
            if (command == "report_1700ms") {
              report_period_ms = 1700;
              last_report = std::chrono::steady_clock::now() - std::chrono::seconds(2);
              log << "ReportMode1700ms" << std::endl;
            }
            if (command == "reports_off") {
              reports_enabled = false;
              log << "ReportsOff" << std::endl;
            }
            if (command == "reports_on") {
              reports_enabled = true;
              last_report = std::chrono::steady_clock::now() - std::chrono::seconds(2);
              log << "ReportsOn" << std::endl;
            }
          }
          if (callback && command == "lost") {callback->OnControlLost(ControlLostInfo{});}
          if (callback && command == "available") {
            callback->OnControlAvailable(ControlAvailableInfo{});
          }
        }
        std::shared_ptr<IDataCallback> callback;
        RobotState snapshot;
        {
          std::lock_guard<std::mutex> lock(mutex);
          callback = data;
          snapshot = state;
        }
        const auto report_now = std::chrono::steady_clock::now();
        if (connected && callback && reports_enabled &&
          report_now-last_report >= std::chrono::milliseconds(report_period_ms))
        {
          callback->OnRobotStateData(snapshot);
          last_report = report_now;
          Record("ReportState");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
  }
  ~Impl() {running = false; if (worker.joinable()) {worker.join();}}
  void Record(const char * action)
  {
    std::lock_guard<std::mutex> lock(mutex);
    log << action << std::endl;
  }
  std::error_code Pose(const char * name, MotionStatus motion)
  {
    std::lock_guard<std::mutex> lock(mutex);
    log << name << std::endl;
    state.motion_status = motion;
    return {};
  }
};

SDKClient::SDKClient(ErrorHandler, ConnectionConfig, TransportProtocol)
: pImpl_(std::make_shared<Impl>()) {}
SDKClient::~SDKClient() = default;
std::error_code SDKClient::Connect(std::string, std::string, bool, ConnectHandler handler)
{
  pImpl_->connected = true;
  pImpl_->Record("Connect");
  handler({});
  return {};
}
std::error_code SDKClient::Disconnect(bool, DisConnectHandler handler)
{
  pImpl_->connected = false;
  pImpl_->running = false;
  if (pImpl_->worker.joinable()) {pImpl_->worker.join();}
  pImpl_->Record("Disconnect");
  handler({});
  return {};
}
bool SDKClient::IsConnected() const {return pImpl_->connected;}
void SDKClient::SetDataCallback(std::shared_ptr<IDataCallback> callback)
{
  std::lock_guard<std::mutex> lock(pImpl_->mutex);
  pImpl_->data = std::move(callback);
}
void SDKClient::SetControlCallback(std::shared_ptr<IControlCallback> callback)
{
  std::lock_guard<std::mutex> lock(pImpl_->mutex);
  pImpl_->control = std::move(callback);
}
const std::string & SDKClient::Version() const
{
  static const std::string version("0.2.1-offline-test");
  return version;
}
const std::string & SDKClient::ProtocolVersion() const
{
  static const std::string version("offline-protocol");
  return version;
}
const std::string & SDKClient::SystemVersion() const
{
  static const std::string version("offline-system");
  return version;
}
std::error_code SDKClient::TakeControl(int, WriteHandler)
{
  pImpl_->Record("TakeControl");
  std::shared_ptr<IControlCallback> callback;
  {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    callback = pImpl_->control;
  }
  if (callback) {callback->OnTakeControlAck(TakeControlAck{0, ""});}
  return {};
}
std::error_code SDKClient::SetSpeed(int level, int, WriteHandler)
{
  std::lock_guard<std::mutex> lock(pImpl_->mutex);
  pImpl_->log << "SetSpeed " << level << std::endl;
  pImpl_->state.speed_level = static_cast<SpeedLevel>(level);
  return {};
}
std::error_code SDKClient::SoftEmergencyStop(bool on, int, WriteHandler)
{
  std::lock_guard<std::mutex> lock(pImpl_->mutex);
  pImpl_->log << "SoftEmergencyStop " << on << std::endl;
  pImpl_->state.software_emergency_status = on ? EmergencyStatus::EMERGENCY_STATUS_STOP :
    EmergencyStatus::EMERGENCY_STATUS_RECOVER;
  return {};
}
std::error_code SDKClient::Move(float lr, float fb, float yaw, int, WriteHandler handler)
{
  {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    pImpl_->log << "Move " << lr << " " << fb << " " << yaw << std::endl;
  }
  handler({}, 1);
  return {};
}
#define TEST_POSE(method, status) \
  std::error_code SDKClient::method(int, WriteHandler) { \
    return pImpl_->Pose(#method, MotionStatus::status); }
TEST_POSE(StandUp, MOTION_STATUS_STAND_UP)
TEST_POSE(BalanceStandUp, MOTION_STATUS_BALANCE_STAND)
TEST_POSE(Stair, MOTION_STATUS_STAIR)
TEST_POSE(LieDown, MOTION_STATUS_LIE_DOWN)
TEST_POSE(Crawl, MOTION_STATUS_CRAWL)
// Hold moving postures until the test reports their actual stable state. This
// verifies that receiving a command ACK does not enable movement prematurely.
TEST_POSE(Climb, MOTION_STATUS_STAND_UP)
std::error_code SDKClient::Gait(int, WriteHandler handler)
{
  auto ec = pImpl_->Pose("Gait", MotionStatus::MOTION_STATUS_STAND_UP);
  std::shared_ptr<IControlCallback> callback;
  {
    std::lock_guard<std::mutex> lock(pImpl_->mutex);
    callback = pImpl_->control;
  }
  handler(ec, 1);
  if (!ec && callback) {callback->OnGait();}
  return ec;
}
TEST_POSE(Slim, MOTION_STATUS_STAND_UP)
TEST_POSE(Sand, MOTION_STATUS_STAND_UP)
TEST_POSE(CrawlWalk, MOTION_STATUS_STAND_UP)
TEST_POSE(DSB, MOTION_STATUS_STAND_UP)
TEST_POSE(Locked, MOTION_STATUS_LOCKED)
#undef TEST_POSE
std::error_code SDKClient::ReverseHeadTail(int, WriteHandler)
{
  pImpl_->Record("ReverseHeadTail"); return {};
}
#define TEST_LIGHT(method) \
  std::error_code SDKClient::method(bool, int, WriteHandler) { \
    pImpl_->Record(#method); return {}; }
TEST_LIGHT(FrontLight)
TEST_LIGHT(BackLight)
TEST_LIGHT(AutoModeLight)
#undef TEST_LIGHT
}  // namespace robot_sdk
