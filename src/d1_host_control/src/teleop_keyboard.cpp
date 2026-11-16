// teleop_keyboard.cpp
// 键盘控制发布者节点：
//   - 按键立即发布，并以 50Hz 持续刷新 /cmd_vel (geometry_msgs/Twist) 给 d1_bridge
//   - 按字母键发布动作指令到 /d1_action (std_msgs/String)，由 d1_bridge 按状态机校验后下发
//   - 按 1/2/3 键发布 /d1_speed_level 切换 SDK 档位
// 不依赖任何 SDK，只负责发布指令。
//
// === 按键映射表 ===
//   速度控制(w/a/s/d/空格)：
//     w=前进  a=左转  d=右转  s/j/k/空格=停止（倒退、横移禁用）
//   遥操挡位(teleop 本地速度系数，影响 /cmd_vel 数值)：
//     x=低速(0.5x)  默认=标准(1.0x)  z=高速(2.0x)
//   SDK 档位(通过 /d1_speed_level 通知 d1_bridge 调 SetSpeed)：
//     1=低速  2=中速  3=高速
//   动作指令(通过 /d1_action 通知 d1_bridge 调对应 SDK API)：
//     r=站立(standup)        前置: 卧倒/匍匐
//     f=趴窝(liedown)        前置: 通用/原地/登阶/匍匐(站立中瞬态不可直接趴下)
//     c=匍匐行走(crawl_walk)  C=静止匍匐(crawl)
//     v=爬高台(climb)         前置: 通用
//     b=步态(gait)           前置: 通用
//     n=窄路/瘦身(slim)      前置: 通用
//     m=过挡鼠板(dsb)        前置: 通用
//     ;=沙地(sand)           SDK Sand()，等待实际 SAND 状态后移动
//     h=禁用前后切换(reverse_headtail)
//     l=锁定(locked)
//   补光灯控制(通过 /d1_action 通知 d1_bridge, 任意状态可用)：
//     i=前补光灯开关(toggle)  o=后补光灯开关(toggle)  p=自动补光灯开关(toggle)
//   软急停(通过 /d1_action 通知 d1_bridge, 任意状态可用)：
//     e=软急停开关(toggle) 开启后所有运动命令被忽略,再按解除
//
// Ctrl+C 退出(退出前主动发一帧零速度停车)

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <mutex>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace {
// 保存原始终端属性，退出时恢复
termios g_old_termios;
bool g_termios_saved = false;
std::atomic<bool> g_running{true};

void RestoreTerminal() {
  if (g_termios_saved) {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_old_termios);
    g_termios_saved = false;
  }
}

void SignalHandler(int sig) {
  (void)sig;
  g_running = false;
}

// 进入 raw 模式：单键即时返回、不回显
bool EnableRawMode() {
  if (tcgetattr(STDIN_FILENO, &g_old_termios) != 0) return false;
  g_termios_saved = true;

  termios raw = g_old_termios;
  raw.c_lflag &= ~(ICANON | ECHO);    // 非规范、不回显
  raw.c_cc[VMIN] = 0;                  // 非阻塞
  raw.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  return true;
}

// 用 select 等待键盘输入，超时返回 -1
int WaitForKey(int timeout_ms) {
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(STDIN_FILENO, &read_fds);
  timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  int ret = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &tv);
  if (ret <= 0) return -1;
  char c = 0;
  if (read(STDIN_FILENO, &c, 1) != 1) return -1;
  return static_cast<int>(static_cast<unsigned char>(c));
}
}  // namespace

namespace {
// 基准速度(标准挡)，乘以挡位系数后发布
constexpr double kBaseLin = 0.3;  // m/s
constexpr double kBaseAng = 0.5;  // rad/s
}  // namespace

class TeleopKeyboard : public rclcpp::Node {
 public:
  TeleopKeyboard() : Node("teleop_keyboard") {
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1));
    // SDK 档位请求：1=低速 2=中速 3=高速(由 d1_bridge 消费并调用 SetSpeed)
    speed_level_pub_ =
        this->create_publisher<std_msgs::msg::Int32>("d1_speed_level", 10);
    // 动作指令请求(由 d1_bridge 消费并按状态机校验后调用对应 SDK API)
    action_pub_ =
        this->create_publisher<std_msgs::msg::String>("d1_action", 10);
    // 姿态切换请求(SDK 姿态接口: 1=StandUp 2=BalanceStandUp)
    posture_pub_ =
        this->create_publisher<std_msgs::msg::Int32>("d1_posture", 10);
    // 50Hz 刷新；按键也立即发布。保留按一次保持、空格停止的操作方式。
    timer_ = this->create_wall_timer(
        20ms, std::bind(&TeleopKeyboard::TimerCallback, this));

    // 启动键盘读取线程
    key_thread_ = std::thread(&TeleopKeyboard::KeyLoop, this);

    RCLCPP_INFO(this->get_logger(),
                "teleop_keyboard ready.");
    RCLCPP_INFO(this->get_logger(),
                "速度: w=前进 a=左转 d=右转 s/j/k/空格=停；倒退、横移禁用");
    RCLCPP_INFO(this->get_logger(),
                "遥操挡位(影响 /cmd_vel): x=低速(0.5x) 默认=标准(1.0x) z=高速(2.0x)");
    RCLCPP_INFO(this->get_logger(),
                "SDK 档位(发 /d1_speed_level): 1=低 2=中 3=高");
    RCLCPP_INFO(this->get_logger(),
                "姿态切换(发 /d1_posture): 7=StandUp 8=BalanceStandUp");
    RCLCPP_INFO(this->get_logger(),
                "动作指令(发 /d1_action): r=站立 f=趴窝 c=匍匐行走 C=静止匍匐 v=爬高台 "
                "b=步态 n=窄路 m=过挡鼠板 ;=沙地 h=禁用前后切换 l=锁定");
    RCLCPP_INFO(this->get_logger(),
                "补光灯(发 /d1_action): i=前灯 o=后灯 p=自动灯(均为toggle开关)");
    RCLCPP_INFO(this->get_logger(),
                "软急停(发 /d1_action): e=toggle(开启后所有运动命令被忽略)");
    RCLCPP_INFO(this->get_logger(),
                "注意: 桥接获取控制权后自动站立；按 r 可重新站立");
    RCLCPP_INFO(this->get_logger(),
                "注意: 姿态切换等待期间桥接暂停速度下发；爬高台确认后可用 W/S/空格；"
                "7/8/9 调用新版 SDK 姿态接口");
    RCLCPP_INFO(this->get_logger(),
                "注意: 动作切换前会自动停车(防运动中趴下/切换导致摔倒)");
    RCLCPP_INFO(this->get_logger(), "Ctrl+C 退出");
  }

  ~TeleopKeyboard() override {
    g_running = false;
    if (key_thread_.joinable()) key_thread_.join();
    RestoreTerminal();
  }

 private:
  // Called while KeyLoop holds twist_mtx_. Direction changes replace all axes.
  void SetMotion(double vx, double vy, double wz) {
    twist_ = geometry_msgs::msg::Twist();
    if (vx < 0.0 || vy != 0.0) {
      RCLCPP_WARN(get_logger(), "本导航版本禁止后退、横移；已改为零速度停车");
      return;
    }
    twist_.linear.x = vx;
    twist_.angular.z = wz;
  }

  void KeyLoop() {
    if (!EnableRawMode()) {
      RCLCPP_ERROR(this->get_logger(), "无法配置 termios，键盘读取失败");
      if (rclcpp::ok()) rclcpp::shutdown();  // 让主线程 spin() 返回
      return;
    }
    while (g_running && rclcpp::ok()) {
      int key = WaitForKey(50);
      if (key < 0) continue;  // 超时，循环检查 running
      char c = static_cast<char>(key);
      const bool motion_key = c == 'w' || c == 's' || c == 'a' || c == 'd' ||
                              c == 'j' || c == 'k' || c == ' ';
      geometry_msgs::msg::Twist key_twist;
      {
        std::lock_guard<std::mutex> lock(twist_mtx_);
        switch (c) {
          case 'w':
            SetMotion(kBaseLin * gear_scale_, 0.0, 0.0);
            break;
          case 's':
            SetMotion(-kBaseLin * gear_scale_, 0.0, 0.0);
            break;
          case 'a':
            SetMotion(0.0, 0.0, kBaseAng * gear_scale_);
            break;
          case 'd':
            SetMotion(0.0, 0.0, -kBaseAng * gear_scale_);
            break;
          case 'j':
            SetMotion(0.0, kBaseLin * gear_scale_, 0.0);
            break;
          case 'k':
            SetMotion(0.0, -kBaseLin * gear_scale_, 0.0);
            break;
          case 'x':  // 低速挡(原停止键改为 space 专责停止)
            gear_scale_ = 0.5;
            RCLCPP_INFO(this->get_logger(), "遥操挡位切换: 低速 (0.5x)");
            break;
          case 'z':  // 高速挡
            gear_scale_ = 2.0;
            RCLCPP_INFO(this->get_logger(), "遥操挡位切换: 高速 (2.0x)");
            break;
          case '1':  // SDK 档位：低/中/高速(SetSpeed 1/2/3)
          case '2':
          case '3': {
            std_msgs::msg::Int32 lvl;
            lvl.data = c - '0';
            speed_level_pub_->publish(lvl);
            RCLCPP_INFO(this->get_logger(),
                        "请求 SDK 档位: %d档 (%s)", lvl.data,
                        lvl.data == 1 ? "低速" : (lvl.data == 2 ? "中速" : "高速"));
            break;
          }
          case '7':  // 姿态切换: 7=StandUp 8=BalanceStandUp
          case '8': {
            twist_ = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(twist_);
            std_msgs::msg::Int32 mode;
            mode.data = c - '6';  // '7'→1 '8'→2
            posture_pub_->publish(mode);
            RCLCPP_INFO(this->get_logger(),
                        "请求姿态切换: %d (%s)", mode.data,
                        mode.data == 1 ? "站立行走" :
                        "原地站立");
            break;
          }
          // ---- 动作指令：发字符串到 /d1_action，由 d1_bridge 校验状态后下发 ----
          case 'r':  // 站立 StandUp(前置: 卧倒/匍匐)
            PublishAction("standup", "站立");
            break;
          case 'f':  // 趴窝 LieDown(前置: 站立相关状态)
            PublishAction("liedown", "趴窝");
            break;
          case 'c':  // 匍匐行走，确认 CRAWL_WALK 后可移动
            PublishAction("crawl_walk", "匍匐行走");
            break;
          case 'C':  // Shift+C 保留静止匍匐及原有趴下链所用的 Crawl
            PublishAction("crawl", "静止匍匐");
            break;
          case 'v':  // 爬高台 Climb(前置: 通用)
            PublishAction("climb", "爬高台");
            break;
          case 'b':  // 步态 Gait(前置: 通用)
            PublishAction("gait", "步态");
            break;
          case ';':  // 沙地 Sand，SDK 对应 action/snow
            PublishAction("sand", "沙地");
            break;
          case 'n':  // 窄路/瘦身 Slim(前置: 通用)
            PublishAction("slim", "瘦身/窄路");
            break;
          case 'm':  // 过挡鼠板 DSB(前置: 通用)
            PublishAction("dsb", "过挡鼠板");
            break;
          case 'h':  // 前后切换 ReverseHeadTail
            SetMotion(0, 0, 0);
            cmd_vel_pub_->publish(twist_);
            RCLCPP_WARN(get_logger(), "本导航版本禁止前后切换");
            break;
          case 'l':  // 锁定 Locked(任意状态可调，软锁)
            PublishAction("locked", "锁定");
            break;
          // ---- 补光灯：toggle 开关，任意状态可用，不影响运动 ----
          case 'i':  // 前补光灯 FrontLight toggle
            front_light_on_ = !front_light_on_;
            PublishAction(front_light_on_ ? "front_light_on" : "front_light_off",
                          front_light_on_ ? "前补光灯开" : "前补光灯关", false);
            break;
          case 'o':  // 后补光灯 BackLight toggle
            back_light_on_ = !back_light_on_;
            PublishAction(back_light_on_ ? "back_light_on" : "back_light_off",
                          back_light_on_ ? "后补光灯开" : "后补光灯关", false);
            break;
          case 'p':  // 自动补光灯 AutoModeLight toggle
            auto_light_on_ = !auto_light_on_;
            PublishAction(auto_light_on_ ? "auto_light_on" : "auto_light_off",
                          auto_light_on_ ? "自动补光灯开" : "自动补光灯关", false);
            break;
          case 'e':  // 软急停 toggle
            soft_emergency_ = !soft_emergency_;
            PublishAction(soft_emergency_ ? "soft_stop_on" : "soft_stop_off",
                          soft_emergency_ ? "软急停开启" : "软急停解除");
            break;
          case ' ':
            twist_ = geometry_msgs::msg::Twist();
            break;
          default:
            break;  // 忽略其他键
        }
        if (motion_key) {
          // Serialize publication with the timer so an older timer snapshot
          // cannot be published after a newer stop or direction key.
          cmd_vel_pub_->publish(twist_);
          key_twist = twist_;
        }
      }
      if (motion_key) {
        RCLCPP_INFO(this->get_logger(),
                    "按键 '%c' -> 速度: vx=%.2f m/s vy=%.2f m/s wz=%.2f rad/s",
                    c, key_twist.linear.x, key_twist.linear.y, key_twist.angular.z);
      }
    }
    // 退出前安全停车：主动发一帧零速度，不完全依赖 d1_bridge 看门狗兜底
    if (timer_) {timer_->cancel();}
    if (cmd_vel_pub_) {
      {
        std::lock_guard<std::mutex> lock(twist_mtx_);
        twist_ = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(twist_);
      }
      RCLCPP_INFO(this->get_logger(), "退出: 已发布零速度停车");
      std::this_thread::sleep_for(100ms);  // 留时间让消息发出
    }
    if (rclcpp::ok()) {
      rclcpp::shutdown();  // 让主线程 spin() 返回，进程正常退出
    }
  }

  void TimerCallback() {
    std::lock_guard<std::mutex> lock(twist_mtx_);
    cmd_vel_pub_->publish(twist_);
  }

  // 发布动作指令字符串到 /d1_action(d1_bridge 按状态机校验后下发 SDK API)
  // action: 英文指令字符串(d1_bridge 端匹配)，zh_name: 中文释义(仅日志显示)
  // stop_first=true 时清零 twist_(姿态切换前停车,防运动中趴下/切换摔倒)
  // stop_first=false 时不清零(补光灯等不影响运动的指令,行进中可安全调用)
  // 注意: 调用者(KeyLoop)已持有 twist_mtx_,此处不再重复获取(否则死锁)
  void PublishAction(const std::string& action, const std::string& zh_name,
                     bool stop_first = true) {
    if (stop_first) {
      twist_ = geometry_msgs::msg::Twist();  // 调用者已持有锁,直接改
      cmd_vel_pub_->publish(twist_);
    }
    std_msgs::msg::String msg;
    msg.data = action;
    action_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(),
                "请求动作: %s (%s)%s", zh_name.c_str(), action.c_str(),
                stop_first ? " [已清零速度]" : "");
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr speed_level_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr posture_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread key_thread_;
  std::mutex twist_mtx_;
  geometry_msgs::msg::Twist twist_;
  double gear_scale_{1.0};  // 挡位系数：0.5 低速 / 1.0 标准 / 2.0 高速
  // 补光灯本地 toggle 状态(SDK 无状态回读,由 teleop 本地维护)
  bool front_light_on_{false};
  bool back_light_on_{false};
  bool auto_light_on_{false};
  bool soft_emergency_{false};  // 软急停本地 toggle(SDK 无状态回读,本地维护)
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  auto node = std::make_shared<TeleopKeyboard>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
