#include <iostream>
#include <cmath>
#include <chrono>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_LOWCMD   "rt/lowcmd"

using namespace unitree::robot;
using namespace unitree::common;

static uint32_t crc32_core(const uint32_t* ptr, uint32_t len) {  // size_t → uint32_t
  uint32_t xbit = 0;
  uint32_t data = 0;                    // ✅ 声明并初始化
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];                      // ✅ 使用 data
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }
      if (data & xbit) CRC32 ^= dwPolynomial;
      xbit >>= 1;
    }
  }
  return CRC32;  // ❌ 官方不做最终 XOR
}


// Unitree 常用“停止标志”
static constexpr double PosStopF = 2.146e9;
static constexpr double VelStopF = 16000.0;

// ===== shared lowstate =====
static std::atomic<bool> g_got_state{false};
static std::mutex g_state_mtx;
static unitree_hg::msg::dds_::LowState_ g_state{};

static void LowStateHandler(const void* msg) {
  std::lock_guard<std::mutex> lk(g_state_mtx);
  g_state = *(const unitree_hg::msg::dds_::LowState_*)msg;
  g_got_state.store(true, std::memory_order_release);
}

// Parse "10,11,12" -> vector<int>
static std::vector<int> ParseCSVInts(const std::string& s) {
  std::vector<int> out;
  if (s.empty()) return out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) out.push_back(std::stoi(item));
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

// Parse "0.1,-0.2,0.0" -> vector<double>
static std::vector<double> ParseCSVDoubles(const std::string& s) {
  std::vector<double> out;
  if (s.empty()) return out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) out.push_back(std::stod(item));
  }
  return out;
}

static double NormalizeSign(double x) {
  return (x >= 0.0) ? 1.0 : -1.0;
}

static void RequireSameSize(const std::vector<int>& a, const std::vector<double>& b,
                            const std::string& a_name, const std::string& b_name) {
  if (!b.empty() && b.size() != a.size()) {
    std::cerr << "Error: " << b_name << " size must match " << a_name << " size. "
              << a_name << " has " << a.size() << ", "
              << b_name << " has " << b.size() << "\n";
    std::exit(1);
  }
}

int main(int argc, char** argv) {
  // ===== tunable params =====
  int domain_id = 1;
  std::string iface = "lo";

  // Multiple joints
  std::vector<int> sine_joints = {10};
  std::vector<int> hold_joints = {};

  // Sine params (shared)
  double amp = 0.2;
  double period = 1.5;
  double kp_sine = 40.0;
  double kd_sine = 2.0;

  // Sine direction
  double dir_all = 1.0;
  std::vector<double> dirs_per_joint;

  // New: per-joint offsets/targets
  std::vector<double> sine_offset;  // per sine joint, rad; added to q0 center
  std::vector<double> hold_q;       // per hold joint, rad; absolute target

  // Hold gains
  double kp_hold = 120.0;
  double kd_hold = 5.0;

  // motor count
  int motor_num = 35;
  int robot_dof = 29;  // 默认 29 DOF

  // ===== argv parsing =====
  for (int i = 1; i < argc; ++i) {
    std::string k(argv[i]);

    auto need_value = [&](const std::string& opt) {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << opt << "\n";
        std::exit(1);
      }
      return std::string(argv[++i]);
    };

    if (k == "--domain") domain_id = std::stoi(need_value(k));
    else if (k == "--iface") iface = need_value(k);

    else if (k == "--sine") sine_joints = ParseCSVInts(need_value(k));
    else if (k == "--hold") hold_joints = ParseCSVInts(need_value(k));

    else if (k == "--amp") amp = std::stod(need_value(k));
    else if (k == "--period") period = std::stod(need_value(k));
    else if (k == "--kp") kp_sine = std::stod(need_value(k));
    else if (k == "--kd") kd_sine = std::stod(need_value(k));

    else if (k == "--dir") dir_all = NormalizeSign(std::stod(need_value(k)));
    else if (k == "--dirs") dirs_per_joint = ParseCSVDoubles(need_value(k));

    // New: per-joint offsets/targets
    else if (k == "--sine_offset") sine_offset = ParseCSVDoubles(need_value(k));
    else if (k == "--hold_q") hold_q = ParseCSVDoubles(need_value(k));

    else if (k == "--kp_hold") kp_hold = std::stod(need_value(k));
    else if (k == "--kd_hold") kd_hold = std::stod(need_value(k));
    else if (k == "--motors") motor_num = std::stoi(need_value(k));
    else if (k == "--dof") robot_dof = std::stoi(need_value(k));

    else if (k == "--help") {
      std::cout <<
        "Usage:\n"
        "  ./g1_identifiaction --domain 1 --iface enp3s0 \\\n"
        "    --sine 10,11 --sine_offset 0.0,0.1 --amp 0.2 --period 1.5 --kp 40 --kd 2 --dirs 1,-1 \\\n"
        "    --hold 0,1,2 --hold_q 0.0,0.0,0.5 --kp_hold 120 --kd_hold 5 --motors 35\n";
      return 0;
    }
  }

  // conflict check: a joint in both sine and hold
  {
    std::vector<int> both;
    std::set_intersection(
      sine_joints.begin(), sine_joints.end(),
      hold_joints.begin(), hold_joints.end(),
      std::back_inserter(both)
    );
    if (!both.empty()) {
      std::cerr << "Error: joints in both --sine and --hold: ";
      for (int j : both) std::cerr << j << " ";
      std::cerr << "\n";
      return 1;
    }
  }

  // size checks for per-joint vectors
  RequireSameSize(sine_joints, dirs_per_joint, "sine", "dirs");
  RequireSameSize(sine_joints, sine_offset, "sine", "sine_offset");
  RequireSameSize(hold_joints, hold_q, "hold", "hold_q");

  // build signs aligned with sine_joints
  std::vector<double> sine_signs(sine_joints.size(), dir_all);
  if (!dirs_per_joint.empty()) {
    for (size_t i = 0; i < dirs_per_joint.size(); ++i) {
      sine_signs[i] = NormalizeSign(dirs_per_joint[i]);
    }
  }

  // default offsets if not provided
  if (sine_offset.empty()) sine_offset.assign(sine_joints.size(), 0.0);
  if (hold_q.empty()) hold_q.assign(hold_joints.size(), 0.0);  // 默认锁在 0 rad

  // ===== init DDS =====
  ChannelFactory::Instance()->Init(domain_id, iface);

  ChannelSubscriber<unitree_hg::msg::dds_::LowState_> lowstate_suber(TOPIC_LOWSTATE);
  lowstate_suber.InitChannel(LowStateHandler);

  ChannelPublisher<unitree_hg::msg::dds_::LowCmd_> lowcmd_puber(TOPIC_LOWCMD);
  lowcmd_puber.InitChannel();

  // wait first state
  while (!g_got_state.load(std::memory_order_acquire)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::cout << "got lowstate\n";
  // capture each sine joint's center position from initial state
  std::vector<double> q0_sine(sine_joints.size(), 0.0);
  {
    std::lock_guard<std::mutex> lk(g_state_mtx);
    for (size_t idx = 0; idx < sine_joints.size(); ++idx) {
      int j = sine_joints[idx];
      if (j < 0 || j >= motor_num) {
        std::cerr << "Invalid sine joint index " << j << " (motor_num=" << motor_num << ")\n";
        return 1;
      }
      q0_sine[idx] = g_state.motor_state()[j].q();
    }
  }

  auto t0 = std::chrono::steady_clock::now();
  // 这里可以根据需要改成 5 / 6，或者做成命令行参数
  const uint8_t kModePr = 0;   // 视你的系统定义而定，通常 0 表示普通过程
  // const uint8_t kModeMachine = 6;  // 5: 29-DOF, 6: 27-DOF
  uint8_t kModeMachine = 5;  // default 29-DOF
  if (robot_dof == 27) kModeMachine = 6;

  while (true) {
    unitree_hg::msg::dds_::LowCmd_ cmd{};
    // 设置整机模式相关字段
    cmd.mode_pr(kModePr);
    cmd.mode_machine(kModeMachine);
    // 1) Default: stop all motors
    for (int i = 0; i < motor_num; ++i) {
      cmd.motor_cmd()[i].q()   = PosStopF;
      cmd.motor_cmd()[i].dq()  = VelStopF;
      cmd.motor_cmd()[i].kp()  = 0.0;
      cmd.motor_cmd()[i].kd()  = 0.0;
      cmd.motor_cmd()[i].tau() = 0.0;
    }

    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    // 2) Hold joints: per-joint target angle
    for (size_t idx = 0; idx < hold_joints.size(); ++idx) {
      int j = hold_joints[idx];
      if (j < 0 || j >= motor_num) continue;

      cmd.motor_cmd()[j].mode() = 1;  // 启用电机
      cmd.motor_cmd()[j].q()   = hold_q[idx];  // 每个 hold 关节自己的目标角
      cmd.motor_cmd()[j].dq()  = 0.0;
      cmd.motor_cmd()[j].kp()  = kp_hold;
      cmd.motor_cmd()[j].kd()  = kd_hold;
      cmd.motor_cmd()[j].tau() = 0.0;
    }

    // 3) Sine joints: per-joint offset added to center
    for (size_t idx = 0; idx < sine_joints.size(); ++idx) {
      int j = sine_joints[idx];
      if (j < 0 || j >= motor_num) continue;

      double center = sine_offset[idx];

      double qdes = center
                  + (sine_signs[idx] * amp) * std::sin(2.0 * M_PI * t / period);

      cmd.motor_cmd()[j].mode() = 1;  // 启用电机
      cmd.motor_cmd()[j].q() = qdes;
      cmd.motor_cmd()[j].dq() = 0.0;
      cmd.motor_cmd()[j].kp() = kp_sine;
      cmd.motor_cmd()[j].kd() = kd_sine;
      cmd.motor_cmd()[j].tau() = 0.0;

    }
      // Unitree 标准 CRC32 计算：对前 N-1 个 uint32_t 字段做校验，最后一个 crc 字段除外
    uint32_t* cmd_ptr = reinterpret_cast<uint32_t*>(&cmd);
    size_t cmd_size_words = (sizeof(cmd) / sizeof(uint32_t)) - 1;  // 减去 crc 字段本身
    
    uint32_t crc = crc32_core((uint32_t*)&cmd, cmd_size_words);
    cmd.crc(crc);

    lowcmd_puber.Write(cmd);
    usleep(2000); // ~500 Hz
  }

  return 0;
}
