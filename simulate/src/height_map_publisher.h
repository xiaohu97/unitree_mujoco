#pragma once

// 感知版 walk 的高程图（sim2sim）：在 MuJoCo 里复刻训练侧的 RayCaster，通过 Unitree DDS 发布
// unitree_go::msg::dds_::HeightMap_，供 unitree_rl_lab 的 Velocity_Rough 状态订阅。
//
// 网格约定与控制器（deploy/robots/g1_29dof/include/height_map_gate.h）和训练
// （perception_cfg.HEIGHT_SCANNER_CFG）一致：
//   * 躯干 yaw 对齐系，原点在 body（默认 torso_link）的地面投影，x 朝前、y 朝左；
//   * width 17（x：-0.8..+0.8 m）× height 11（y：-0.5..+0.5 m），分辨率 0.1 m，origin = (-0.8, -0.5)；
//   * data[iy * width + ix] = 地面高度 − 躯干高度 [m]，x 变化最快；打不到地面填 NaN。
// 射线从躯干上方 2 m 向下打，命中机器人自身几何体时从命中点下方继续，只保留静态地形。
#include <mujoco/mujoco.h>

#include <unitree/common/thread/recurrent_thread.hpp>
#include <unitree/idl/go2/HeightMap_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// 向下打射线求地面高度，跳过 root 为 robot_root_id 的几何体（机器人自身）。
// 打不到返回 NaN。overlay 和发布器共用这一份，避免两处几何定义漂移。
// 注意 mj_ray 的最后一个参数是 normal（可空），不是距离上限。
inline double ray_ground_height(const mjModel* m, const mjData* d, int robot_root_id,
                                double x, double y, double z_start, int max_self_hits)
{
    mjtNum pnt[3] = {x, y, z_start};
    const mjtNum vec[3] = {0, 0, -1};
    int geomid[1] = {-1};
    for (int k = 0; k < max_self_hits; ++k) {
        const mjtNum dist = mj_ray(m, d, pnt, vec, nullptr, 1, -1, geomid, nullptr);
        if (dist < 0) return std::numeric_limits<double>::quiet_NaN();
        const int hit_body = m->geom_bodyid[geomid[0]];
        if (m->body_rootid[hit_body] != robot_root_id) return pnt[2] - dist;
        pnt[2] -= dist + 1e-4;  // 打到自己，从命中点稍下方继续
    }
    return std::numeric_limits<double>::quiet_NaN();
}


class HeightMapPublisher
{
public:
    static constexpr uint32_t kWidth = 17, kHeight = 11;
    static constexpr float kResolution = 0.1f;
    static constexpr float kOriginX = -0.8f, kOriginY = -0.5f;
    static constexpr double kRayStartHeight = 2.0;  // 射线起点在躯干上方多少米
    static constexpr int kMaxSelfHits = 64;  // 穿过整条腿（每个几何体进/出各一次命中）要 20 次以上

    HeightMapPublisher(mjModel* model, mjData* data, std::recursive_mutex* physics_mtx,
                       const std::string& topic, const std::string& body_name, double rate_hz)
        : model_(model), data_(data), physics_mtx_(physics_mtx), rate_hz_(rate_hz)
    {
        body_id_ = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());
        if (body_id_ < 0) throw std::runtime_error("height map body '" + body_name + "' not found in the model");
        root_id_ = model_->body_rootid[body_id_];
        if (!(rate_hz_ > 0)) throw std::runtime_error("height_map_rate must be positive");
        // 局部网格，x 变化最快（Isaac Lab GridPattern "xy" 序）
        local_xy_.reserve(kWidth * kHeight);
        for (uint32_t iy = 0; iy < kHeight; ++iy) {
            for (uint32_t ix = 0; ix < kWidth; ++ix) {
                local_xy_.push_back({kOriginX + kResolution * ix, kOriginY + kResolution * iy});
            }
        }
        publisher_ = std::make_unique<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::HeightMap_>>(topic);
        publisher_->InitChannel();
        std::cout << "Height map publisher: '" << topic << "' " << kWidth << "x" << kHeight << " @ " << kResolution
                  << " m around " << body_name << ", " << rate_hz_ << " Hz" << std::endl;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "height_map", UT_CPU_ID_NONE, static_cast<uint64_t>(1e6 / rate_hz_), [this]() { this->publish(); });
    }

    // 采样一次（与 mj_step 互斥）；返回策略布局的 z_ground − z_torso。
    std::vector<float> sample(double& sim_time)
    {
        std::unique_lock<std::recursive_mutex> lock;
        if (physics_mtx_) lock = std::unique_lock<std::recursive_mutex>(*physics_mtx_);
        sim_time = data_->time;
        const mjtNum* pos = data_->xpos + 3 * body_id_;
        const mjtNum* q = data_->xquat + 4 * body_id_;  // w x y z
        const double yaw = std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        const double c = std::cos(yaw), s = std::sin(yaw);
        const double z_start = pos[2] + kRayStartHeight;
        std::vector<float> cells(kWidth * kHeight);
        for (size_t i = 0; i < local_xy_.size(); ++i) {
            const double lx = local_xy_[i][0], ly = local_xy_[i][1];
            const double wx = pos[0] + c * lx - s * ly;
            const double wy = pos[1] + s * lx + c * ly;
            const double ground = ground_height(wx, wy, z_start);
            cells[i] = std::isfinite(ground) ? static_cast<float>(ground - pos[2]) : std::numeric_limits<float>::quiet_NaN();
        }
        return cells;
    }

    void publish()
    {
        double sim_time = 0;
        const auto cells = sample(sim_time);
        unitree_go::msg::dds_::HeightMap_ msg;
        msg.stamp() = sim_time;
        msg.frame_id() = "torso_yaw";
        msg.resolution() = kResolution;
        msg.width() = kWidth;
        msg.height() = kHeight;
        msg.origin() = {kOriginX, kOriginY};
        msg.data() = cells;
        publisher_->Write(msg);
    }

    int body_id() const { return body_id_; }
    int robot_root_id() const { return root_id_; }

private:
    double ground_height(double x, double y, double z_start) const
    {
        return ray_ground_height(model_, data_, root_id_, x, y, z_start, kMaxSelfHits);
    }

    mjModel* model_;
    mjData* data_;
    std::recursive_mutex* physics_mtx_;
    double rate_hz_;
    int body_id_ = -1, root_id_ = -1;
    std::vector<std::array<double, 2>> local_xy_;
    std::unique_ptr<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::HeightMap_>> publisher_;
    unitree::common::RecurrentThreadPtr thread_;
};
