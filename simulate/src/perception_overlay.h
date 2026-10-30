#pragma once

// MuJoCo 查看器里的感知可视化（只画，不参与物理，也不影响 DDS 上发布的数据）：
//
//   * **高程图**：策略实际消费的那张 17×11 网格，每个格子在地形命中点上画一个球，
//     按"相对躯干正下方地面"的高度着色（蓝 = 低，绿 = 平，红 = 高），未命中的格子不画。
//     和 HeightMapPublisher 用同一套网格常量与同一个射线函数，看到的就是策略看到的。
//   * **深度相机视野**：按 URDF 里 d435_joint 的安装位姿（torso_link 前上方，俯仰 47.6°）
//     和 D435 的深度视场（87°×58°）打一片针孔射线，画命中点和视锥边。
//     打到地形是青色，打到机器人自己是橙色（看得见哪部分视野被自己挡住），超量程不画。
//     深度相机目前只是训练框架里的预留通路，策略并不消费它，这里纯粹用来看安装位姿够不够。
//
// 上游 simulate 只在 passive 模式渲染 user_scn，本仓库的 mujoco/simulate/simulate.cc
// 打了补丁让托管模式同样生效（搜 "unitree_mujoco patch"）。
//
// 线程：自己的 RecurrentThread 按 overlay_rate 采样，写 user_scn 时持有 sim->mtx，
// 与渲染线程在 Simulate::Sync() 里拷贝几何体的那把锁是同一把。
#include "height_map_publisher.h"

#include <mujoco/mujoco.h>
#include <unitree/common/thread/recurrent_thread.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

struct PerceptionOverlayConfig
{
    std::string body = "torso_link";
    bool show_height_map = true;
    bool show_camera = false;
    double rate_hz = 20.0;
    float point_size = 0.02f;
    // D435 深度相机：安装位姿取自 g1_29dof_rev_1_0.urdf 的 d435_joint（parent = torso_link）
    std::array<double, 3> camera_offset = {0.0576235, 0.01753, 0.42987};
    double camera_pitch = 0.8307767239493009;  // rad，绕 y 轴向下
    double camera_fov_h_deg = 87.0, camera_fov_v_deg = 58.0;
    int camera_cols = 24, camera_rows = 14;
    double camera_range = 4.0;  // m，D435 深度可靠范围之外不画
};

class PerceptionOverlay
{
public:
    PerceptionOverlay(mjModel* model, mjData* data, std::recursive_mutex* mtx, PerceptionOverlayConfig cfg)
        : model_(model), data_(data), mtx_(mtx), cfg_(cfg)
    {
        body_id_ = mj_name2id(model_, mjOBJ_BODY, cfg_.body.c_str());
        if (body_id_ < 0) throw std::runtime_error("perception overlay body '" + cfg_.body + "' not found");
        root_id_ = model_->body_rootid[body_id_];
        if (!(cfg_.rate_hz > 0)) throw std::runtime_error("overlay_rate must be positive");
        cfg_.camera_cols = std::max(2, cfg_.camera_cols);
        cfg_.camera_rows = std::max(2, cfg_.camera_rows);

        const int max_geom = HeightMapPublisher::kWidth * HeightMapPublisher::kHeight
                           + cfg_.camera_cols * cfg_.camera_rows + 16;
        mjv_defaultScene(&scene_);
        mjv_makeScene(model_, &scene_, max_geom);
        scene_.ngeom = 0;
        std::cout << "Perception overlay: height_map=" << (cfg_.show_height_map ? "on" : "off")
                  << " camera=" << (cfg_.show_camera ? "on" : "off") << " (keys: M / C), "
                  << cfg_.rate_hz << " Hz, up to " << max_geom << " markers" << std::endl;
    }

    ~PerceptionOverlay()
    {
        thread_.reset();
        mjv_freeScene(&scene_);
    }

    mjvScene* scene() { return &scene_; }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "perception_overlay", UT_CPU_ID_NONE, static_cast<uint64_t>(1e6 / cfg_.rate_hz), [this]() { this->update(); });
    }

    void toggle_height_map()
    {
        cfg_.show_height_map = !cfg_.show_height_map;
        std::cout << "[overlay] height map " << (cfg_.show_height_map ? "on" : "off") << std::endl;
    }

    void toggle_camera()
    {
        cfg_.show_camera = !cfg_.show_camera;
        std::cout << "[overlay] depth camera view " << (cfg_.show_camera ? "on" : "off") << std::endl;
    }

private:
    void update()
    {
        std::unique_lock<std::recursive_mutex> lock;
        if (mtx_) lock = std::unique_lock<std::recursive_mutex>(*mtx_);
        int n = 0;
        if (cfg_.show_height_map) n = draw_height_map(n);
        if (cfg_.show_camera) n = draw_camera(n);
        scene_.ngeom = n;
    }

    // ---- 小工具 ----
    mjvGeom* next(int& n)
    {
        if (n >= scene_.maxgeom) return nullptr;
        return &scene_.geoms[n++];
    }

    void add_point(int& n, const mjtNum pos[3], const float rgba[4])
    {
        if (mjvGeom* g = next(n)) {
            const mjtNum size[3] = {cfg_.point_size, cfg_.point_size, cfg_.point_size};
            mjv_initGeom(g, mjGEOM_SPHERE, size, pos, nullptr, rgba);
        }
    }

    void add_line(int& n, const mjtNum from[3], const mjtNum to[3], const float rgba[4])
    {
        if (mjvGeom* g = next(n)) {
            mjv_initGeom(g, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba);
            mjv_connector(g, mjGEOM_LINE, 2.0, from, to);
        }
    }

    // 蓝(低) -> 绿(平) -> 红(高)，span 之外饱和
    static void height_color(double dz, double span, float rgba[4])
    {
        const double t = std::clamp(dz / span, -1.0, 1.0);
        rgba[0] = static_cast<float>(std::max(0.0, t));
        rgba[1] = static_cast<float>(1.0 - std::fabs(t));
        rgba[2] = static_cast<float>(std::max(0.0, -t));
        rgba[3] = 0.9f;
    }

    void body_frame(mjtNum pos[3], mjtNum mat[9]) const
    {
        for (int i = 0; i < 3; ++i) pos[i] = data_->xpos[3 * body_id_ + i];
        mju_quat2Mat(mat, data_->xquat + 4 * body_id_);
    }

    int draw_height_map(int n)
    {
        mjtNum pos[3], mat[9];
        body_frame(pos, mat);
        const mjtNum* q = data_->xquat + 4 * body_id_;
        const double yaw = std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        const double c = std::cos(yaw), s = std::sin(yaw);
        const double z_start = pos[2] + HeightMapPublisher::kRayStartHeight;

        // 先全部采样，再按躯干正下方的地面高度做相对着色（躯干高度会随步态起伏，不适合当基准）
        const int w = HeightMapPublisher::kWidth, h = HeightMapPublisher::kHeight;
        std::vector<std::array<double, 3>> hits(static_cast<size_t>(w) * h);
        for (int iy = 0; iy < h; ++iy) {
            for (int ix = 0; ix < w; ++ix) {
                const double lx = HeightMapPublisher::kOriginX + HeightMapPublisher::kResolution * ix;
                const double ly = HeightMapPublisher::kOriginY + HeightMapPublisher::kResolution * iy;
                const double wx = pos[0] + c * lx - s * ly;
                const double wy = pos[1] + s * lx + c * ly;
                const double gz = ray_ground_height(model_, data_, root_id_, wx, wy, z_start,
                                                    HeightMapPublisher::kMaxSelfHits);
                hits[static_cast<size_t>(iy) * w + ix] = {wx, wy, gz};
            }
        }
        const double center = hits[static_cast<size_t>(h / 2) * w + w / 2][2];
        const double base = std::isfinite(center) ? center : pos[2] - 0.78;

        for (const auto& hit : hits) {
            if (!std::isfinite(hit[2])) continue;  // 未命中 = 未知格，不画
            float rgba[4];
            height_color(hit[2] - base, 0.2, rgba);
            const mjtNum p[3] = {hit[0], hit[1], hit[2]};
            add_point(n, p, rgba);
        }
        return n;
    }

    int draw_camera(int n)
    {
        mjtNum torso_pos[3], torso_mat[9];
        body_frame(torso_pos, torso_mat);

        // 相机位姿：torso * (平移 offset, 绕 y 俯仰 pitch)
        mjtNum offset_w[3];
        const mjtNum offset[3] = {cfg_.camera_offset[0], cfg_.camera_offset[1], cfg_.camera_offset[2]};
        mju_mulMatVec3(offset_w, torso_mat, offset);
        const mjtNum cam_pos[3] = {torso_pos[0] + offset_w[0], torso_pos[1] + offset_w[1], torso_pos[2] + offset_w[2]};
        const double cp = std::cos(cfg_.camera_pitch), sp = std::sin(cfg_.camera_pitch);
        const mjtNum pitch_mat[9] = {cp, 0, sp, 0, 1, 0, -sp, 0, cp};  // Ry(pitch)
        mjtNum cam_mat[9];
        mju_mulMatMat(cam_mat, torso_mat, pitch_mat, 3, 3, 3);
        // 相机系：+X 前、+Y 左、+Z 上（与训练里 offset convention="world" 一致）
        const mjtNum fwd[3] = {cam_mat[0], cam_mat[3], cam_mat[6]};
        const mjtNum left[3] = {cam_mat[1], cam_mat[4], cam_mat[7]};
        const mjtNum up[3] = {cam_mat[2], cam_mat[5], cam_mat[8]};

        const double th = std::tan(cfg_.camera_fov_h_deg * M_PI / 360.0);
        const double tv = std::tan(cfg_.camera_fov_v_deg * M_PI / 360.0);
        const float terrain_rgba[4] = {0.1f, 0.9f, 0.9f, 0.9f};   // 青：地形
        const float robot_rgba[4] = {1.0f, 0.55f, 0.1f, 0.9f};    // 橙：打到机器人自己
        const float frustum_rgba[4] = {1.0f, 0.9f, 0.2f, 0.7f};   // 黄：视锥边

        const auto ray_dir = [&](double u, double v, mjtNum dir[3]) {
            for (int i = 0; i < 3; ++i) dir[i] = fwd[i] - u * left[i] + v * up[i];  // u > 0 朝右
            mju_normalize3(dir);
        };

        for (int j = 0; j < cfg_.camera_rows; ++j) {
            const double v = -tv + 2.0 * tv * j / (cfg_.camera_rows - 1);
            for (int i = 0; i < cfg_.camera_cols; ++i) {
                const double u = -th + 2.0 * th * i / (cfg_.camera_cols - 1);
                mjtNum dir[3];
                ray_dir(u, v, dir);
                int geomid[1] = {-1};
                const mjtNum dist = mj_ray(model_, data_, cam_pos, dir, nullptr, 1, -1, geomid, nullptr);
                if (dist < 0 || dist > cfg_.camera_range) continue;  // 无命中或超量程
                const mjtNum p[3] = {cam_pos[0] + dir[0] * dist, cam_pos[1] + dir[1] * dist, cam_pos[2] + dir[2] * dist};
                const bool self = model_->body_rootid[model_->geom_bodyid[geomid[0]]] == root_id_;
                add_point(n, p, self ? robot_rgba : terrain_rgba);
            }
        }

        // 视锥：四条角边 + 远端方框
        mjtNum corners[4][3];
        const double us[4] = {-th, th, th, -th};
        const double vs[4] = {-tv, -tv, tv, tv};
        for (int k = 0; k < 4; ++k) {
            mjtNum dir[3];
            ray_dir(us[k], vs[k], dir);
            for (int i = 0; i < 3; ++i) corners[k][i] = cam_pos[i] + dir[i] * cfg_.camera_range;
            add_line(n, cam_pos, corners[k], frustum_rgba);
        }
        for (int k = 0; k < 4; ++k) add_line(n, corners[k], corners[(k + 1) % 4], frustum_rgba);
        return n;
    }

    mjModel* model_;
    mjData* data_;
    std::recursive_mutex* mtx_;
    PerceptionOverlayConfig cfg_;
    int body_id_ = -1, root_id_ = -1;
    mjvScene scene_{};
    unitree::common::RecurrentThreadPtr thread_;
};
