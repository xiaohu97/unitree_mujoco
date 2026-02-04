#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <iostream>
#include <array>
#include <cmath>

#include "param.h"
#include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;
    // 添加接触状态定义
    #define CONTACT_UNKNOWN 0
    #define CONTACT_MADE 1
    #define CONTACT_LOST 2

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;
        // 添加脚部body名称
    std::vector<std::string> foot_link_names_;
    std::vector<int> foot_body_ids_;

        // 检测接触状态的通用方法
    std::vector<int16_t> detectFootContacts(double force_threshold = 0.5)
    {
        std::vector<int16_t> contact_states(foot_body_ids_.size(), CONTACT_UNKNOWN);
        
        if (foot_body_ids_.empty()) {
            return contact_states;
        }
        
        // 初始化所有脚为无接触
        std::vector<bool> contact_detected(foot_body_ids_.size(), false);
        
        // 遍历所有接触点
        for (int i = 0; i < mj_data_->ncon; i++) {
            int geom1 = mj_data_->contact[i].geom1;
            int geom2 = mj_data_->contact[i].geom2;
            
            int body1 = (geom1 >= 0) ? mj_model_->geom_bodyid[geom1] : -1;
            int body2 = (geom2 >= 0) ? mj_model_->geom_bodyid[geom2] : -1;
            
            // 忽略自碰撞
            if (body1 >= 0 && body1 == body2) continue;
            
            // 计算接触力
            mjtNum force6[6] = {0};
            mj_contactForce(mj_model_, mj_data_, i, force6);
            
            double fx = static_cast<double>(force6[0]);
            double fy = static_cast<double>(force6[1]);
            double fz = static_cast<double>(force6[2]);
            double force_magnitude = std::sqrt(fx*fx + fy*fy + fz*fz);
            
            // 检查是否超过力阈值
            if (force_magnitude > force_threshold) {
                for (size_t j = 0; j < foot_body_ids_.size(); j++) {
                    if (body1 == foot_body_ids_[j] || body2 == foot_body_ids_[j]) {
                        contact_detected[j] = true;
                    }
                }
            }
        }
        
        // 转换为接触状态
        for (size_t i = 0; i < contact_detected.size(); i++) {
            contact_states[i] = contact_detected[i] ? CONTACT_MADE : CONTACT_LOST;
        }
        
        return contact_states;
    }

    // 计算脚底六维力 [fx fy fz mx my mz]（世界坐标系，力矩关于脚部body原点）
    std::vector<std::array<double, 6>> computeFootWrenches()
    {
        std::vector<std::array<double, 6>> wrenches(foot_body_ids_.size(), {0, 0, 0, 0, 0, 0});

        if (foot_body_ids_.empty()) {
            return wrenches;
        }

        for (int i = 0; i < mj_data_->ncon; i++) {
            int geom1 = mj_data_->contact[i].geom1;
            int geom2 = mj_data_->contact[i].geom2;

            int body1 = (geom1 >= 0) ? mj_model_->geom_bodyid[geom1] : -1;
            int body2 = (geom2 >= 0) ? mj_model_->geom_bodyid[geom2] : -1;

            // 忽略自碰撞
            if (body1 >= 0 && body1 == body2) continue;

            mjtNum force6[6] = {0};
            mj_contactForce(mj_model_, mj_data_, i, force6);

            // contact frame -> world frame
            const mjtNum* frame = mj_data_->contact[i].frame;
            double fx_c = static_cast<double>(force6[0]);
            double fy_c = static_cast<double>(force6[1]);
            double fz_c = static_cast<double>(force6[2]);

            double fx_w = frame[0] * fx_c + frame[1] * fy_c + frame[2] * fz_c;
            double fy_w = frame[3] * fx_c + frame[4] * fy_c + frame[5] * fz_c;
            double fz_w = frame[6] * fx_c + frame[7] * fy_c + frame[8] * fz_c;

            const mjtNum* pos = mj_data_->contact[i].pos;

            for (size_t j = 0; j < foot_body_ids_.size(); j++) {
                int foot_body = foot_body_ids_[j];
                int sign = 0;
                if (body1 == foot_body) {
                    sign = 1;
                } else if (body2 == foot_body) {
                    sign = -1;
                }
                if (sign == 0) continue;

                double fx = sign * fx_w;
                double fy = sign * fy_w;
                double fz = sign * fz_w;

                double bx = mj_data_->xpos[3 * foot_body + 0];
                double by = mj_data_->xpos[3 * foot_body + 1];
                double bz = mj_data_->xpos[3 * foot_body + 2];

                double rx = static_cast<double>(pos[0]) - bx;
                double ry = static_cast<double>(pos[1]) - by;
                double rz = static_cast<double>(pos[2]) - bz;

                // torque = r x f
                double mx = ry * fz - rz * fy;
                double my = rz * fx - rx * fz;
                double mz = rx * fy - ry * fx;

                wrenches[j][0] += fx;
                wrenches[j][1] += fy;
                wrenches[j][2] += fz;
                wrenches[j][3] += mx;
                wrenches[j][4] += my;
                wrenches[j][5] += mz;
            }
        }

        return wrenches;
    }
    
    // 初始化脚部body IDs
    void initFootBodies(const std::vector<std::string>& foot_names)
    {
        foot_link_names_ = foot_names;
        foot_body_ids_.clear();
        
        for (const auto& name : foot_names) {
            int body_id = mj_name2id(mj_model_, mjOBJ_BODY, name.c_str());
            if (body_id >= 0) {
                foot_body_ids_.push_back(body_id);
                if(param::config.print_scene_information == 1) {
                    std::cout << "Found foot body: " << name << " (id: " << body_id << ")" << std::endl;
                }
            } else {
                std::cerr << "Warning: Foot body '" << name << "' not found in model" << std::endl;
            }
        }
    }

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }

        // lowstate
        if(lowstate->trylock()) {
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            
            if(imu_quat_adr_ >= 0) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }
            
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }
        // highstate
        if(highstate->trylock()) {
            {
                double sim_time = mj_data_->time;
                int64_t sec = static_cast<int64_t>(sim_time);
                int64_t nsec = static_cast<int64_t>((sim_time - static_cast<double>(sec)) * 1e9);
                if (nsec < 0) nsec = 0;
                highstate->msg_.stamp().sec() = sec;
                highstate->msg_.stamp().nanosec() = nsec;
            }
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            updateHighStateExtra();
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;

protected:
    virtual void updateHighStateExtra() {}
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

// 为Go2添加特化版本
class Go2BridgeWithContact : public Go2Bridge
{
public:
    Go2BridgeWithContact(mjModel *model, mjData *data) : Go2Bridge(model, data)
    {
        // Go2是四足机器人，初始化4个脚
        // 根据你的URDF/MJCF模型调整这些名称
        std::vector<std::string> go2_feet = {
            "FL_foot",  // 左前脚
            "FR_foot",  // 右前脚
            "RL_foot",  // 左后脚
            "RR_foot"   // 右后脚
        };
        initFootBodies(go2_feet);
    }
    
protected:
    void updateHighStateExtra() override
    {
        // 检测脚底接触
        auto contacts = detectFootContacts(0.5); // 0.5N阈值

        // 旧字段布局：使用 foot_force(int16[4]) 填充接触状态
        for (size_t i = 0; i < 4; i++) {
            highstate->msg_.foot_force()[i] = (i < contacts.size()) ? contacts[i] : 0;
        }
    }
};

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }
        // G1是人形机器人，初始化2个脚
        // 根据你的URDF/MJCF模型调整这些名称
        std::vector<std::string> g1_feet = {
            "left_ankle_roll_link",   // 左脚
            "right_ankle_roll_link"   // 右脚
        };
        initFootBodies(g1_feet);

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    void run() override
    {
        RobotBridge::run();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

protected:
    void updateHighStateExtra() override
    {
        // 添加脚底接触检测（旧字段布局）
        auto contacts = detectFootContacts(0.5); // 0.5N阈值
        for (size_t i = 0; i < 4; i++) {
            highstate->msg_.foot_force()[i] = (i < contacts.size()) ? contacts[i] : 0;
        }

        // 临时复用 foot_speed_body(float[12]) 存放六维力
        auto wrenches = computeFootWrenches();
        for (size_t i = 0; i < 12; i++) {
            highstate->msg_.foot_speed_body()[i] = 0;
        }
        if (wrenches.size() >= 1) {
            for (size_t k = 0; k < 6; k++) {
                highstate->msg_.foot_speed_body()[k] = static_cast<float>(wrenches[0][k]);
            }
        }
        if (wrenches.size() >= 2) {
            for (size_t k = 0; k < 6; k++) {
                highstate->msg_.foot_speed_body()[6 + k] = static_cast<float>(wrenches[1][k]);
            }
        }
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
};
