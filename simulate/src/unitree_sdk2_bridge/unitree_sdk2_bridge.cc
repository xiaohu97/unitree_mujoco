#include "unitree_sdk2_bridge.h"
#include <chrono>

// 接触状态定义
#define CONTACT_UNKNOWN 0
#define CONTACT_MADE 1
#define CONTACT_LOST 2



UnitreeSdk2Bridge::UnitreeSdk2Bridge(mjModel *model, mjData *data) : mj_model_(model), mj_data_(data)
{
    CheckSensor();
        // 初始化接触状态
    prev_left_foot_contact_ = CONTACT_UNKNOWN;
    prev_right_foot_contact_ = CONTACT_UNKNOWN;
    if (idl_type_ == 0)
    {
        low_cmd_go_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        low_cmd_go_suber_->InitChannel(bind(&UnitreeSdk2Bridge::LowCmdGoHandler, this, placeholders::_1), 1);

        low_state_go_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        low_state_go_puber_->InitChannel();

        lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishLowStateGo, this);
    }
    else
    {
        low_cmd_hg_suber_.reset(new ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        low_cmd_hg_suber_->InitChannel(bind(&UnitreeSdk2Bridge::LowCmdHgHandler, this, placeholders::_1), 1);

        low_state_hg_puber_.reset(new ChannelPublisher<unitree_hg::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        low_state_hg_puber_->InitChannel();
    // 指定周期为 2000 微秒（2 毫秒 500hz）
        lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishLowStateHg, this);
    }

    high_state_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    high_state_puber_->InitChannel();
    wireless_controller_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::WirelessController_>(TOPIC_WIRELESS_CONTROLLER));
    wireless_controller_puber_->InitChannel();

    HighStatePuberThreadPtr = CreateRecurrentThreadEx("highstate", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishHighState, this);
    WirelessControllerPuberThreadPtr = CreateRecurrentThreadEx("wirelesscontroller", UT_CPU_ID_NONE, 2000, &UnitreeSdk2Bridge::PublishWirelessController, this);
}

UnitreeSdk2Bridge::~UnitreeSdk2Bridge()
{
    delete js_;
}

void UnitreeSdk2Bridge::LowCmdGoHandler(const void *msg)
{
    const unitree_go::msg::dds_::LowCmd_ *cmd = (const unitree_go::msg::dds_::LowCmd_ *)msg;
    if (mj_data_)
    {
        for (int i = 0; i < num_motor_; i++)
        {
            mj_data_->ctrl[i] = cmd->motor_cmd()[i].tau() +
                                cmd->motor_cmd()[i].kp() * (cmd->motor_cmd()[i].q() - mj_data_->sensordata[i]) +
                                cmd->motor_cmd()[i].kd() * (cmd->motor_cmd()[i].dq() - mj_data_->sensordata[i + num_motor_]);
        }
    }
}

void UnitreeSdk2Bridge::LowCmdHgHandler(const void *msg)
{
    const unitree_hg::msg::dds_::LowCmd_ *cmd = (const unitree_hg::msg::dds_::LowCmd_ *)msg;
    if (mj_data_)
    {
        for (int i = 0; i < num_motor_; i++)
        {
            mj_data_->ctrl[i] = cmd->motor_cmd()[i].tau() +
                                cmd->motor_cmd()[i].kp() * (cmd->motor_cmd()[i].q() - mj_data_->sensordata[i]) +
                                cmd->motor_cmd()[i].kd() * (cmd->motor_cmd()[i].dq() - mj_data_->sensordata[i + num_motor_]);
        }
    }
}

void UnitreeSdk2Bridge::PublishLowStateGo()
{
    if (mj_data_)
    {
        for (int i = 0; i < num_motor_; i++)
        {
            low_state_go_.motor_state()[i].q() = mj_data_->sensordata[i];
            low_state_go_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
            low_state_go_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
        }

        if (have_frame_sensor_)
        {
            low_state_go_.imu_state().quaternion()[0] = mj_data_->sensordata[dim_motor_sensor_ + 0];
            low_state_go_.imu_state().quaternion()[1] = mj_data_->sensordata[dim_motor_sensor_ + 1];
            low_state_go_.imu_state().quaternion()[2] = mj_data_->sensordata[dim_motor_sensor_ + 2];
            low_state_go_.imu_state().quaternion()[3] = mj_data_->sensordata[dim_motor_sensor_ + 3];

	    double w = low_state_go_.imu_state().quaternion()[0];
	    double x = low_state_go_.imu_state().quaternion()[1];
	    double y = low_state_go_.imu_state().quaternion()[2];
	    double z = low_state_go_.imu_state().quaternion()[3];

	    low_state_go_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
	    low_state_go_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
	    low_state_go_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

            low_state_go_.imu_state().gyroscope()[0] = mj_data_->sensordata[dim_motor_sensor_ + 4];
            low_state_go_.imu_state().gyroscope()[1] = mj_data_->sensordata[dim_motor_sensor_ + 5];
            low_state_go_.imu_state().gyroscope()[2] = mj_data_->sensordata[dim_motor_sensor_ + 6];

            low_state_go_.imu_state().accelerometer()[0] = mj_data_->sensordata[dim_motor_sensor_ + 7];
            low_state_go_.imu_state().accelerometer()[1] = mj_data_->sensordata[dim_motor_sensor_ + 8];
            low_state_go_.imu_state().accelerometer()[2] = mj_data_->sensordata[dim_motor_sensor_ + 9];
        }

        if (js_)
        {
            GetWirelessRemote();
            memcpy(&low_state_go_.wireless_remote()[0], &wireless_remote_, 40);
        }

        low_state_go_puber_->Write(low_state_go_);
    }
}

void UnitreeSdk2Bridge::PublishLowStateHg()
{
    // 修改 C++ 代码添加 tick 时间戳
    if (mj_data_)
    {
         // 设置时间戳（毫秒）
        auto now = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch());
        low_state_hg_.tick() = static_cast<uint32_t>(duration.count());

        for (int i = 0; i < num_motor_; i++)
        {
            low_state_hg_.motor_state()[i].q() = mj_data_->sensordata[i];
            low_state_hg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
            low_state_hg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
        }

        if (have_frame_sensor_)
        {
            low_state_hg_.imu_state().quaternion()[0] = mj_data_->sensordata[dim_motor_sensor_ + 0];
            low_state_hg_.imu_state().quaternion()[1] = mj_data_->sensordata[dim_motor_sensor_ + 1];
            low_state_hg_.imu_state().quaternion()[2] = mj_data_->sensordata[dim_motor_sensor_ + 2];
            low_state_hg_.imu_state().quaternion()[3] = mj_data_->sensordata[dim_motor_sensor_ + 3];

	    double w = low_state_hg_.imu_state().quaternion()[0];
	    double x = low_state_hg_.imu_state().quaternion()[1];
	    double y = low_state_hg_.imu_state().quaternion()[2];
	    double z = low_state_hg_.imu_state().quaternion()[3];

	    low_state_hg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
	    low_state_hg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
	    low_state_hg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

            low_state_hg_.imu_state().gyroscope()[0] = mj_data_->sensordata[dim_motor_sensor_ + 4];
            low_state_hg_.imu_state().gyroscope()[1] = mj_data_->sensordata[dim_motor_sensor_ + 5];
            low_state_hg_.imu_state().gyroscope()[2] = mj_data_->sensordata[dim_motor_sensor_ + 6];

            low_state_hg_.imu_state().accelerometer()[0] = mj_data_->sensordata[dim_motor_sensor_ + 7];
            low_state_hg_.imu_state().accelerometer()[1] = mj_data_->sensordata[dim_motor_sensor_ + 8];
            low_state_hg_.imu_state().accelerometer()[2] = mj_data_->sensordata[dim_motor_sensor_ + 9];
        }

        if (js_)
        {
            GetWirelessRemote();
            memcpy(&low_state_hg_.wireless_remote()[0], &wireless_remote_, 40);
        }

        low_state_hg_puber_->Write(low_state_hg_);
    }
}

void UnitreeSdk2Bridge::PublishHighState()
{
    if (mj_data_ && have_frame_sensor_)
    {

        high_state_.position()[0] = mj_data_->sensordata[dim_motor_sensor_ + 10];
        high_state_.position()[1] = mj_data_->sensordata[dim_motor_sensor_ + 11];
        high_state_.position()[2] = mj_data_->sensordata[dim_motor_sensor_ + 12];

        high_state_.velocity()[0] = mj_data_->sensordata[dim_motor_sensor_ + 13];
        high_state_.velocity()[1] = mj_data_->sensordata[dim_motor_sensor_ + 14];
        high_state_.velocity()[2] = mj_data_->sensordata[dim_motor_sensor_ + 15];

        // 添加时间戳
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        high_state_.stamp().sec() = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
        high_state_.stamp().nanosec() = std::chrono::duration_cast<std::chrono::nanoseconds>(duration % std::chrono::seconds(1)).count();
         
        // 获取脚底接触状态
        int left_ankle_body_id = mj_name2id(mj_model_, mjOBJ_BODY, "left_ankle_roll_link");
        int right_ankle_body_id = mj_name2id(mj_model_, mjOBJ_BODY, "right_ankle_roll_link");
                // 初始化为 CONTACT_UNKNOWN
        int16_t left_foot_contact = CONTACT_UNKNOWN;
        int16_t right_foot_contact = CONTACT_UNKNOWN;

        if (left_ankle_body_id >= 0 && right_ankle_body_id >= 0)
    {
        bool left_contact_detected = false;
        bool right_contact_detected = false;

        // 力阈值：根据模型质量和单位调整（牛顿级）；若接触力非常小请调低
        const double contact_force_threshold = 1e-3;

        // 如果没有接触记录，ncon == 0
        for (int i = 0; i < mj_data_->ncon; i++)
        {
            int geom1 = mj_data_->contact[i].geom1;
            int geom2 = mj_data_->contact[i].geom2;

            int body1 = (geom1 >= 0) ? mj_model_->geom_bodyid[geom1] : -1;
            int body2 = (geom2 >= 0) ? mj_model_->geom_bodyid[geom2] : -1;

            // 忽略同一 body 的自碰撞条目
            if (body1 >= 0 && body1 == body2) continue;

            // mj_contactForce 会写入 6 个值（前三为力，后三为力矩）
            mjtNum force6[6] = {0};
            mj_contactForce(mj_model_, mj_data_, i, force6);

            // 计算力矢量模（使用前三个分量）
            double fx = static_cast<double>(force6[0]);
            double fy = static_cast<double>(force6[1]);
            double fz = static_cast<double>(force6[2]);
            double fmag = std::sqrt(fx*fx + fy*fy + fz*fz);

            // 如果力超过阈值，判断是否与左右脚 body 相关
            if (fmag > contact_force_threshold)
            {
                if (body1 == left_ankle_body_id || body2 == left_ankle_body_id)
                    left_contact_detected = true;
                if (body1 == right_ankle_body_id || body2 == right_ankle_body_id)
                    right_contact_detected = true;
            }

            // 可选调试信息（如需启用，取消注释）
            // printf("contact %d: geom1=%d body1=%d geom2=%d body2=%d f=%g\n", i, geom1, body1, geom2, body2, fmag);
        }

        // 将检测结果映射到状态定义
        left_foot_contact = left_contact_detected ? CONTACT_MADE : CONTACT_LOST;
        right_foot_contact = right_contact_detected ? CONTACT_MADE : CONTACT_LOST;
    }
    else
    {
        // 未能找到 body id（模型名称可能不匹配），保持 UNKNOWN
        left_foot_contact = CONTACT_UNKNOWN;
        right_foot_contact = CONTACT_UNKNOWN;
    }


        // 更新 foot_force
        high_state_.foot_force()[0] = left_foot_contact;  // 左脚
        high_state_.foot_force()[1] = right_foot_contact; // 右脚
        high_state_.foot_force()[2] = 0; // 未使用
        high_state_.foot_force()[3] = 0; // 未使用

        // 保存当前状态为下一帧的上一帧状态
        // prev_left_foot_contact_ = left_foot_contact;
        // prev_right_foot_contact_ = right_foot_contact;

        high_state_puber_->Write(high_state_);
    }
}

void UnitreeSdk2Bridge::PublishWirelessController()
{
    if (js_)
    {
        js_->getState();
        dds_keys_.components.R1 = js_->button_[js_id_.button["RB"]];
        dds_keys_.components.L1 = js_->button_[js_id_.button["LB"]];
        dds_keys_.components.start = js_->button_[js_id_.button["START"]];
        dds_keys_.components.select = js_->button_[js_id_.button["SELECT"]];
        dds_keys_.components.R2 = (js_->axis_[js_id_.axis["RT"]] > 0);
        dds_keys_.components.L2 = (js_->axis_[js_id_.axis["LT"]] > 0);
        dds_keys_.components.F1 = 0;
        dds_keys_.components.F2 = 0;
        dds_keys_.components.A = js_->button_[js_id_.button["A"]];
        dds_keys_.components.B = js_->button_[js_id_.button["B"]];
        dds_keys_.components.X = js_->button_[js_id_.button["X"]];
        dds_keys_.components.Y = js_->button_[js_id_.button["Y"]];
        dds_keys_.components.up = (js_->axis_[js_id_.axis["DY"]] < 0);
        dds_keys_.components.right = (js_->axis_[js_id_.axis["DX"]] > 0);
        dds_keys_.components.down = (js_->axis_[js_id_.axis["DY"]] > 0);
        dds_keys_.components.left = (js_->axis_[js_id_.axis["DX"]] < 0);

        wireless_controller_.lx() = double(js_->axis_[js_id_.axis["LX"]]) / max_value_;
        wireless_controller_.ly() = -double(js_->axis_[js_id_.axis["LY"]]) / max_value_;
        wireless_controller_.rx() = double(js_->axis_[js_id_.axis["RX"]]) / max_value_;
        wireless_controller_.ry() = -double(js_->axis_[js_id_.axis["RY"]]) / max_value_;
        wireless_controller_.keys() = dds_keys_.value;

        wireless_controller_puber_->Write(wireless_controller_);
    }
}

void UnitreeSdk2Bridge::Run()
{
    while (1)
    {
        sleep(2);
    }
}

void UnitreeSdk2Bridge::SetupJoystick(string device, string js_type, int bits)
{
    js_ = new Joystick(device);
    if (!js_->isFound())
    {
        cout << "Error: Joystick open failed." << endl;
        exit(1);
    }

    max_value_ = (1 << (bits - 1));

    if (js_type == "xbox")
    {
        js_id_.axis["LX"] = 0; // Left stick axis x
        js_id_.axis["LY"] = 1; // Left stick axis y
        js_id_.axis["RX"] = 3; // Right stick axis x
        js_id_.axis["RY"] = 4; // Right stick axis y
        js_id_.axis["LT"] = 2; // Left trigger
        js_id_.axis["RT"] = 5; // Right trigger
        js_id_.axis["DX"] = 6; // Directional pad x
        js_id_.axis["DY"] = 7; // Directional pad y

        js_id_.button["X"] = 2;
        js_id_.button["Y"] = 3;
        js_id_.button["B"] = 1;
        js_id_.button["A"] = 0;
        js_id_.button["LB"] = 4;
        js_id_.button["RB"] = 5;
        js_id_.button["SELECT"] = 6;
        js_id_.button["START"] = 7;
    }
    else if (js_type == "switch")
    {
        js_id_.axis["LX"] = 0; // Left stick axis x
        js_id_.axis["LY"] = 1; // Left stick axis y
        js_id_.axis["RX"] = 2; // Right stick axis x
        js_id_.axis["RY"] = 3; // Right stick axis y
        js_id_.axis["LT"] = 5; // Left trigger
        js_id_.axis["RT"] = 4; // Right trigger
        js_id_.axis["DX"] = 6; // Directional pad x
        js_id_.axis["DY"] = 7; // Directional pad y

        js_id_.button["X"] = 3;
        js_id_.button["Y"] = 4;
        js_id_.button["B"] = 1;
        js_id_.button["A"] = 0;
        js_id_.button["LB"] = 6;
        js_id_.button["RB"] = 7;
        js_id_.button["SELECT"] = 10;
        js_id_.button["START"] = 11;
    }
    else
    {
        cout << "Unsupported gamepad." << endl;
    }
}

void UnitreeSdk2Bridge::PrintSceneInformation()
{
    cout << endl;

    cout << "<<------------- Link ------------->> " << endl;
    for (int i = 0; i < mj_model_->nbody; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_BODY, i);
        if (name)
        {
            cout << "link_index: " << i << ", "
                 << "name: " << name
                 << endl;
        }
    }
    cout << endl;

    cout << "<<------------- Joint ------------->> " << endl;
    for (int i = 0; i < mj_model_->njnt; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_JOINT, i);
        if (name)
        {
            cout << "joint_index: " << i << ", "
                 << "name: " << name
                 << endl;
        }
    }
    cout << endl;

    cout << "<<------------- Actuator ------------->> " << endl;
    for (int i = 0; i < mj_model_->nu; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_ACTUATOR, i);
        if (name)
        {
            cout << "actuator_index: " << i << ", "
                 << "name: " << name
                 << endl;
        }
    }
    cout << endl;

    cout << "<<------------- Sensor ------------->> " << endl;
    int index = 0;
    // 多维传感器，输出第一维的index
    for (int i = 0; i < mj_model_->nsensor; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        if (name)
        {
            cout << "sensor_index: " << index << ", "
                 << "name: " << name << ", "
                 << "dim: " << mj_model_->sensor_dim[i]
                 << endl;
        }
        index = index + mj_model_->sensor_dim[i];
    }
    cout << endl;
}

void UnitreeSdk2Bridge::CheckSensor()
{
    num_motor_ = mj_model_->nu;
    dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

    for (int i = dim_motor_sensor_; i < mj_model_->nsensor; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        if (strcmp(name, "imu_quat") == 0)
        {
            have_imu_ = true;
        }
        if (strcmp(name, "frame_pos") == 0)
        {
            have_frame_sensor_ = true;
        }
    }

    if (num_motor_ > NUM_MOTOR_IDL_GO)
    {
        idl_type_ = 1; // unitree_hg
    }
    else
    {
        idl_type_ = 0; // unitree_go
    }
}

void UnitreeSdk2Bridge::GetWirelessRemote()
// anjian yinshe
{
    js_->getState();
    wireless_remote_.btn.components.R1 = js_->button_[js_id_.button["RB"]];
    wireless_remote_.btn.components.L1 = js_->button_[js_id_.button["LB"]];
    wireless_remote_.btn.components.start = js_->button_[js_id_.button["START"]];
    wireless_remote_.btn.components.select = js_->button_[js_id_.button["SELECT"]];
    wireless_remote_.btn.components.R2 = (js_->axis_[js_id_.axis["RT"]] > 0);
    wireless_remote_.btn.components.L2 = (js_->axis_[js_id_.axis["LT"]] > 0);
    wireless_remote_.btn.components.F1 = 0;
    wireless_remote_.btn.components.F2 = 0;
    wireless_remote_.btn.components.A = js_->button_[js_id_.button["A"]];
    wireless_remote_.btn.components.B = js_->button_[js_id_.button["B"]];
    wireless_remote_.btn.components.X = js_->button_[js_id_.button["X"]];
    wireless_remote_.btn.components.Y = js_->button_[js_id_.button["Y"]];
    wireless_remote_.btn.components.up = (js_->axis_[js_id_.axis["DY"]] < 0);
    wireless_remote_.btn.components.right = (js_->axis_[js_id_.axis["DX"]] > 0);
    wireless_remote_.btn.components.down = (js_->axis_[js_id_.axis["DY"]] > 0);
    wireless_remote_.btn.components.left = (js_->axis_[js_id_.axis["DX"]] < 0);

    wireless_remote_.lx = double(js_->axis_[js_id_.axis["LX"]]) / max_value_;
    wireless_remote_.ly = -double(js_->axis_[js_id_.axis["LY"]]) / max_value_;
    wireless_remote_.rx = double(js_->axis_[js_id_.axis["RX"]]) / max_value_;
    wireless_remote_.ry = -double(js_->axis_[js_id_.axis["RY"]]) / max_value_;
}