HP@80998  houtaitui 左手 2.5 kg 负载, 髋 roll 零点随机化 +-0.03  (2026-10-14, 未上机)
checkpoint = /home/zxh/unitree_rl_lab/logs/rsl_rl/ustc_humanoid_ultra_27dof_mimic_houtaitui_stance_hiproll_2_5kg/2026-10-14_01-52-47_hiproll003_2p5kg_from_z3p_72999/model_80998.pt  (sha256 84100c4217b480e8...)
task = USTC-Humanoid-Ultra-27dof-Mimic-houtaitui-stance-hiproll-2-5kg ; 从 Z3P@72999 续训 8000 迭代, 只改零点随机化 (同无负载版)。
必须配本目录的平滑参考 npz (md5 fe9bc73467)。

离线评测 (负载场景, 延迟 3 步, 每组 50 种子):
                                     Z3P@72999   本策略
  放下又抬 从行走切入                 7           8
  放下又抬 从行走切入+零点偏差         25 (跌 1)   11 (跌 0)
  单腿站倾斜误差 (从行走)              3.5 deg     3.8 deg
  触地 中位/最大 (从行走)              1.50/2.22   1.38/2.53 BW
  落地后右脚挪动 (从行走, 参考 6.4 cm) 16.6 cm     15.2 cm
  抬腿高度                             0.545       0.527-0.529
单个零点误差下左脚落点横偏 (从行走无推力 20 次, +为偏左):
  无误差 11 -> -1.5 cm; 右髋roll+0.03 跌倒 7/20 落点 -68 cm -> 跌倒 0 落点 -6 cm; 左髋roll+0.03 27 -> -2 cm
  右踝 roll +-0.03 仍偏 38-66 cm (未改善)
第0帧起步(复位 +-0.5 m/s 初速)的跌倒 3/50 都在开局站立段。

实机: python3 sim2real_humanoidultra27dof_walk.py --houtaitui-model pt/zxh-mimic-houtaitui/<目录>/policy.pt \
        --houtaitui-motion pt/zxh-mimic-houtaitui/<目录>/ustc1_rightstand_stand_transition_smoothapex.npz

上机录数据 (每次试 houtaitui 都录一份, 发给分析):
  1) 终端 A: 先起控制器 (见上面 "实机" 命令), 站稳在 walk 状态
  2) 终端 B: 开始录制, 再在手柄上按 LT+下 触发 houtaitui, 动作结束 (或倒地 / 保护触发) 后 Ctrl+C 停止
     rosbag record -O houtaitui_0929rollfix2-5kg_$(date +%m%d_%H%M%S).bag /leg/jointsPosVel /leg/targetPos /imu
     建议多录这三个 (腰部关节用于还原躯干朝向, 实际力矩用于看哪个电机吃力):
     rosbag record -O houtaitui_0929rollfix2-5kg_$(date +%m%d_%H%M%S).bag /leg/jointsPosVel /leg/targetPos /imu /waist/jointsPosVel /leg/realTorque /leg/targetTorque
  3) 同时把控制器终端的输出另存成 txt (含 tilt error / EtherCAT 报错), 和 .bag 一起发
  检查录到了数据: rosbag info <文件>.bag   (每个话题的消息数都应 > 0)
