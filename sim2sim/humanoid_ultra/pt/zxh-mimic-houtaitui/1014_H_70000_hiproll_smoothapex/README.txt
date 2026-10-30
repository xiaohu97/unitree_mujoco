H@70000  houtaitui 无负载, 髋 roll 零点随机化 +-0.03  (2026-10-14, 未上机)
checkpoint = /home/zxh/unitree_rl_lab/logs/rsl_rl/ustc_humanoid_ultra_27dof_mimic_houtaitui_stance_hiproll/2026-10-14_01-38-41_hiproll003_from_z3_63000/model_70000.pt  (sha256 e3248aabfede0774...)
task = USTC-Humanoid-Ultra-27dof-Mimic-houtaitui-stance-hiproll ; 从 0929 (Z3@63000) 续训 7000 迭代, 只改零点随机化:
  髋 roll +-0.01 -> +-0.03 (膝/踝 +-0.03, 其余 +-0.01 不变)
必须配本目录的平滑参考 npz (md5 fe9bc73467)。

离线评测 (sim2sim 辨识模型, 延迟 3 步, 从行走切入 = 只对齐 yaw, 每组 50 种子):
                                     0929        本策略
  放下又抬 从行走切入                 7           7
  放下又抬 从行走切入+零点偏差(髋roll/膝/踝 +-0.03)  34 (跌 2)   17 (跌 2)
  放下又抬 第0帧起步+零点偏差          30          17
  单腿站倾斜误差 (从行走)              2.6 deg     2.1 deg
  触地 中位/最大                       1.6/2.5     1.6/2.3 BW
  抬腿高度 (参考 0.542 m)              0.543       0.520-0.528   <- 略低约 2 cm
单个零点误差下左脚落点横偏 (从行走无推力 20 次, +为偏左):
  无误差 18 -> 3.6 cm; 左髋roll+0.03 38 -> 22 cm; 右髋roll+0.03 跌倒 8/20 -> 4/20, 最大左倾 13.4 -> 2.1 deg
  右踝 roll +-0.03 仍偏 50-63 cm (踝 roll 原本就是 +-0.03, 没有改善) -> 实机仍需核对踝 roll 零点

实机: python3 sim2real_humanoidultra27dof_walk.py --houtaitui-model pt/zxh-mimic-houtaitui/<目录>/policy.pt \
        --houtaitui-motion pt/zxh-mimic-houtaitui/<目录>/ustc1_rightstand_stand_transition_smoothapex.npz

上机录数据 (每次试 houtaitui 都录一份, 发给分析):
  1) 终端 A: 先起控制器 (见上面 "实机" 命令), 站稳在 walk 状态
  2) 终端 B: 开始录制, 再在手柄上按 LT+下 触发 houtaitui, 动作结束 (或倒地 / 保护触发) 后 Ctrl+C 停止
     rosbag record -O houtaitui_0929rollfix_$(date +%m%d_%H%M%S).bag /leg/jointsPosVel /leg/targetPos /imu
     建议多录这三个 (腰部关节用于还原躯干朝向, 实际力矩用于看哪个电机吃力):
     rosbag record -O houtaitui_0929rollfix_$(date +%m%d_%H%M%S).bag /leg/jointsPosVel /leg/targetPos /imu /waist/jointsPosVel /leg/realTorque /leg/targetTorque
  3) 同时把控制器终端的输出另存成 txt (含 tilt error / EtherCAT 报错), 和 .bag 一起发
  检查录到了数据: rosbag info <文件>.bag   (每个话题的消息数都应 > 0)
