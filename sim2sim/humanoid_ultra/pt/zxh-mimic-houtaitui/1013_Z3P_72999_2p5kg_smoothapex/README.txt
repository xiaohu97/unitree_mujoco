Z3P@72999  houtaitui 左手 2.5 kg 负载版  (2026-10-13, 未上机)
==========================================================
checkpoint = /home/zxh/unitree_rl_lab/logs/rsl_rl/ustc_humanoid_ultra_27dof_mimic_houtaitui_noplant_refroll_stance_2_5kg/2026-09-29_21-49-46_z3p_2p5kg_from_z3_63000/model_72999.pt
             (sha256 f3065244a393adff...)
task       = USTC-Humanoid-Ultra-27dof-Mimic-houtaitui-noplant-refroll-stance-2-5kg
policy.pt  = actor 原样导出 (无内嵌裁剪)
参考 npz   = 平滑顶点参考 (md5 fe9bc73467), 必须配这个 npz 用
来历: 无负载 Z3@63000 (0929 目录那版) 换 2.5 kg 左手负载模型继续训练 10000 迭代; 奖励/终止与 Z3 相同。

离线评测 (scene_27dof_identified_leftarm2p5kg.xml, 延迟 3 步, 部署裁剪, 推力照旧, 每组 50 种子):
                              旧负载 W2@73000   旧负载 Y2@94000   本策略
  放下又抬: 标称/平稳起步/行走切入   37/40/18          30/32/33          2/3/7
  放下又抬: 行走切入无推力 (30)      -                 -                 0
  单腿站倾斜误差                     5.5-5.9 deg       4.5-5.8 deg       2.6-3.5 deg
  触地 中位/最大                     1.6-1.8/2.0-2.1   1.9/2.1-2.2       1.4-1.5/2.0-2.2 BW
  落地后右脚挪动 (参考 6.4 cm)       3-5 cm            4-8 cm            15-19 cm
  漂移                               0.15-0.24 m       0.16-0.25 m       0.22-0.41 m
  从行走切入 跌倒                    0/50              0/50              0/130
另: 第二组种子 / 零点偏差 / 延迟 2 步 / 延迟 4 步: 放下又抬 2/7/2/2 (每组 50)。
从参考第 0 帧起步的 300 次里倒了 6 次, 全部在开局 1.4-2.4 s 站立段 (评测复位给 +-0.5 m/s 初速),
无推力参与; 踢腿/落地阶段 0 次。站立段踝 roll 指令 左 -0.1 / 右 -0.0 度, 姿态干净。
注意: 上表 "行走切入" 用的是只对齐 yaw 的启动方式 (控制器 walk.py 已于 10-13 改为此方式)。

sim2sim 查看:
  cd /home/zxh/ustc_humanoid/unitree_mujoco/sim2sim/humanoid_ultra
  python sim2sim.py --mode mimic --mimic-current-asset-defaults \
    --policy pt/zxh-mimic-houtaitui/1013_Z3P_72999_2p5kg_smoothapex/policy.pt \
    --motion-file pt/zxh-mimic-houtaitui/1013_Z3P_72999_2p5kg_smoothapex/ustc1_rightstand_stand_transition_smoothapex.npz \
    --scene /home/zxh/ustc_humanoid/unitree_mujoco/unitree_robots/humanoid_ultra/scene_27dof_identified_leftarm2p5kg.xml
