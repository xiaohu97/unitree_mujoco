Z3@63000  houtaitui 无负载 "不放脚" 候选  (2026-09-29, 未上机)
==========================================================
checkpoint = /home/zxh/unitree_rl_lab/logs/rsl_rl/ustc_humanoid_ultra_27dof_mimic_houtaitui_noplant_refroll_stance/
             2026-09-29_15-24-52_z3_stance_from_z2_54000/model_63000.pt   (sha256 9fc0da9b48c2f45e...)
task       = USTC-Humanoid-Ultra-27dof-Mimic-houtaitui-noplant-refroll-stance
policy.pt  = actor 原样导出 (无内嵌裁剪, 与 0808/W1 的导出方式相同)
参考 npz   = 平滑顶点参考 (md5 fe9bc73467)。本策略在平滑参考上训练, 必须配这个 npz 用。

来历: 0808 -> Z2 (5500 迭代) -> Z3 (9000 迭代), 相对 rawroll 线 (W1/X2) 的改动:
  - 平滑顶点参考 (去掉参考自身 12 cm 下沉 + 6.7 cm 回抬)
  - 去掉 0.35 m 漂移硬终止 (策略看不到水平位置, 这一项逼出了 "顶点放脚")
  - 参考脚离地 > 0.15 m 时摆动脚触地扣分 (swing_foot_contact_penalty -10)
  - 参考脚着地 (< 0.10 m) 时这只脚离地扣分 (stance_foot_lift_penalty -10), 去掉落地后的迈步
  - 踝 roll 原始动作限幅以参考为中心 +-0.20 rad (站立时与原来相同; 踢腿时支撑踝能跟到 -12 度)
  - 其余同 rawroll: 落地力惩罚 (>2 BW), 踝 pitch 力矩惩罚, 膝踝零点随机化 +-0.03

离线评测 (scene_27dof_identified.xml, 延迟 3 步, sim2sim 部署裁剪, 推力照旧):
                         0808+平滑参考     本策略
  放下又抬 (标称 100)      10               3
  放下又抬 (零点偏差 50)   24               6
  单腿站倾斜误差           2.2 deg          2.1-2.2 deg
  支撑脚压力中心离脚边     1.3 cm           1.9 cm
  触地 中位/最大           2.0-2.2 / 5.5-6.6 BW    1.6 / 2.0-2.2 BW
  落地后支撑脚位移         4-5 cm           9-10 cm   (参考本身 6.4 cm)
  漂移                     0.35-0.41 m      0.33-0.39 m
  跌倒 (250 次)            0                5
平稳起步 (复位初速 +-0.1 m/s, 更接近实机从行走切入), 100 种子:
                         0808+平滑参考     本策略
  跌倒                     0/100            1/100
  放下又抬                 10/100           9/100
  触地 中位/最大           2.27 / 6.54      1.65 / 2.31 BW
  漂移                     0.29 m           0.23 m

已知不足: 5/250 次跌倒全部发生在开局 1-4 s 的站立段 (评测脚本复位时给 +-0.5 m/s 初速、
+-0.1 rad 姿态/关节扰动), 踢腿/单腿站/落地阶段一次没有。原因是 "参考脚着地时不许离地"
这一项也罚了被强推后救急的那一步; 0808 在同样复位下 0/250。Z6 (只在 440-650 帧罚) 在训练中。
站立段逐关节检查: 踝 roll 指令 左 +0.8 / 右 +0.1 度 (S1 是 -29.5, 0820 -28), 踝 pitch -12.0 / -14.3
(R1 是 -45.7) -- 没有实机失败过的那两种姿态问题。

sim2sim 查看:
  cd /home/zxh/ustc_humanoid/unitree_mujoco/sim2sim/humanoid_ultra
  python sim2sim.py --mode mimic --mimic-current-asset-defaults \
    --policy pt/zxh-mimic-houtaitui/0929_Z3_63000_noplant_smoothapex/policy.pt \
    --motion-file pt/zxh-mimic-houtaitui/0929_Z3_63000_noplant_smoothapex/ustc1_rightstand_stand_transition_smoothapex.npz \
    --scene /home/zxh/ustc_humanoid/unitree_mujoco/unitree_robots/humanoid_ultra/scene_27dof_identified.xml

实机 (不替换现有文件, 把本目录拷到控制器 script/pt/zxh-mimic-houtaitui/ 下后用参数指定):
  python3 sim2real_humanoidultra27dof_walk.py \
    --houtaitui-model   pt/zxh-mimic-houtaitui/0929_Z3_63000_noplant_smoothapex/policy.pt \
    --houtaitui-motion  pt/zxh-mimic-houtaitui/0929_Z3_63000_noplant_smoothapex/ustc1_rightstand_stand_transition_smoothapex.npz
