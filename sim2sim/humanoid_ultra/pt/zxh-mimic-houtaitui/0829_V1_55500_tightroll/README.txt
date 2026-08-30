houtaitui V1 tight-roll / checkpoint 55500
===========================================

用途
----
这是 V1 tight-roll 训练中优先用于 sim2sim 验证的一版。选择 55500 而不是
尚未离线评测的最终 checkpoint 69998，也没有覆盖上一级目录现有的 policy.pt。

来源
----
run:
  /home/zxh/unitree_rl_lab/logs/rsl_rl/ustc_humanoid_ultra_27dof_mimic_houtaitui_tightroll/2026-08-28_21-32-10_v1_tightroll_from_0808
checkpoint:
  model_55500.pt
checkpoint SHA256:
  8ff1dda8acf70f04fb03f762efe5f24866a993e77b445a3b5fe4be9446a63bba
policy SHA256:
  6f28470f9080323088c73765a0cf9d2dab516b57727730383984d460c263e802
raw actor SHA256:
  1e85f945df5599c85c5fc33c511917e5a79de223dde86feaa5b854290438bdf9
network:
  144 -> 512 -> 256 -> 128 -> 27, ELU, no empirical normalizer
  policy.pt 在 actor 输出后内嵌了训练时的完整 27 关节目标限位换算。
  actor 权重与 checkpoint 逐元素一致；限位是显式部署后处理，不是重新训练。

离线评测（训练一致的 ankle-roll +/-0.20 rad 裁剪）
---------------------------------------------------
seed 0-99:     跌倒 5/100，漂移中位 0.253 m，触地 1.72/2.22 BW，抬腿峰值 0.554 m
seed 100-199:  跌倒 4/100，漂移中位 0.303 m，触地 1.66/2.21 BW，抬腿峰值 0.559 m
合并结论:      跌倒 9/200 (4.5%)；两组落地峰值一致，故优先于 56500 做轻落地验证。

0808 基线（100 seeds）为：跌倒 0/100，漂移 0.371 m，触地 2.12/7.75 BW，
抬腿 0.543 m。V1 明显减轻了落地并改善漂移，但跌倒率从 0 上升到约 4.5%，
所以尚不能称为实机就绪。

推荐的 sim2sim 命令
--------------------
cd /home/zxh/ustc_humanoid/unitree_mujoco
conda activate gmr
python sim2sim/humanoid_ultra/sim2sim.py \
  --mode mimic \
  --dof 27 \
  --policy sim2sim/humanoid_ultra/pt/zxh-mimic-houtaitui/0829_V1_55500_tightroll/policy.pt \
  --motion-file sim2sim/humanoid_ultra/pt/zxh-mimic-houtaitui/ustc1_rightstand_stand_transition.npz \
  --mimic-current-asset-defaults \
  --mimic-ankle-roll-limit 0.20 \
  --scene unitree_robots/humanoid_ultra/scene_27dof_identified.xml

无窗口快速检查可在末尾添加：
  --headless

本机已跑完 941 帧 headless nominal rollout：最终 base_z=1.004 m。

安全警告
--------
此策略训练和正式评测时都依赖左右 ankle-roll 绝对关节目标 +/-0.20 rad
(约 +/-11.46 deg)。原 sim2sim 和当前实机控制器外层默认是 +/-0.5236 rad
(+/-30 deg)。裸 actor 会把 ankle-roll 推到约 -25 至 -29 deg。

主文件 policy.pt 已把保存的完整训练目标限位嵌入 JIT，因此复制该文件时限位会
一起带走；命令中的 --mimic-ankle-roll-limit 0.20 是严格复现正式评测口径的
第二层显式保护。policy_raw_unclipped.pt 仅供分析对照，禁止用于 sim2sim 默认
多策略入口或实机。

即使主 policy.pt 已自带限位，它仍有 9/200 (4.5%) 的离线跌倒率，并且网络的
ankle-roll 原始输出仍长期顶着限位。这版先做 sim2sim；若要上实机，必须吊绳、
低风险姿态、可用急停，并先排除 Bad WKC/Slave lost 等 EtherCAT 故障。
