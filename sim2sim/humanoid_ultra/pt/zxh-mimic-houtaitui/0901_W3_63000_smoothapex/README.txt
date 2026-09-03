houtaitui W3 平滑顶点参考 / checkpoint 63000
==============================================

用途
----
和 0829_V1_55500_tightroll 配对做 A/B:两条 run 除参考动作外配置逐项相同
(compare_mimic_runs.py 的 config fingerprint 17 项全同),用来看"把踢腿顶点的
重定向抖动去掉"对策略行为的影响。选 63000 不是选最终 checkpoint,是因为它在
27 点名义 rollout 扫描里顶点回抬最小(0.002 m)、身体下沉最少。

来源
----
run:
  /home/zxh/unitree_rl_lab/logs/rsl_rl/ustc_humanoid_ultra_27dof_mimic_houtaitui_tightroll_smoothapex/2026-09-01_14-39-24_w3_smoothapex_from_0808
task:
  USTC-Humanoid-Ultra-27dof-Mimic-houtaitui-tightroll-smoothapex
  (= RobotHoutaituiTightRollEnvCfg,只把 motion_file 换成平滑版)
checkpoint:
  model_63000.pt
checkpoint SHA256:
  78cce1bffc45463914072ae37331860cc96d3eaa2fff0146966acdf570a18fb0
policy SHA256:
  543240cd2aef103584a128749dee1218c63e73ad87960695b073b26d5376e21c
raw actor SHA256:
  1ddc0a19ed5db353e74e93e567573d8e251e2daa5805035230160b83b6a5826a
warm start:
  0808 的 model_49999.pt,与 V1 同一父 checkpoint,同样跑 49999 -> 69999
network:
  144 -> 512 -> 256 -> 128 -> 27, ELU, no empirical normalizer
  policy.pt 与 0829 那版同构:normalizer -> actor -> clamp(action)。
  action_lower/action_upper 由本 run 的 params/env.yaml 现算
  ((关节限位 - 默认角) / 0.25),27 项与 0829 包内的缓冲区逐位相同。
  actor 权重与 checkpoint 逐元素一致。

参考动作
--------
ustc1_rightstand_stand_transition_smoothapex.npz(随包附带)
仅 477-523 帧(9.54-10.46 s)与原参考不同,其余逐位相同。原参考在踢腿顶点
掉 12 cm 再回抬 6.5 cm,平滑后单峰。生成工具与校验见
unitree_rl_lab/source/.../ustc1_rightstand/APEX_SMOOTH.md。

sim2sim 并排对比(名义 rollout,941 帧,各自跑自己的参考)
--------------------------------------------------------
                              V1@55500      w3@63000
  抬腿峰值                     0.573 m       0.527 m
  顶点回抬                     0.007 m       0.002 m
  中途落地                       否            否
  最低 base_z                  0.977 m       0.979 m
  脚高跟踪 RMSE (9.0-10.9 s)   0.033 m       0.015 m
  左/右踝 roll 目标顶限位       53% / 45%     98% / 92%

w3 跟踪更准、踢腿期间躯干更稳(base_z 全程 1.041-1.045,V1 掉到 1.004),
但抬腿低 4.6 cm。

27 点 checkpoint 扫描(55500-69998,每点一次名义 rollout)
  干净点(不中途落地且回抬<0.05):V1 2/27,w3 5/27
  顶点回抬中位:V1 0.350 m,w3 0.250 m(Mann-Whitney p=0.007)

安全警告
--------
**这一版的踝 roll 逃逸比 V1 更严重,不要直接上实机。**
在上面同一条 rollout 记录的观测上跑裸 actor:

                        V1@55500              w3@63000
  raw 目标峰值 左/右    0.91 / 0.79 rad       1.00 / 0.51 rad
                        (52 / 45 deg)         (57 / 29 deg)
  超出 ±0.20 的均值     0.067 / 0.030 rad     0.112 / 0.070 rad
  超限步数占比          53% / 46%             98% / 92%

即 w3@63000 几乎每一步都在顶着 ±0.20 rad 的裁剪,平均超出量是 V1 的 1.7-2.3 倍。
policy.pt 自带的裁剪能在 sim2sim 里兜住,但实机控制器外层是 ±0.5236 rad
(±30°),两版通过那个更宽的裁剪都会命令到 -30°,回到 S1/T1/U1 的失败姿态。
这正是 RawBoundedRollRewardsCfg / rawroll 那条线要解决的问题——平滑参考没有
解决它,反而更严重。

要上实机应当先在 rawroll(raw_action_excess)配置下用平滑参考重训,而不是
直接部署本包。policy_raw_unclipped.pt 仅供上述分析对照,禁止用于实机。

推荐的 sim2sim 命令
--------------------
cd /home/zxh/ustc_humanoid/unitree_mujoco
conda activate gmr
python sim2sim/humanoid_ultra/sim2sim.py \
  --mode mimic \
  --dof 27 \
  --policy sim2sim/humanoid_ultra/pt/zxh-mimic-houtaitui/0901_W3_63000_smoothapex/policy.pt \
  --motion-file sim2sim/humanoid_ultra/pt/zxh-mimic-houtaitui/0901_W3_63000_smoothapex/ustc1_rightstand_stand_transition_smoothapex.npz \
  --mimic-current-asset-defaults \
  --mimic-ankle-roll-limit 0.20 \
  --scene unitree_robots/humanoid_ultra/scene_27dof_identified.xml

本机已跑完 941 帧 headless nominal rollout:最终 base_z=1.007 m。
