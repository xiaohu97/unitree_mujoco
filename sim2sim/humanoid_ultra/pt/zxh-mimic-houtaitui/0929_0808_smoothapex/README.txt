0808 策略 + 平滑顶点参考  (2026-09-29)
======================================
policy.pt   = 0808/policy.pt 原样复制 (logs/rsl_rl/ustc_humanoid_ultra_27dof_mimic_houtaitui/
              2026-08-08_18-06-58/model_49999.pt), 权重没有任何改动
参考 npz    = ustc1_rightstand_stand_transition_smoothapex.npz (md5 fe9bc73467),
              与原始参考只在 477-523 帧不同: 踢腿顶点的 "0.54 -> 0.42 -> 0.49 m"
              (掉 12 cm 再回抬 6.7 cm) 换成单峰平滑下落, 起跳/落地时刻不变

为什么: 0808 是目前唯一 "只抬一次、单腿站稳、实机验证过" 的策略, 它原来会跟着
原始参考在顶点下沉再回抬约 3.4 cm。策略观测里带参考指令, 换参考不用重训。

离线评测 (scene_27dof_identified.xml, 复位扰动+推力, 延迟 3 步, sim2sim 部署裁剪):
                         原始参考          平滑参考
  顶点回抬 (中位)       3.4-3.5 cm        0.3 cm        (延迟4步: 4.4 -> 1.1)
  放下又抬 (种子数)     3/50, 4/50        5/50, 5/50
  单腿站倾斜误差        2.4 deg           2.2 deg
  支撑脚压力中心边距    0.70-0.75 cm      1.34-1.36 cm  (越大离脚边越远)
  跌倒                  0/100             0/100
  触地 中位/最大        2.2 / 5.7-6.4     2.0-2.2 / 5.5-6.6 BW
  漂移                  0.34-0.39 m       0.35-0.41 m
无扰动回放: 回抬 3.0 -> 0.3 cm, 压力中心边距 1.24 -> 2.12 cm, 都不放脚。

注意: 膝/踝零点偏差 +-0.03 rad 时 0808 (两种参考都一样) 会在约一半种子里
中途放脚、落地后迈一大步 (支撑脚位移中位 40-50 cm)。上机前先核一遍关节零点。

sim2sim 查看:
  cd /home/zxh/ustc_humanoid/unitree_mujoco/sim2sim/humanoid_ultra
  python sim2sim.py --mode mimic --mimic-current-asset-defaults \
    --policy pt/zxh-mimic-houtaitui/0929_0808_smoothapex/policy.pt \
    --motion-file pt/zxh-mimic-houtaitui/0929_0808_smoothapex/ustc1_rightstand_stand_transition_smoothapex.npz \
    --scene /home/zxh/ustc_humanoid/unitree_mujoco/unitree_robots/humanoid_ultra/scene_27dof_identified.xml

实机 (不替换任何现有文件, 把本目录拷到控制器 script/pt/ 下后用参数指定):
  python3 sim2real_humanoidultra27dof_walk.py \
    --houtaitui-model   pt/zxh-mimic-houtaitui/0929_0808_smoothapex/policy.pt \
    --houtaitui-motion  pt/zxh-mimic-houtaitui/0929_0808_smoothapex/ustc1_rightstand_stand_transition_smoothapex.npz
