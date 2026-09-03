W2@63000  (2.5kg 左臂负载)
= logs/rsl_rl/ustc_humanoid_ultra_27dof_mimic_houtaitui_rawroll_2_5kg/
  2026-08-31_12-25-15_w2_rawroll_2p5kg_from_V1_56500/model_63000.pt

配置与 0831_W1_61000 完全相同，只有 plant 不同:
  URDF = humanoid_ultra_27dof_description_identified_leftarm2-5kg.urdf
两条线只差这一处，所以差异可以归到负载上。

任务 id: USTC-Humanoid-Ultra-27dof-Mimic-houtaitui-rawroll-2-5kg
（不是旧的 -2-5kg，那个继承的是没有任何修复的旧基类）

离线评测 (scene_27dof_identified_leftarm2p5kg.xml, 100 seed, 复位 ±0.5,
延迟 3 步 = 15 ms, sim2sim 原生 ±30° 裁剪):
  跌倒        1/100      <- 比无负载线还稳
  漂移中位    0.294
  触地中位    1.74       (体重倍数)
  触地峰值    2.48
  抬腿峰值    0.557
  站立段踝 roll 指令  左 -1.4°  右 +0.1°

同批其他点: @60000 跌倒 8, @61500 跌倒 4, @63500 抬腿 0.514 跌破线不要用。

尚未做双种子复验（W1@61000 做过，4/100 与 4/100 一致）。
上实机前建议先补一次。
