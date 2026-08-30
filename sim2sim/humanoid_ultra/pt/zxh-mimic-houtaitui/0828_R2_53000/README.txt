R2@53000 = logs/rsl_rl/ustc_humanoid_ultra_27dof_mimic_houtaitui_0808drift/
           2026-08-27_21-05-1x_r2_0808drift_from_R1_51k/model_53000.pt

血统: 0808 (2026-08-08_18-06-58/model_49999) -> R1 (加 feet_contact_force)
      -> R2 (加 anchor_pos_xy 漂移约束)

配置 = 0808 的 16 项奖励 + feet_contact_force_excess(阈值 2.0 体重, 权重 -20)
       终止 5 项: time_out / anchor_pos / anchor_ori / ee_body_pos / anchor_pos_xy(0.35)
       不含 anchor_yaw, 不含手臂奖励 -- 这两项是 0820 之后髋 yaw 偏转的来源
plant: 辨识 URDF + USTCActuator 力矩-速度曲线 + armature 0.02 + hip_yaw kd 1.6 + 延迟 1-3

离线评测 (scene_27dof_identified.xml, 100 seed, 复位 +-0.5, 延迟 3 步 = 15 ms)
两组不重叠的种子:
                  种子 0-99    种子 100-199
  跌倒              3/100         2/100
  漂移中位          0.252         0.282
  漂移 p90          0.682         0.485
  触地中位          1.40          1.37    (体重倍数)
  触地峰值          1.95          1.96
  抬腿峰值          0.564         0.565   (参考 0.542)
  右髋 yaw 偏差     -21.9°  (40 seed 中位, 参考轨迹 0 -> +47.6°)

对照 0725 (实机现用):
  跌倒 0/100  漂移 0.410  触地 2.15/7.49  抬腿 0.547  右髋 -20.6°

即: 漂移小 39%, 触地中位小 35%, 触地峰值小 3.8 倍, 抬腿略高, 髋 yaw 持平,
    代价是 2-3% 的跌倒率 (0725 是 0)。

注意: 同一 run 的 model_54500 右髋是 -52.8°, 明显劣化。checkpoint 之间髋 yaw 有波动,
      换点必须单独复验, 不能假定同 run 的点都合格。
