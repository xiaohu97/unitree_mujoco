# Humanoid Ultra Sim2Sim 策略资产

目录结构与 `humanoid_rl_controllers/script/pt` 保持一致。运行
`../sim2sim.py` 时默认从这里加载 walk、stand 和所有命名 Mimic 动作。

Mimic 策略必须与同目录的训练参考 NPZ 成对更新：

- `zxh-mimic-pick`: `policy.pt` + `ustc1_pick_stand_transition.npz`
- `zxh-mimic-houtaitui`: `policy.pt` + `ustc1_rightstand_stand_transition.npz`

当前来源：

- walk: `humanoidultra27dof_flat/2026-07-04_12-01-28_continue_20k/exported/policy.pt`
- stand-leftarm: `humanoidultra27dof_stand_leftarm/2026-06-30_19-24-09/exported/policy.pt`
- pick: `ustc_humanoid_ultra_27dof_mimic_pick/exported/policy.pt`
- houtaitui: `ustc_humanoid_ultra_27dof_mimic_houtaitui/exported/policy.pt`

新增动作时，在 `sim2sim.py` 的 `DEFAULT_MIMIC_ACTIONS` 表中增加
`MimicActionSpec`，并为它分配唯一的名字和手柄触发键。

旧的 6 月 13 日 walk 策略保存在
`zxh-walk/2026-06-13_old/policy.pt`，不参与默认加载。
