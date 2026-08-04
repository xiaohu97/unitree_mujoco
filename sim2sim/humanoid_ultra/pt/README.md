# Humanoid Ultra Sim2Sim 策略资产

目录布局沿用 `humanoid_rl_controllers/script/pt`。运行
`../sim2sim.py` 时默认从这里加载 walk、stand 和所有命名 Mimic 动作。

Mimic 策略必须与同目录的训练参考 NPZ 成对更新：

- `zxh-mimic-pick`: `policy.pt` + `ustc1_pick_stand_transition.npz`
- `zxh-mimic-houtaitui`: `policy.pt` + `ustc1_rightstand_stand_transition.npz`
- `zxh-mimic-spin`: `policy.pt` + `ustc1_spin_stand_transition_hold_2p5s.npz`
- `zxh-mimic-taitui-left`: `policy.pt` + `ustc_taitui_left_stand_transition.npz`

当前来源：

- walk: `humanoidultra27dof_flat/2026-07-04_12-01-28_continue_20k/exported/policy.pt`
- stand-leftarm: `humanoidultra27dof_stand_leftarm/2026-06-30_19-24-09/exported/policy.pt`
- pick: `ustc_humanoid_ultra_27dof_mimic_pick/2026-08-04_09-59-48/exported/policy.pt`
- houtaitui: `ustc_humanoid_ultra_27dof_mimic_houtaitui/exported/policy.pt`
- spin: `ustc_humanoid_ultra_27dof_mimic_spin/2026-08-02_00-21-31_spin_hold2p5_frame0p25_ft_from38000_v2_novis/exported/policy.pt`
- taitui-left: `ustc_humanoid_ultra_27dof_mimic_taitui_left/2026-08-03_15-40-26/exported/policy.pt`

新增动作时，在 `sim2sim.py` 的 `DEFAULT_MIMIC_ACTIONS` 表中增加
`MimicActionSpec`，并为它分配唯一的名字、手柄触发键和键盘键。新训练的
144 维策略还应设置 `use_current_asset_defaults=True`；旧 144 维策略保持为
`False`。

旧的 6 月 13 日 walk 策略保存在
`zxh-walk/2026-06-13_old/policy.pt`，不参与默认加载。
