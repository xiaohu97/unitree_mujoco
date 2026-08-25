# Humanoid Ultra Sim2Sim 策略资产

目录布局沿用 `humanoid_rl_controllers/script/pt`。运行
`../sim2sim.py` 时默认从这里加载 walk、stand 和所有命名 Mimic 动作。

Mimic 策略必须与同目录的训练参考 NPZ 成对更新：

- `zxh-mimic-pick`: `policy.pt` + `ustc1_pick_stand_transition.npz`
- `zxh-mimic-houtaitui`: `policy.pt` + `ustc1_rightstand_stand_transition.npz`
- `zxh-mimic-spin`: `policy.pt` + `ustc1_spin_stand_transition_hold_2p5s.npz`
- `zxh-mimic-taitui-left`: `policy.pt` + `ustc_taitui_left_stand_transition.npz`
- `zxh-mimic-taitui-right`: `policy.pt` + `ustc_taitui_right_stand_transition.npz`

当前来源：

- walk: `humanoidultra27dof_identified_flat/2026-07-26_11-42-21/model_41500.pt`
- stand-leftarm: `humanoidultra27dof_identified_stand_leftarm/2026-07-27_09-57-45/model_49999.pt`
- pick: `ustc_humanoid_ultra_27dof_mimic_pick/2026-08-04_09-59-48/model_24000.pt`
- houtaitui: `ustc_humanoid_ultra_27dof_mimic_houtaitui_yawarm/2026-08-20_20-41-18_yawarm_from_A36k/model_42000.pt` (2026-08-24)
- taitui-right: `ustc_humanoid_ultra_27dof_mimic_taitui_right/2026-08-23_16-33-59_lowent_from_17500/model_28000.pt` (2026-08-24, new action)
- spin: `ustc_humanoid_ultra_27dof_mimic_spin/2026-08-04_20-15-50/exported/policy.pt` (`model_43000.pt`)
- taitui-left: `ustc_humanoid_ultra_27dof_mimic_taitui_left/2026-08-03_15-40-26/model_44500.pt`

本次替换前的策略保留在原目录：

- stand-leftarm `model_30000`: `zxh-stand-leftarm/policy_model_30000_previous.pt`
- pick `model_17500`: `zxh-mimic-pick/policy_model_17500_previous.pt`

新增动作时，在 `sim2sim.py` 的 `DEFAULT_MIMIC_ACTIONS` 表中增加
`MimicActionSpec`，并为它分配唯一的名字、手柄触发键和键盘键。新训练的
144 维策略还应设置 `use_current_asset_defaults=True`；旧 144 维策略保持为
`False`。

旧的 6 月 13 日 walk 策略保存在
`zxh-walk/2026-06-13_old/policy.pt`，不参与默认加载。

## 2026-08-24 replacement

Both new entries were picked by 100-seed MuJoCo rollouts under the training
reset spread (+-0.5 m/s) and the recurring push, on
`scene_27dof_identified.xml`, not by TensorBoard: on these tasks the logged
curves and the offline fall rate are close to uncorrelated, and individual
checkpoints are occasional outliers in both directions, so every candidate was
scored against its neighbours.

| policy | falls / 100 | swing peak (reference) | replaced |
|---|---:|---|---|
| houtaitui `model_42000` | 3 | 0.549 (0.542) | 07-24 v3 `model_37500`: 0 falls but **peak 0.162** -- it never lifted the leg, it just stood |
| taitui-right `model_28000` | 18 | 0.868 (0.850) | nothing; this action is new |
| taitui-left (unchanged) | 22 | 0.866 (0.822) | a 20k-iteration fine-tune reached only 31, so `2026-08-03_15-40-26/model_44500` stays |

The previous houtaitui policy is kept as
`zxh-mimic-houtaitui/policy_v3_model_37500_previous.pt`.

`houtaitui` flipped to `use_current_asset_defaults=True` in
`DEFAULT_MIMIC_ACTIONS` at the same time: the old file was a legacy 144-D
policy on the pre-0.10 shoulder-roll zero point and the new one is not, so the
flag has to move with the file.

`taitui_right` is bound to keyboard `Y` and to gamepad `RT+D-pad Right`.  The
four `LT+D-pad` slots were already taken, so RT became a second modifier layer
in `GamepadCommandSource`; RT+Down/Up/Left remain free.

Payload variants are **not** in this table.  The default scene has no 2.5 kg
left-hand load, so a payload policy must be run explicitly, e.g.
`--scene unitree_robots/humanoid_ultra/scene_27dof_identified_leftarm2p5kg.xml`:

- taitui-right + 2.5 kg: `ustc_humanoid_ultra_27dof_mimic_taitui_right_2_5kg/2026-08-23_00-40-09_payload_from_17500_lowent/model_24000.pt` (19 falls / 100 on the payload scene)
- houtaitui + 2.5 kg: use the unloaded `model_42000` above -- it scores 9 / 100 on the payload scene, where a run trained with the load scored 25-35.
