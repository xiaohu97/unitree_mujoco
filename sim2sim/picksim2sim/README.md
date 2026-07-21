# Humanoid Ultra Mimic-Pick Sim2Sim

这个目录封装 `USTC-Humanoid-Ultra-27dof-Mimic-Pick` 的策略和 50 Hz 参考动作，
核心仿真仍复用相邻的 `../humanoid_ultra/sim2sim.py`，避免两份控制、观测和关节
映射代码逐渐不一致。

## 目录

```text
picksim2sim/
├── picksim2sim.py
├── README.md
└── assets/
    ├── policy.pt
    ├── ustc1_pick.npz
    ├── agent.yaml
    ├── env.yaml
    └── tracking_env_cfg.py
```

当前策略输入为单帧 144 维、输出 27 维；`ustc1_pick.npz` 为 360 帧、50 Hz，
动作时长约 7.2 秒。

## Walk → Pick → Walk

```bash
cd /home/zxh/ustc_humanoid/unitree_mujoco
conda activate gmr

# 键盘控制：进入 walk 后按 M 播放 Pick
python sim2sim/picksim2sim/picksim2sim.py

# 手柄控制：进入 walk 后按 LT + 十字键下播放 Pick
python sim2sim/picksim2sim/picksim2sim.py --gamepad
```

默认使用以下状态机：

```text
walk → pick_prepare → pick_play → pick_recover → walk
```

`pick_prepare` 用最长 4 秒的固定五次 smoothstep 轨迹移动到 Pick 首帧（到位门槛
提前满足时立即交接），
`pick_recover` 用 4 秒回到 walk-ready 默认姿态。两个阶段都不调用 Pick 策略，
关节目标限制为每秒最多 `1.0 rad`，并限制在训练关节范围内；Pick 播放期目标
限制为每秒最多 `4.0 rad`。只有同时满足以下条件才会交接策略：

- 关节位置 RMS 误差不超过 `0.12 rad`；
- 单关节最大误差不超过 `0.25 rad`；
- 最大关节速度不超过 `0.8 rad/s`；
- 基座高度、根节点和躯干朝向仍在安全范围内。

动作播放完后自动进入 `pick_recover`；播放或 prepare 中按键盘 `P`、手柄 `X`
会提前进入 recover，不会直接把当前状态交给 walk 策略。

因为现有 Pick 策略没有训练 walk↔Pick 过渡，prepare/recover 期间还会临时施加
MuJoCo 专用的 6D 基座稳定力。进入 Pick 时用 0.5 秒撤掉，恢复到默认姿态后启动
walk 并用 1 秒撤掉。它用于可靠查看 Pick 的 Sim2Sim 效果，不能直接复制到实机；
实机仍需要过渡策略、包含过渡段的 Mimic 策略或其他真实可实现的平衡控制器。

如需调慢过渡：

```bash
python sim2sim/picksim2sim/picksim2sim.py \
  --mimic-prepare-time 6 \
  --mimic-recover-time 6 \
  --mimic-transition-target-speed 0.7
```

## 直接查看 Pick

`--direct` 会从 Pick 参考首帧启动，不需要按触发键：

```bash
python sim2sim/picksim2sim/picksim2sim.py --direct
```

无界面完整播放并做兼容性检查：

```bash
python sim2sim/picksim2sim/picksim2sim.py \
  --direct \
  --headless \
  --duration 7.2
```

`--direct` 只用于验证策略本体，会直接从参考状态启动，因此不会经过 prepare/recover。
其他参数，例如 `--mimic-start-frame`、`--elastic-band`、`--duration`，会原样转发
给通用 sim2sim 脚本。

## 注意

这个训练任务是动作 Mimic，当前 MuJoCo 场景不会自动生成被拾取物体。因此这里
检查的是“弯腰/伸手/起身”等 Pick 动作及关节跟踪效果，不代表已经训练了物体接触、
抓取力或抓取成功判定。若要验证真实拾取，需要再增加物体模型、碰撞参数和抓取任务逻辑。
