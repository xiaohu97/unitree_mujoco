"""HeightMapSampler 离线测试：只要 mujoco + numpy（gmr 环境即可）。

    /home/zxh/miniconda3/envs/gmr/bin/python simulate_python/test/test_height_map_sampler.py
"""
from __future__ import annotations

import math
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from height_map_publisher import HeightMapSampler, build_height_map_message  # noqa: E402

# 地面 + 前方一块 0.15 m 高的台阶（射线正好打在箱体边界上会判未命中，所以箱体比格子范围宽 5 cm） + 一个带自身几何体的"机器人"（自由基座，躯干下挂一块板，射线必须穿过它）
XML = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 0.05"/>
    <geom name="step" type="box" pos="0.6 0 0.075" size="0.25 0.65 0.075"/>  <!-- x 0.35..0.85, y -0.65..0.65：格子不压边 -->
    <body name="pelvis" pos="0 0 0.75">
      <freejoint/>
      <geom type="sphere" size="0.05" mass="1"/>
      <body name="torso_link" pos="0 0 0.05">
        <geom type="box" size="0.3 0.3 0.01" pos="0 0 -0.3" contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def require(ok: bool, message: str) -> None:
    if not ok:
        raise AssertionError(message)


def main() -> None:
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    sampler = HeightMapSampler(model)
    require(sampler.num_cells == 187, "policy grid is 17x11")
    torso_z = float(data.xpos[sampler.body_id][2])
    require(abs(torso_z - 0.8) < 1e-6, "torso height")

    def on_step(world_x: float, world_y: float) -> bool:
        return 0.35 <= world_x <= 0.85 and -0.65 <= world_y <= 0.65

    cells = sampler.sample(data)
    require(cells.shape == (187,) and np.all(np.isfinite(cells)), "all rays hit the floor/step")
    # 索引约定：data[iy * width + ix]，x = -0.8 + 0.1 ix，y = -0.5 + 0.1 iy
    for iy in range(11):
        for ix in range(17):
            x, y = -0.8 + 0.1 * ix, -0.5 + 0.1 * iy
            expected = (0.15 if on_step(x, y) else 0.0) - torso_z
            got = cells[iy * 17 + ix]
            require(abs(got - expected) < 1e-4, f"cell ix={ix} iy={iy}: {got} != {expected} (x={x:.1f})")
    # 躯干正下方那块自身几何体没有挡住射线
    require(abs(cells[5 * 17 + 8] - (-torso_z)) < 1e-4, "self geometry must be skipped")

    # 转 90° 朝 +y 后，台阶（世界 +x）出现在机器人右侧（body y < 0）：world = (-y_body, x_body)
    yaw = math.pi / 2
    data.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    mujoco.mj_forward(model, data)
    cells = sampler.sample(data)
    on_step_count = 0
    for iy in range(11):
        for ix in range(17):
            x, y = -0.8 + 0.1 * ix, -0.5 + 0.1 * iy
            world_x, world_y = -y, x
            expected = (0.15 if on_step(world_x, world_y) else 0.0) - torso_z
            on_step_count += expected > -torso_z
            got = cells[iy * 17 + ix]
            require(abs(got - expected) < 1e-4, f"yaw-aligned cell ix={ix} iy={iy}: {got} != {expected}")
    require(on_step_count > 0, "yaw test must cover some step cells")

    # 走出地面范围（平面无限大，用一个坑模拟未命中）：把机器人抬到台阶上方看数值随躯干高度变化
    data.qpos[0:3] = [0.6, 0, 1.75]
    data.qpos[3:7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    cells = sampler.sample(data)
    require(abs(cells[5 * 17 + 8] - (0.15 - 1.8)) < 1e-4, "cell under torso on the step")

    msg = build_height_map_message(Msg, 1.5, sampler, cells)
    require(msg.width == 17 and msg.height == 11 and abs(msg.resolution - 0.1) < 1e-9, "message dims")
    require(msg.origin == [-0.8, -0.5] and len(msg.data) == 187 and msg.frame_id == "torso_yaw", "message layout")
    print("PASS height map sampler: index order, yaw alignment, self-hit skipping, message layout")


if __name__ == "__main__":
    main()
