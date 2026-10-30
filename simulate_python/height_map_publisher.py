"""MuJoCo 里复刻训练侧的高程扫描，并通过 Unitree DDS 发布给感知版 walk 控制器（sim2sim）。

训练（unitree_rl_lab ``perception_cfg.HEIGHT_SCANNER_CFG``）：挂在 ``torso_link`` 上的 RayCaster，
躯干 yaw 对齐的 17×11 网格（x −0.8..+0.8 m，y −0.5..+0.5 m，0.1 m），从上往下打射线到地形。
这里用 ``mujoco.mj_ray`` 做同样的事，只打静态地形（命中机器人自身时继续往下打）。

消息约定和控制器侧一致（unitree_rl_lab ``deploy/robots/g1_29dof/include/height_map_gate.h``）：
``unitree_go::msg::dds_::HeightMap_``，``data[iy * width + ix]`` = 地面高度 − 躯干高度 [m]，
x 变化最快，未命中填 NaN，话题默认 ``rt/perceptive/height_map``。

``HeightMapSampler`` 只依赖 mujoco + numpy，``HeightMapPublisher`` 才引入 unitree_sdk2py。
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

DEFAULT_TOPIC = "rt/perceptive/height_map"


class HeightMapSampler:
    """躯干 yaw 系网格射线采样。``sample()`` 返回 (width*height,) 的 z_ground − z_torso。"""

    def __init__(
        self,
        model: mujoco.MjModel,
        body_name: str = "torso_link",
        width: int = 17,
        height: int = 11,
        resolution: float = 0.1,
        origin: tuple[float, float] = (-0.8, -0.5),
        ray_start_height: float = 2.0,
        max_self_hits: int = 64,  # 穿过整条腿（每个几何体进/出各一次命中）要 20 次以上
    ) -> None:
        self.model = model
        self.body_id = model.body(body_name).id
        self.root_id = int(model.body_rootid[self.body_id])
        self.width, self.height = int(width), int(height)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.ray_start_height = float(ray_start_height)
        self.max_self_hits = int(max_self_hits)
        xs = self.origin[0] + self.resolution * np.arange(self.width)
        ys = self.origin[1] + self.resolution * np.arange(self.height)
        grid_x, grid_y = np.meshgrid(xs, ys, indexing="xy")  # (height, width): x 变化最快
        self._local_xy = np.stack([grid_x.ravel(), grid_y.ravel()], axis=1)
        self._geomid = np.zeros(1, dtype=np.int32)
        self._down = np.array([0.0, 0.0, -1.0])

    @property
    def num_cells(self) -> int:
        return self.width * self.height

    @staticmethod
    def yaw_from_quat_wxyz(q: np.ndarray) -> float:
        w, x, y, z = (float(v) for v in q)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def torso_pose(self, data: mujoco.MjData) -> tuple[np.ndarray, float]:
        return np.array(data.xpos[self.body_id]), self.yaw_from_quat_wxyz(data.xquat[self.body_id])

    def _ground_height(self, data: mujoco.MjData, x: float, y: float, z_start: float) -> float:
        """从 (x, y, z_start) 向下打射线；跳过机器人自身的几何体。命不中返回 NaN。"""
        pnt = np.array([x, y, z_start])
        for _ in range(self.max_self_hits):
            dist = mujoco.mj_ray(self.model, data, pnt, self._down, None, 1, -1, self._geomid)
            if dist < 0:
                return math.nan
            hit_body = int(self.model.geom_bodyid[self._geomid[0]])
            if int(self.model.body_rootid[hit_body]) != self.root_id:
                return float(pnt[2] - dist)
            pnt[2] -= dist + 1e-4  # 打到自己了，从命中点稍下方继续
        return math.nan

    def sample(self, data: mujoco.MjData) -> np.ndarray:
        pos, yaw = self.torso_pose(data)
        c, s = math.cos(yaw), math.sin(yaw)
        world_xy = pos[:2] + self._local_xy @ np.array([[c, s], [-s, c]])
        z_start = float(pos[2]) + self.ray_start_height
        cells = np.empty(self.num_cells, dtype=np.float32)
        for i, (x, y) in enumerate(world_xy):
            cells[i] = self._ground_height(data, float(x), float(y), z_start) - float(pos[2])
        return cells


def build_height_map_message(message_type, stamp: float, sampler: HeightMapSampler, cells: np.ndarray, frame_id: str = "torso_yaw"):
    """按控制器约定填一条 HeightMap_（message_type 传 unitree_sdk2py 的 HeightMap_ 类，便于离线测试）。"""
    if cells.shape != (sampler.num_cells,):
        raise ValueError(f"expected {sampler.num_cells} cells, got {cells.shape}")
    return message_type(
        stamp=float(stamp),
        frame_id=frame_id,
        resolution=sampler.resolution,
        width=sampler.width,
        height=sampler.height,
        origin=[sampler.origin[0], sampler.origin[1]],
        data=[float(v) for v in cells],
    )


class HeightMapPublisher:
    """采样 + DDS 发布。必须在 ``ChannelFactoryInitialize`` 之后构造。"""

    def __init__(self, model: mujoco.MjModel, topic: str = DEFAULT_TOPIC, **sampler_kwargs) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_

        self.message_type = HeightMap_
        self.sampler = HeightMapSampler(model, **sampler_kwargs)
        self.publisher = ChannelPublisher(topic, HeightMap_)
        self.publisher.Init()
        self.topic = topic

    def publish(self, data: mujoco.MjData, stamp: float) -> np.ndarray:
        cells = self.sampler.sample(data)
        self.publisher.Write(build_height_map_message(self.message_type, stamp, self.sampler, cells))
        return cells
