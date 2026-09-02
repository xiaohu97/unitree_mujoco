#!/usr/bin/env python3
"""
惯性参数辨识数据集生成器
==========================
生成多组不同惯性参数(phi)的仿真数据，用于训练神经网络

方法：
1. 运行时修改MuJoCo模型参数 (mujoco.MjModel)
2. 或者生成多个带有不同参数的MJCF文件

关键惯性参数（每个link共11个）：
- mass: 质量 (1个)
- com: 质心位置 (3个)，在MuJoCo中存储为 inertial pos
- inertia: 惯量矩阵 (6个，对称矩阵的上三角)
- rotor_inertia: 转子惯量 (1个)，在MuJoCo中对应 joint armature

作者: ustczxh
日期: 2025-01-30
"""

import os
import sys
import numpy as np
import mujoco
import copy
import xml.etree.ElementTree as ET
from typing import List, Dict, Tuple, Optional
import json
from pathlib import Path
import shutil


class InertiaParameterVariator:
    """
    惯性参数变化器
    用于在MuJoCo模型中修改惯性参数
    """
    
    def __init__(self, model_path: str):
        """
        参数:
            model_path: MJCF/XML模型文件路径
        """
        self.model_path = model_path
        self.original_model = mujoco.MjModel.from_xml_path(model_path)
        self.original_data = mujoco.MjData(self.original_model)
        
        # 保存原始参数
        self.original_params = self._extract_inertia_params(self.original_model)
        
        print(f"加载模型: {model_path}")
        print(f"  - Body数量: {self.original_model.nbody}")
        print(f"  - 关节数量: {self.original_model.njnt}")
        print(f"  - 执行器数量: {self.original_model.nu}")
        print(f"  - 总质量: {self._get_total_mass(self.original_model):.3f} kg")
        
        self._print_body_info()
    
    def _print_body_info(self):
        """打印各body的惯性信息"""
        print("\n各Body惯性参数:")
        print("-" * 80)
        for i in range(self.original_model.nbody):
            name = mujoco.mj_id2name(self.original_model, mujoco.mjtObj.mjOBJ_BODY, i)
            mass = self.original_model.body_mass[i]
            ipos = self.original_model.body_ipos[i]
            iquat = self.original_model.body_iquat[i]
            inertia = self.original_model.body_inertia[i]
            
            if name and mass > 1e-6:  # 跳过无质量的body
                print(f"  [{i:2d}] {name:30s}: mass={mass:8.4f} kg, "
                      f"ipos=({ipos[0]:7.4f}, {ipos[1]:7.4f}, {ipos[2]:7.4f}), "
                      f"inertia=({inertia[0]:.4e}, {inertia[1]:.4e}, {inertia[2]:.4e})")
        print("-" * 80)
    
    def _get_total_mass(self, model: mujoco.MjModel) -> float:
        """计算总质量"""
        return np.sum(model.body_mass)
    
    def _extract_inertia_params(self, model: mujoco.MjModel) -> Dict:
        """
        从MuJoCo模型中提取惯性参数
        
        返回:
            params: {
                'body_mass': (nbody,),
                'body_ipos': (nbody, 3),      # 相对于body frame的惯性中心位置
                'body_iquat': (nbody, 4),     # 惯性主轴方向
                'body_inertia': (nbody, 3),   # 主惯量 (I_xx, I_yy, I_zz in principal frame)
                'jnt_armature': (njnt,),      # 关节转子惯量
            }
        """
        return {
            'body_mass': model.body_mass.copy(),
            'body_ipos': model.body_ipos.copy(),
            'body_iquat': model.body_iquat.copy(),
            'body_inertia': model.body_inertia.copy(),
            'jnt_armature': model.dof_armature.copy(),
        }
    
    def _apply_inertia_params(self, model: mujoco.MjModel, params: Dict):
        """
        将惯性参数应用到MuJoCo模型
        
        注意：修改后需要调用 mj_setConst 更新派生常量
        """
        model.body_mass[:] = params['body_mass']
        model.body_ipos[:] = params['body_ipos']
        model.body_iquat[:] = params['body_iquat']
        model.body_inertia[:] = params['body_inertia']
        model.dof_armature[:] = params['jnt_armature']
        
        # 更新MuJoCo内部常量
        mujoco.mj_setConst(model, mujoco.MjData(model))
    
    def generate_varied_params(self, 
                               mass_noise_std: float = 0.1,
                               com_noise_std: float = 0.01,
                               inertia_noise_std: float = 0.1,
                               armature_noise_std: float = 0.1,
                               mass_scale_range: Tuple[float, float] = (0.8, 1.2),
                               seed: int = None) -> Dict:
        """
        生成带有噪声的惯性参数
        
        参数:
            mass_noise_std: 质量相对噪声标准差 (例如0.1表示10%)
            com_noise_std: 质心位置绝对噪声标准差 (米)
            inertia_noise_std: 惯量相对噪声标准差
            armature_noise_std: 转子惯量相对噪声标准差
            mass_scale_range: 全局质量缩放范围
            seed: 随机种子
            
        返回:
            varied_params: 变化后的参数字典
        """
        if seed is not None:
            np.random.seed(seed)
        
        params = copy.deepcopy(self.original_params)
        
        # 全局质量缩放因子
        global_mass_scale = np.random.uniform(*mass_scale_range)
        
        for i in range(self.original_model.nbody):
            if params['body_mass'][i] < 1e-6:
                continue  # 跳过无质量的body
            
            # 1. 质量：乘性噪声 + 全局缩放
            mass_noise = 1.0 + np.random.randn() * mass_noise_std
            mass_noise = np.clip(mass_noise, 0.5, 2.0)  # 限制范围
            params['body_mass'][i] *= mass_noise * global_mass_scale
            
            # 2. 质心位置：加性噪声
            params['body_ipos'][i] += np.random.randn(3) * com_noise_std
            
            # 3. 惯量：乘性噪声（保持正定性）
            inertia_noise = 1.0 + np.random.randn(3) * inertia_noise_std
            inertia_noise = np.clip(inertia_noise, 0.5, 2.0)
            params['body_inertia'][i] *= inertia_noise
        
        # 4. 转子惯量：乘性噪声
        for i in range(len(params['jnt_armature'])):
            if params['jnt_armature'][i] > 1e-8:
                armature_noise = 1.0 + np.random.randn() * armature_noise_std
                armature_noise = np.clip(armature_noise, 0.5, 2.0)
                params['jnt_armature'][i] *= armature_noise
        
        return params
    
    def create_varied_model(self, params: Dict) -> Tuple[mujoco.MjModel, mujoco.MjData]:
        """
        创建带有变化参数的模型实例
        
        返回:
            model, data: 新的MuJoCo模型和数据
        """
        # 从原始XML重新加载模型
        model = mujoco.MjModel.from_xml_path(self.model_path)
        data = mujoco.MjData(model)
        
        # 应用变化的参数
        self._apply_inertia_params(model, params)
        
        return model, data
    
    def params_to_phi(self, params: Dict, include_armature: bool = True) -> np.ndarray:
        """
        将参数字典转换为phi向量（用于神经网络训练的标签）
        
        phi格式（每个link 11个参数）:
        [mass, h_x, h_y, h_z, I_xx, I_xy, I_yy, I_xz, I_yz, I_zz, rotor_inertia]
        
        其中 h = mass * com (一阶矩)
        
        注意：MuJoCo中的body_inertia是主惯量(对角元素)，需要转换
        """
        nbody = len(params['body_mass'])
        phi_list = []
        
        for i in range(nbody):
            mass = params['body_mass'][i]
            ipos = params['body_ipos'][i]  # 质心位置
            inertia = params['body_inertia'][i]  # 主惯量 [I_xx, I_yy, I_zz] in principal frame
            iquat = params['body_iquat'][i]  # 主轴方向
            
            # 一阶矩 h = m * c
            h = mass * ipos
            
            # 主惯量 -> 完整惯量矩阵（在principal frame中是对角的）
            # 需要通过iquat旋转到body frame
            # 这里简化处理：假设主轴与body frame对齐
            # 如果iquat不是单位四元数，需要做旋转变换
            I_xx, I_yy, I_zz = inertia
            I_xy, I_xz, I_yz = 0.0, 0.0, 0.0  # 简化：假设对角
            
            # 转子惯量 (简化处理：按body索引取对应关节的armature)
            # 实际上需要根据关节-连杆映射关系
            rotor_inertia = 0.0
            if include_armature and i < len(params['jnt_armature']):
                rotor_inertia = params['jnt_armature'][i] if i > 0 else 0.0
            
            phi_link = [mass, h[0], h[1], h[2], 
                        I_xx, I_xy, I_yy, I_xz, I_yz, I_zz,
                        rotor_inertia]
            phi_list.extend(phi_link)
        
        return np.array(phi_list, dtype=np.float32)


class MJCFVariator:
    """
    MJCF文件变化器
    通过修改XML文件生成不同参数的模型
    """
    
    def __init__(self, template_path: str, output_dir: str):
        """
        参数:
            template_path: 原始MJCF模板文件路径
            output_dir: 输出目录
        """
        self.template_path = template_path
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 解析XML
        self.tree = ET.parse(template_path)
        self.root = self.tree.getroot()
        
        # 提取所有body的惯性信息
        self.bodies = self._find_all_bodies()
        print(f"找到 {len(self.bodies)} 个带惯性参数的body")
    
    def _find_all_bodies(self) -> List[ET.Element]:
        """找到所有带有inertial的body元素"""
        bodies = []
        for body in self.root.iter('body'):
            inertial = body.find('inertial')
            if inertial is not None:
                bodies.append(body)
        return bodies
    
    def _parse_inertial(self, inertial: ET.Element) -> Dict:
        """解析inertial元素的属性"""
        return {
            'pos': np.array([float(x) for x in inertial.get('pos', '0 0 0').split()]),
            'quat': np.array([float(x) for x in inertial.get('quat', '1 0 0 0').split()]),
            'mass': float(inertial.get('mass', '0')),
            'diaginertia': np.array([float(x) for x in inertial.get('diaginertia', '0 0 0').split()]),
        }
    
    def _update_inertial(self, inertial: ET.Element, params: Dict):
        """更新inertial元素的属性"""
        inertial.set('pos', ' '.join(f'{x:.6f}' for x in params['pos']))
        inertial.set('quat', ' '.join(f'{x:.6f}' for x in params['quat']))
        inertial.set('mass', f'{params["mass"]:.6f}')
        inertial.set('diaginertia', ' '.join(f'{x:.6e}' for x in params['diaginertia']))
    
    def generate_varied_xml(self, 
                            variation_id: int,
                            mass_noise_std: float = 0.1,
                            com_noise_std: float = 0.01,
                            inertia_noise_std: float = 0.1,
                            seed: int = None) -> Tuple[str, Dict]:
        """
        生成带有变化参数的MJCF文件
        
        返回:
            output_path: 输出文件路径
            phi_true: 真实的惯性参数向量
        """
        if seed is not None:
            np.random.seed(seed)
        
        # 深拷贝XML树
        tree_copy = copy.deepcopy(self.tree)
        root_copy = tree_copy.getroot()
        
        phi_dict = {}
        
        for body in root_copy.iter('body'):
            inertial = body.find('inertial')
            if inertial is None:
                continue
            
            body_name = body.get('name', 'unknown')
            params = self._parse_inertial(inertial)
            
            # 添加噪声
            mass_noise = 1.0 + np.random.randn() * mass_noise_std
            mass_noise = np.clip(mass_noise, 0.5, 2.0)
            params['mass'] *= mass_noise
            
            params['pos'] += np.random.randn(3) * com_noise_std
            
            inertia_noise = 1.0 + np.random.randn(3) * inertia_noise_std
            inertia_noise = np.clip(inertia_noise, 0.5, 2.0)
            params['diaginertia'] *= inertia_noise
            
            # 更新XML
            self._update_inertial(inertial, params)
            
            # 记录参数
            phi_dict[body_name] = params
        
        # 保存文件
        output_filename = f"model_var{variation_id:04d}.xml"
        output_path = self.output_dir / output_filename
        tree_copy.write(output_path, encoding='unicode')
        
        return str(output_path), phi_dict
    
    def generate_batch(self, num_variations: int, 
                       base_seed: int = 42,
                       **noise_params) -> List[Tuple[str, Dict]]:
        """
        批量生成多个变化的模型文件
        """
        results = []
        for i in range(num_variations):
            seed = base_seed + i
            path, phi = self.generate_varied_xml(i, seed=seed, **noise_params)
            results.append((path, phi))
            print(f"生成模型 {i+1}/{num_variations}: {path}")
        
        return results


class SimulationDataCollector:
    """
    仿真数据收集器
    使用给定的控制器运行仿真并收集数据
    """
    
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data
        
        # 数据缓冲区
        self.q_buffer = []
        self.dq_buffer = []
        self.ddq_buffer = []
        self.tau_buffer = []
        self.time_buffer = []
    
    def reset(self):
        """重置数据缓冲区"""
        self.q_buffer = []
        self.dq_buffer = []
        self.ddq_buffer = []
        self.tau_buffer = []
        self.time_buffer = []
        
        # 重置仿真状态
        mujoco.mj_resetData(self.model, self.data)
    
    def step(self, ctrl: np.ndarray = None):
        """
        执行一步仿真并记录数据
        """
        # 应用控制
        if ctrl is not None:
            self.data.ctrl[:len(ctrl)] = ctrl
        
        # 记录当前状态
        self.q_buffer.append(self.data.qpos.copy())
        self.dq_buffer.append(self.data.qvel.copy())
        self.tau_buffer.append(self.data.ctrl.copy())
        self.time_buffer.append(self.data.time)
        
        # 仿真步进
        mujoco.mj_step(self.model, self.data)
        
        # 计算加速度 (通过差分)
        if len(self.dq_buffer) >= 2:
            dt = self.model.opt.timestep
            ddq = (self.data.qvel - self.dq_buffer[-1]) / dt
            self.ddq_buffer.append(ddq)
        else:
            self.ddq_buffer.append(np.zeros_like(self.data.qvel))
    
    def run_trajectory(self, controller, duration: float, dt: float = 0.001) -> Dict:
        """
        运行一段轨迹并收集数据
        
        参数:
            controller: 控制器函数 ctrl = controller(q, dq, t)
            duration: 仿真时长（秒）
            dt: 仿真时间步长
            
        返回:
            data: 收集的数据字典
        """
        self.reset()
        
        # 设置时间步长
        self.model.opt.timestep = dt
        
        steps = int(duration / dt)
        
        for i in range(steps):
            q = self.data.qpos.copy()
            dq = self.data.qvel.copy()
            t = self.data.time
            
            # 获取控制命令
            ctrl = controller(q, dq, t)
            
            # 执行仿真步骤
            self.step(ctrl)
        
        return self.get_data()
    
    def get_data(self) -> Dict:
        """获取收集的数据"""
        return {
            'q': np.array(self.q_buffer),      # (T, nq)
            'dq': np.array(self.dq_buffer),    # (T, nv)
            'ddq': np.array(self.ddq_buffer),  # (T, nv)
            'tau': np.array(self.tau_buffer),  # (T, nu)
            'time': np.array(self.time_buffer),
        }


def create_exciting_trajectory_controller(model: mujoco.MjModel, 
                                          frequencies: List[float] = None,
                                          amplitudes: List[float] = None):
    """
    创建激励轨迹控制器
    
    使用多频率正弦信号作为位置目标，通过PD控制实现
    这种轨迹可以更好地激励动力学系统，利于参数辨识
    """
    nu = model.nu
    
    if frequencies is None:
        # 默认使用多个频率
        frequencies = [0.1, 0.2, 0.5, 1.0, 2.0]
    
    if amplitudes is None:
        # 默认幅度
        amplitudes = [0.3, 0.2, 0.1, 0.05, 0.02]
    
    # PD控制参数
    kp = np.ones(nu) * 100.0
    kd = np.ones(nu) * 10.0
    
    # 获取默认位置
    default_qpos = np.zeros(nu)
    
    def controller(q, dq, t):
        # 目标位置：多频率正弦叠加
        q_target = default_qpos.copy()
        dq_target = np.zeros(nu)
        
        for freq, amp in zip(frequencies, amplitudes):
            phase = np.random.rand(nu) * 2 * np.pi  # 随机相位
            q_target += amp * np.sin(2 * np.pi * freq * t + phase)
            dq_target += amp * 2 * np.pi * freq * np.cos(2 * np.pi * freq * t + phase)
        
        # 当前关节位置和速度（跳过浮动基座）
        if len(q) > nu:
            q_joint = q[7:7+nu]
            dq_joint = dq[6:6+nu]
        else:
            q_joint = q[:nu]
            dq_joint = dq[:nu]
        
        # PD控制
        tau = kp * (q_target - q_joint) + kd * (dq_target - dq_joint)
        
        return tau
    
    return controller


def save_dataset(data: Dict, phi: np.ndarray, output_path: str):
    """
    保存数据集到文件
    
    格式与你的训练代码兼容
    """
    output_dir = Path(output_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 保存各数据文件 (转置为 num_joints x num_samples 格式)
    np.savetxt(output_dir / 'g1_robot_low_q.dat', data['q'].T, delimiter='\t')
    np.savetxt(output_dir / 'g1_robot_dq.dat', data['dq'].T, delimiter='\t')
    np.savetxt(output_dir / 'g1_robot_ddq.dat', data['ddq'].T, delimiter='\t')
    np.savetxt(output_dir / 'g1_robot_tau.dat', data['tau'].T, delimiter='\t')
    
    # 保存接触状态（如果有）
    if 'contact' in data:
        np.savetxt(output_dir / 'g1_robot_contact.dat', data['contact'].T, delimiter='\t')
    else:
        # 创建假的接触数据
        contact = np.ones((2, len(data['time'])))  # 假设双足接触
        np.savetxt(output_dir / 'g1_robot_contact.dat', contact, delimiter='\t')
    
    # 保存phi (真值)
    np.save(output_dir / 'phi_true.npy', phi)
    
    # 保存元信息
    meta = {
        'num_samples': len(data['time']),
        'dt': data['time'][1] - data['time'][0] if len(data['time']) > 1 else 0.001,
        'duration': data['time'][-1],
        'nq': data['q'].shape[1],
        'nv': data['dq'].shape[1],
        'nu': data['tau'].shape[1],
        'num_phi': len(phi),
    }
    with open(output_dir / 'meta.json', 'w') as f:
        json.dump(meta, f, indent=2)
    
    print(f"数据已保存到: {output_dir}")
    print(f"  - 样本数: {meta['num_samples']}")
    print(f"  - 时长: {meta['duration']:.2f}s")


def main():
    """
    主函数：生成多组不同惯性参数的仿真数据
    """
    import argparse
    parser = argparse.ArgumentParser(description='生成参数辨识训练数据集')
    parser.add_argument('--model', type=str, 
                        default='/home/ustczxh/humanoid/unitree_mujoco/unitree_robots/g1/scene_29dof.xml',
                        help='MuJoCo模型文件路径')
    parser.add_argument('--output', type=str, default='./data/varied_inertia',
                        help='输出目录')
    parser.add_argument('--num-variations', type=int, default=10,
                        help='生成的参数变体数量')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='每次仿真时长（秒）')
    parser.add_argument('--method', type=str, choices=['runtime', 'xml'], default='runtime',
                        help='参数变化方法: runtime(运行时修改) 或 xml(生成新XML文件)')
    parser.add_argument('--mass-noise', type=float, default=0.15,
                        help='质量噪声标准差 (相对值)')
    parser.add_argument('--com-noise', type=float, default=0.01,
                        help='质心位置噪声标准差 (米)')
    parser.add_argument('--inertia-noise', type=float, default=0.15,
                        help='惯量噪声标准差 (相对值)')
    parser.add_argument('--seed', type=int, default=42, help='随机种子')
    
    args = parser.parse_args()
    
    np.random.seed(args.seed)
    
    print("=" * 60)
    print("惯性参数辨识数据集生成器")
    print("=" * 60)
    
    output_base = Path(args.output)
    output_base.mkdir(parents=True, exist_ok=True)
    
    if args.method == 'runtime':
        # 方法1：运行时修改参数
        print("\n使用方法1：运行时修改MuJoCo模型参数")
        
        variator = InertiaParameterVariator(args.model)
        
        for i in range(args.num_variations):
            print(f"\n{'='*60}")
            print(f"生成变体 {i+1}/{args.num_variations}")
            print(f"{'='*60}")
            
            # 生成变化的参数
            varied_params = variator.generate_varied_params(
                mass_noise_std=args.mass_noise,
                com_noise_std=args.com_noise,
                inertia_noise_std=args.inertia_noise,
                seed=args.seed + i
            )
            
            # 创建模型
            model, data = variator.create_varied_model(varied_params)
            
            print(f"  新总质量: {np.sum(model.body_mass):.3f} kg "
                  f"(原始: {variator._get_total_mass(variator.original_model):.3f} kg)")
            
            # 转换为phi向量
            phi = variator.params_to_phi(varied_params)
            print(f"  phi向量长度: {len(phi)}")
            
            # 创建数据收集器
            collector = SimulationDataCollector(model, data)
            
            # 创建激励轨迹控制器
            controller = create_exciting_trajectory_controller(model)
            
            # 运行仿真收集数据
            print(f"  运行仿真 ({args.duration}秒)...")
            sim_data = collector.run_trajectory(controller, duration=args.duration)
            
            # 保存数据
            output_path = output_base / f"var_{i:04d}"
            save_dataset(sim_data, phi, str(output_path))
    
    else:
        # 方法2：生成多个XML文件
        print("\n使用方法2：生成多个带有不同参数的MJCF文件")
        
        # 解析模型路径，找到主模型文件
        model_dir = Path(args.model).parent
        
        variator = MJCFVariator(args.model, str(output_base / 'models'))
        
        results = variator.generate_batch(
            args.num_variations,
            base_seed=args.seed,
            mass_noise_std=args.mass_noise,
            com_noise_std=args.com_noise,
            inertia_noise_std=args.inertia_noise
        )
        
        # 对每个变体运行仿真
        for i, (xml_path, phi_dict) in enumerate(results):
            print(f"\n运行变体 {i+1}/{args.num_variations}: {xml_path}")
            
            try:
                model = mujoco.MjModel.from_xml_path(xml_path)
                data = mujoco.MjData(model)
                
                collector = SimulationDataCollector(model, data)
                controller = create_exciting_trajectory_controller(model)
                
                sim_data = collector.run_trajectory(controller, duration=args.duration)
                
                # 简化phi（从字典转为向量）
                phi = np.concatenate([
                    np.array([p['mass'], *p['pos'], *p['diaginertia'], 0.0])
                    for p in phi_dict.values()
                ])
                
                output_path = output_base / f"var_{i:04d}"
                save_dataset(sim_data, phi, str(output_path))
                
            except Exception as e:
                print(f"  错误: {e}")
                continue
    
    print("\n" + "=" * 60)
    print("数据生成完成!")
    print(f"输出目录: {output_base}")
    print("=" * 60)
    
    # 生成使用说明
    readme = f"""
# 惯性参数辨识数据集

## 数据格式

每个变体目录包含:
- `g1_robot_low_q.dat`: 关节位置 (num_joints x num_samples)
- `g1_robot_dq.dat`: 关节速度 (num_joints x num_samples)
- `g1_robot_ddq.dat`: 关节加速度 (num_joints x num_samples)
- `g1_robot_tau.dat`: 关节力矩 (num_joints x num_samples)
- `g1_robot_contact.dat`: 接触状态 (2 x num_samples)
- `phi_true.npy`: 真实惯性参数向量
- `meta.json`: 元信息

## 参数设置

- 变体数量: {args.num_variations}
- 质量噪声: {args.mass_noise * 100:.1f}%
- 质心噪声: {args.com_noise * 100:.1f} cm
- 惯量噪声: {args.inertia_noise * 100:.1f}%
- 仿真时长: {args.duration}s

## 使用方法

在 nn_identification.py 中:

```python
config = {{
    'sim_data_paths': [
        '{output_base}/var_0000/',
        '{output_base}/var_0001/',
        # ... 添加更多路径
    ],
    # ...
}}
```

注意：使用多组phi对应的数据训练时，每个样本的phi_true应该是该样本对应的phi值，
而不是统一使用URDF的phi_prior。需要修改Dataset类来加载每个变体对应的phi_true.npy。
"""
    
    with open(output_base / 'README.md', 'w') as f:
        f.write(readme)


if __name__ == '__main__':
    main()
