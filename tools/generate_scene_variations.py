#!/usr/bin/env python3
"""
生成带有不同惯性参数的MJCF场景文件
===================================
专门用于配合现有的 unitree_mujoco 仿真和数据收集脚本

使用方法:
1. 运行此脚本生成多个变体场景
2. 手动或使用 run_batch_collection.py 收集每个变体的数据
3. 使用 nn_identification_multi_phi.py 训练网络

作者: ustczxh
日期: 2025-01-30
"""

import os
import sys
import numpy as np
import xml.etree.ElementTree as ET
import copy
import json
import shutil
import argparse
from pathlib import Path
from typing import Dict, List, Tuple


class SceneVariationGenerator:
    """
    场景变体生成器
    生成带有不同惯性参数的MJCF场景文件
    """
    
    def __init__(self, 
                 robot_model_path: str,
                 scene_template_path: str = None,
                 output_dir: str = "./data/varied_scenes"):
        """
        参数:
            robot_model_path: 机器人模型XML路径 (如 g1_29dof.xml)
            scene_template_path: 场景模板XML路径 (如 scene.xml)，如果为None则自动推断
            output_dir: 输出目录
        """
        self.robot_model_path = Path(robot_model_path)
        self.robot_dir = self.robot_model_path.parent
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 确定场景模板
        if scene_template_path:
            self.scene_template_path = Path(scene_template_path)
        else:
            self.scene_template_path = self.robot_dir / "scene.xml"
        
        # 解析机器人模型
        self.robot_tree = ET.parse(str(self.robot_model_path))
        self.robot_root = self.robot_tree.getroot()
        
        # 提取body和joint信息
        self.bodies_info = self._extract_bodies()
        self.joints_info = self._extract_joints()
        
        print(f"机器人模型: {robot_model_path}")
        print(f"场景模板: {self.scene_template_path}")
        print(f"找到 {len(self.bodies_info)} 个带惯性参数的body")
        print(f"找到 {len(self.joints_info)} 个关节")
        print(f"输出目录: {output_dir}")
    
    def _extract_bodies(self) -> List[Dict]:
        """提取所有带惯性参数的body"""
        bodies = []
        for body in self.robot_root.iter('body'):
            inertial = body.find('inertial')
            if inertial is not None:
                info = {
                    'name': body.get('name', 'unknown'),
                    'mass': float(inertial.get('mass', '0')),
                    'pos': np.array([float(x) for x in inertial.get('pos', '0 0 0').split()]),
                    'quat': np.array([float(x) for x in inertial.get('quat', '1 0 0 0').split()]),
                    'diaginertia': np.array([float(x) for x in inertial.get('diaginertia', '0 0 0').split()]),
                }
                bodies.append(info)
        return bodies
    
    def _extract_joints(self) -> List[Dict]:
        """提取所有关节 (包括通过class引用默认值的)"""
        # 首先提取default中的armature值
        default_armatures = {}
        for default_elem in self.robot_root.iter('default'):
            class_name = default_elem.get('class')
            if class_name:
                for joint_default in default_elem.findall('joint'):
                    armature_str = joint_default.get('armature')
                    if armature_str:
                        default_armatures[class_name] = float(armature_str)
        
        joints = []
        for body in self.robot_root.iter('body'):
            for joint in body.findall('joint'):
                joint_class = joint.get('class')
                # 优先使用直接设置的armature，否则使用class默认值
                armature_str = joint.get('armature')
                if armature_str:
                    armature = float(armature_str)
                elif joint_class and joint_class in default_armatures:
                    armature = default_armatures[joint_class]
                else:
                    armature = 0.0
                
                info = {
                    'name': joint.get('name', 'unknown'),
                    'armature': armature,
                    'class': joint_class,
                    'damping': float(joint.get('damping', '0')),
                    'frictionloss': float(joint.get('frictionloss', '0')),
                }
                joints.append(info)
        return joints
    
    def _params_to_phi(self, bodies: List[Dict], joints: List[Dict]) -> np.ndarray:
        """
        将参数转换为phi向量
        
        phi格式 (每个link 11个参数):
        [mass, h_x, h_y, h_z, I_xx, I_xy, I_yy, I_xz, I_yz, I_zz, rotor_inertia]
        """
        phi_list = []
        
        for i, body in enumerate(bodies):
            mass = body['mass']
            pos = body['pos']
            diaginertia = body['diaginertia']
            
            # 一阶矩 h = m * c
            h = mass * pos
            
            # 惯量 (假设对角)
            I_xx, I_yy, I_zz = diaginertia
            I_xy, I_xz, I_yz = 0.0, 0.0, 0.0
            
            # 转子惯量
            rotor = 0.0
            if i < len(joints):
                rotor = joints[i]['armature']
            
            phi_list.extend([mass, h[0], h[1], h[2],
                            I_xx, I_xy, I_yy, I_xz, I_yz, I_zz, rotor])
        
        return np.array(phi_list, dtype=np.float32)
    
    def get_original_phi(self) -> np.ndarray:
        """获取原始惯性参数向量"""
        return self._params_to_phi(self.bodies_info, self.joints_info)
    
    def generate_variation(self,
                           variation_id: int,
                           mass_noise_std: float = 0.15,
                           com_noise_std: float = 0.01,
                           inertia_noise_std: float = 0.15,
                           armature_noise_std: float = 0.2,
                           mass_scale_range: Tuple[float, float] = (0.9, 1.1),
                           seed: int = None) -> Dict:
        """
        生成单个变体
        
        返回包含路径和参数信息的字典
        """
        if seed is not None:
            np.random.seed(seed)
        
        var_dir = self.output_dir / f"var_{variation_id:04d}"
        var_dir.mkdir(parents=True, exist_ok=True)
        
        # 深拷贝机器人模型树
        robot_tree_copy = copy.deepcopy(self.robot_tree)
        robot_root_copy = robot_tree_copy.getroot()
        
        # 全局质量缩放
        global_mass_scale = np.random.uniform(*mass_scale_range)
        
        # 修改 <default> 中的 armature 值 (class 引用的默认值)
        default_armatures = {}
        for default_elem in robot_root_copy.iter('default'):
            class_name = default_elem.get('class')
            if class_name:
                for joint_default in default_elem.findall('joint'):
                    armature_str = joint_default.get('armature')
                    if armature_str:
                        orig_armature = float(armature_str)
                        if orig_armature > 1e-8:
                            armature_noise = 1.0 + np.random.randn() * armature_noise_std
                            armature_noise = np.clip(armature_noise, 0.6, 1.5)
                            new_armature = orig_armature * armature_noise
                            joint_default.set('armature', f'{new_armature:.6f}')
                            default_armatures[class_name] = new_armature
        
        # 修改后的参数记录
        varied_bodies = []
        
        # 修改body惯性参数
        for body in robot_root_copy.iter('body'):
            inertial = body.find('inertial')
            if inertial is None:
                continue
            
            body_name = body.get('name', 'unknown')
            
            # 原始参数
            orig_mass = float(inertial.get('mass', '0'))
            orig_pos = np.array([float(x) for x in inertial.get('pos', '0 0 0').split()])
            orig_quat = np.array([float(x) for x in inertial.get('quat', '1 0 0 0').split()])
            orig_diag = np.array([float(x) for x in inertial.get('diaginertia', '0 0 0').split()])
            
            # 添加噪声
            mass_noise = 1.0 + np.random.randn() * mass_noise_std
            mass_noise = np.clip(mass_noise, 0.6, 1.5)
            new_mass = orig_mass * mass_noise * global_mass_scale
            
            new_pos = orig_pos + np.random.randn(3) * com_noise_std
            
            inertia_noise = 1.0 + np.random.randn(3) * inertia_noise_std
            inertia_noise = np.clip(inertia_noise, 0.6, 1.5)
            new_diag = orig_diag * inertia_noise
            
            # 更新XML
            inertial.set('mass', f'{new_mass:.6f}')
            inertial.set('pos', ' '.join(f'{x:.6f}' for x in new_pos))
            inertial.set('diaginertia', ' '.join(f'{x:.6e}' for x in new_diag))
            
            varied_bodies.append({
                'name': body_name,
                'mass': new_mass,
                'pos': new_pos,
                'quat': orig_quat,
                'diaginertia': new_diag,
            })
        
        # 修改关节参数 (包括直接设置和通过class引用的)
        varied_joints = []
        for body in robot_root_copy.iter('body'):
            for joint in body.findall('joint'):
                joint_name = joint.get('name', 'unknown')
                joint_class = joint.get('class')
                
                # 首先检查关节是否直接设置了armature
                armature_str = joint.get('armature')
                if armature_str:
                    orig_armature = float(armature_str)
                    if orig_armature > 1e-8:
                        armature_noise = 1.0 + np.random.randn() * armature_noise_std
                        armature_noise = np.clip(armature_noise, 0.6, 1.5)
                        new_armature = orig_armature * armature_noise
                        joint.set('armature', f'{new_armature:.6f}')
                        varied_joints.append({
                            'name': joint_name,
                            'armature': new_armature,
                        })
                    else:
                        varied_joints.append({
                            'name': joint_name,
                            'armature': 0.0,
                        })
                # 如果通过class引用，使用已修改的default值
                elif joint_class and joint_class in default_armatures:
                    varied_joints.append({
                        'name': joint_name,
                        'armature': default_armatures[joint_class],
                    })
                else:
                    # 没有armature设置
                    varied_joints.append({
                        'name': joint_name,
                        'armature': 0.0,
                    })
        
        # 修改compiler的meshdir为绝对路径，指向原始mesh文件夹
        # 同时添加 balanceinertia="true" 让MuJoCo自动修正不满足物理约束的惯量
        for compiler in robot_root_copy.iter('compiler'):
            orig_meshdir = compiler.get('meshdir', 'meshes')
            # 构建绝对路径
            abs_meshdir = str((self.robot_dir / orig_meshdir).absolute())
            compiler.set('meshdir', abs_meshdir)
            # 添加balanceinertia使MuJoCo自动修正惯量以满足 A+B>=C 约束
            compiler.set('balanceinertia', 'true')
        
        # 保存修改后的机器人模型
        robot_output = var_dir / f"{self.robot_model_path.stem}_varied.xml"
        robot_tree_copy.write(str(robot_output), encoding='unicode')
        
        # 创建场景文件
        scene_output = var_dir / "scene_varied.xml"
        self._create_scene_file(str(robot_output), str(scene_output))
        
        # 计算phi向量
        phi = self._params_to_phi(varied_bodies, varied_joints)
        
        # 保存phi_true
        np.save(var_dir / "phi_true.npy", phi)
        
        # 计算总质量
        total_mass = sum(b['mass'] for b in varied_bodies)
        original_mass = sum(b['mass'] for b in self.bodies_info)
        
        # 保存参数信息
        params_info = {
            'variation_id': variation_id,
            'seed': seed,
            'total_mass': float(total_mass),
            'original_mass': float(original_mass),
            'mass_scale': float(global_mass_scale),
            'noise_params': {
                'mass_noise_std': mass_noise_std,
                'com_noise_std': com_noise_std,
                'inertia_noise_std': inertia_noise_std,
                'armature_noise_std': armature_noise_std,
            },
            'default_armatures': {k: float(v) for k, v in default_armatures.items()},
            'phi_length': len(phi),
            'num_bodies': len(varied_bodies),
            'num_joints': len(varied_joints),
        }
        
        with open(var_dir / "params_info.json", 'w') as f:
            json.dump(params_info, f, indent=2)
        
        print(f"  变体 {variation_id:04d}: 质量 {total_mass:.2f}kg ({total_mass/original_mass*100:.1f}%), 关节数 {len(varied_joints)}")
        
        return {
            'variation_id': variation_id,
            'var_dir': str(var_dir),
            'robot_xml': str(robot_output),
            'scene_xml': str(scene_output),
            'phi': phi,
            'params_info': params_info,
        }
    
    def _create_scene_file(self, robot_xml_path: str, scene_output_path: str):
        """创建场景文件 - 使用文本替换而非XML解析，以处理非标准XML"""
        robot_xml_path = Path(robot_xml_path)
        
        if self.scene_template_path.exists():
            # 读取场景模板内容
            with open(self.scene_template_path, 'r') as f:
                content = f.read()
            
            # 使用正则表达式替换include路径
            import re
            
            # 找到包含机器人模型的include标签并替换
            # 匹配 <include file="xxx.xml"/> 或 <include file="xxx.xml" />
            robot_stem = self.robot_model_path.stem
            
            # 模式1: 匹配精确的文件名
            pattern1 = rf'(<include\s+file=")[^"]*{robot_stem}\.xml(")'
            replacement1 = rf'\g<1>{robot_xml_path.absolute()}\g<2>'
            content = re.sub(pattern1, replacement1, content)
            
            # 模式2: 如果没有匹配到，尝试匹配第一个include
            if robot_stem not in content:
                pattern2 = r'(<include\s+file=")([^"]+\.xml)(")'
                def replace_first(m):
                    return f'{m.group(1)}{robot_xml_path.absolute()}{m.group(3)}'
                content = re.sub(pattern2, replace_first, content, count=1)
            
            with open(scene_output_path, 'w') as f:
                f.write(content)
        else:
            # 创建简单的场景文件
            scene_content = f'''<mujoco model="varied_scene">
  <include file="{robot_xml_path.absolute()}"/>
  
  <statistic center="0 0 0.5" extent="2.0"/>
  
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
  </visual>
  
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane"/>
  </worldbody>
</mujoco>
'''
            with open(scene_output_path, 'w') as f:
                f.write(scene_content)
    
    def generate_batch(self, 
                       num_variations: int,
                       base_seed: int = 42,
                       **noise_params) -> List[Dict]:
        """
        批量生成变体
        """
        print(f"\n{'='*60}")
        print(f"开始生成 {num_variations} 个变体")
        print(f"{'='*60}")
        
        results = []
        
        for i in range(num_variations):
            result = self.generate_variation(
                variation_id=i,
                seed=base_seed + i,
                **noise_params
            )
            results.append(result)
        
        # 保存原始phi
        original_phi = self.get_original_phi()
        np.save(self.output_dir / "phi_original.npy", original_phi)
        
        # 保存摘要
        summary = {
            'num_variations': num_variations,
            'base_seed': base_seed,
            'original_phi_length': len(original_phi),
            'original_total_mass': sum(b['mass'] for b in self.bodies_info),
            'noise_params': noise_params,
            'robot_model': str(self.robot_model_path),
            'variations': [r['params_info'] for r in results],
        }
        
        with open(self.output_dir / "summary.json", 'w') as f:
            json.dump(summary, f, indent=2)
        
        # 打印使用说明
        self._print_usage()
        
        return results
    
    def _print_usage(self):
        """打印使用说明"""
        print(f"""
{'='*60}
生成完成！
{'='*60}

输出目录: {self.output_dir}

目录结构:
{self.output_dir}/
├── phi_original.npy      # 原始惯性参数
├── summary.json          # 摘要信息
├── var_0000/
│   ├── *_varied.xml      # 修改后的机器人模型
│   ├── scene_varied.xml  # 场景文件
│   ├── phi_true.npy      # 该变体的惯性参数
│   └── params_info.json  # 参数信息
├── var_0001/
│   └── ...
└── ...

使用方法:
---------

方法1: 手动收集数据
~~~~~~~~~~~~~~~~~~~
对于每个变体:

1. 修改 simulate/config.yaml:
   robot_scene: "{self.output_dir}/var_XXXX/scene_varied.xml"

2. 启动仿真:
   cd simulate/build && ./unitree_mujoco

3. 启动你的控制程序

4. 启动数据收集:
   python read_sim_g1_data_logger.py var_XXXX

5. 转换CSV为DAT格式并移动到 var_XXXX 目录

方法2: 自动批量收集
~~~~~~~~~~~~~~~~~~~
python tools/run_batch_collection.py \\
    --data-dir {self.output_dir} \\
    --collector your_collector_script.py \\
    --duration 30

训练网络:
---------
python tools/nn_identification_multi_phi.py \\
    --data-dir {self.output_dir} \\
    --epochs 100
""")


def main():
    parser = argparse.ArgumentParser(
        description='生成带有不同惯性参数的MJCF场景文件',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python generate_scene_variations.py --model ../unitree_robots/g1/g1_29dof.xml --num 20
  python generate_scene_variations.py --model ../unitree_robots/g1/g1_29dof.xml --output ./data/my_dataset --mass-noise 0.2
"""
    )
    
    parser.add_argument('--model', type=str, required=True,
                        help='机器人模型XML路径 (如 g1_29dof.xml)')
    parser.add_argument('--scene', type=str, default=None,
                        help='场景模板XML路径 (可选，默认自动推断)')
    parser.add_argument('--output', type=str, default='./data/varied_scenes',
                        help='输出目录')
    parser.add_argument('--num', type=int, default=10,
                        help='变体数量')
    parser.add_argument('--mass-noise', type=float, default=0.15,
                        help='质量噪声标准差 (相对值，如0.15表示15%%)')
    parser.add_argument('--com-noise', type=float, default=0.01,
                        help='质心位置噪声标准差 (米)')
    parser.add_argument('--inertia-noise', type=float, default=0.15,
                        help='惯量噪声标准差 (相对值)')
    parser.add_argument('--armature-noise', type=float, default=0.2,
                        help='转子惯量噪声标准差 (相对值)')
    parser.add_argument('--seed', type=int, default=42,
                        help='随机种子')
    
    args = parser.parse_args()
    
    # 检查文件存在
    if not Path(args.model).exists():
        print(f"错误: 模型文件不存在: {args.model}")
        sys.exit(1)
    
    generator = SceneVariationGenerator(
        robot_model_path=args.model,
        scene_template_path=args.scene,
        output_dir=args.output
    )
    
    generator.generate_batch(
        num_variations=args.num,
        base_seed=args.seed,
        mass_noise_std=args.mass_noise,
        com_noise_std=args.com_noise,
        inertia_noise_std=args.inertia_noise,
        armature_noise_std=args.armature_noise,
    )


if __name__ == '__main__':
    main()
