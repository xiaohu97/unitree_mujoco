#!/usr/bin/env python3
"""
批量数据收集脚本 (集成版)
==========================
配合 generate_varied_inertia_data.py 生成的变体模型，
使用现有的 unitree_sdk2py 数据收集脚本收集仿真数据

工作流程：
1. 读取变体目录列表
2. 对每个变体：
   - 修改 config.yaml 指向该变体的 scene 文件
   - 启动 unitree_mujoco 仿真
   - 启动数据收集脚本
   - 等待指定时长
   - 停止仿真和数据收集
   - 将收集的数据移动到变体目录

作者: ustczxh
日期: 2025-01-30
"""

import os
import sys
import time
import subprocess
import signal
import shutil
import yaml
import glob
import argparse
from pathlib import Path
from typing import Optional
import pandas as pd
import numpy as np


class BatchDataCollector:
    """批量数据收集器"""
    
    def __init__(self, 
                 project_dir: str,
                 data_dir: str,
                 collector_script: str = None,
                 robot_name: str = "g1"):
        """
        参数:
            project_dir: unitree_mujoco项目目录
            data_dir: 变体数据目录 (包含 var_0000, var_0001, ...)
            collector_script: 数据收集脚本路径
            robot_name: 机器人名称
        """
        self.project_dir = Path(project_dir)
        self.data_dir = Path(data_dir)
        self.robot_name = robot_name
        
        # 路径设置
        self.sim_executable = self.project_dir / "simulate" / "build" / "unitree_mujoco"
        self.config_path = self.project_dir / "simulate" / "config.yaml"
        self.collector_script = Path(collector_script) if collector_script else None
        
        # 子进程
        self.sim_process = None
        self.collector_process = None
        
        # 备份原始配置
        self._backup_config()
    
    def _backup_config(self):
        """备份原始配置文件"""
        backup_path = self.config_path.with_suffix('.yaml.backup_batch')
        if not backup_path.exists():
            shutil.copy(self.config_path, backup_path)
            print(f"已备份配置文件到: {backup_path}")
    
    def _restore_config(self):
        """恢复原始配置文件"""
        backup_path = self.config_path.with_suffix('.yaml.backup_batch')
        if backup_path.exists():
            shutil.copy(backup_path, self.config_path)
            print("已恢复原始配置文件")
    
    def _update_config(self, scene_path: str):
        """更新config.yaml以使用指定的场景文件"""
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # 更新robot_scene
        config['robot_scene'] = str(Path(scene_path).absolute())
        
        with open(self.config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
        
        print(f"已更新配置文件，使用场景: {scene_path}")
    
    def _start_simulation(self) -> subprocess.Popen:
        """启动MuJoCo仿真"""
        sim_dir = self.sim_executable.parent
        
        process = subprocess.Popen(
            [str(self.sim_executable)],
            cwd=str(sim_dir),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # 创建新的进程组
        )
        
        print(f"已启动仿真进程 (PID: {process.pid})")
        return process
    
    def _start_collector(self, output_name: str, working_dir: str) -> subprocess.Popen:
        """启动数据收集脚本"""
        if self.collector_script is None or not self.collector_script.exists():
            print("警告：未指定数据收集脚本或脚本不存在")
            return None
        
        process = subprocess.Popen(
            [sys.executable, str(self.collector_script), output_name],
            cwd=working_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        
        print(f"已启动数据收集进程 (PID: {process.pid})")
        return process
    
    def _stop_process(self, process: subprocess.Popen, name: str = "process"):
        """停止进程"""
        if process is None:
            return
        
        try:
            # 发送SIGTERM到进程组
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
            print(f"已停止{name} (PID: {process.pid})")
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            print(f"已强制停止{name} (PID: {process.pid})")
        except Exception as e:
            print(f"停止{name}时出错: {e}")
    
    def _find_latest_csv(self, working_dir: str, prefix: str) -> Optional[str]:
        """查找最新的CSV文件"""
        pattern = f"{working_dir}/{prefix}*.csv"
        files = glob.glob(pattern)
        if not files:
            return None
        return max(files, key=os.path.getctime)
    
    def _convert_csv_to_dat(self, csv_path: str, output_dir: str):
        """将CSV转换为DAT格式"""
        try:
            df = pd.read_csv(csv_path)
            
            # 定义列映射
            datasets = self._get_column_mappings()
            
            for file_name, cols in datasets:
                # 检查列是否存在
                available_cols = [c for c in cols if c in df.columns]
                if len(available_cols) < len(cols):
                    missing = set(cols) - set(available_cols)
                    print(f"  警告: {file_name} 缺少列: {missing}")
                    if not available_cols:
                        continue
                
                data = df[available_cols].to_numpy().T
                
                # 处理NaN和Inf
                data = np.nan_to_num(data, nan=0.0, posinf=0.0, neginf=0.0)
                
                output_path = Path(output_dir) / file_name
                np.savetxt(output_path, data, delimiter='\t', fmt='%.6f')
                print(f"  已保存: {file_name} ({data.shape})")
            
            return True
            
        except Exception as e:
            print(f"  转换CSV时出错: {e}")
            return False
    
    def _get_column_mappings(self):
        """获取CSV列到DAT文件的映射"""
        # 基于你的数据收集脚本定义
        low_q_cols = [
            'odom_position_x', 'odom_position_y', 'odom_position_z',
            'low_imu_quat_x', 'low_imu_quat_y', 'low_imu_quat_z', 'low_imu_quat_w'
        ] + [f'low_motor_{i}_q' for i in range(12)]
        
        dq_cols = [
            'odom_velocity_x', 'odom_velocity_y', 'odom_velocity_z',
            'low_imu_gyro_x', 'low_imu_gyro_y', 'low_imu_gyro_z'
        ] + [f'low_motor_{i}_dq' for i in range(12)]
        
        ddq_cols = [
            'low_imu_accel_x', 'low_imu_accel_y', 'low_imu_accel_z'
        ] + [f'low_motor_{i}_ddq' for i in range(12)]
        
        tau_cols = [f'low_motor_{i}_tau_est' for i in range(12)]
        
        contact_cols = ['odom_foot_force_1', 'odom_foot_force_2']
        
        return [
            (f'{self.robot_name}_robot_low_q.dat', low_q_cols),
            (f'{self.robot_name}_robot_dq.dat', dq_cols),
            (f'{self.robot_name}_robot_ddq.dat', ddq_cols),
            (f'{self.robot_name}_robot_tau.dat', tau_cols),
            (f'{self.robot_name}_robot_contact.dat', contact_cols),
        ]
    
    def collect_single_variant(self, 
                                variant_dir: str,
                                duration: float = 30.0,
                                warmup_time: float = 3.0):
        """
        收集单个变体的数据
        
        参数:
            variant_dir: 变体目录路径
            duration: 数据收集时长（秒）
            warmup_time: 仿真预热时间（秒）
        """
        variant_dir = Path(variant_dir)
        variant_name = variant_dir.name
        
        print(f"\n{'='*60}")
        print(f"收集变体: {variant_name}")
        print(f"{'='*60}")
        
        # 检查scene文件
        scene_path = variant_dir / "scene_varied.xml"
        if not scene_path.exists():
            print(f"错误：场景文件不存在: {scene_path}")
            return False
        
        try:
            # 1. 更新配置文件
            self._update_config(str(scene_path))
            
            # 2. 启动仿真
            self.sim_process = self._start_simulation()
            
            # 3. 等待仿真启动
            print(f"等待仿真启动 ({warmup_time}秒)...")
            time.sleep(warmup_time)
            
            # 4. 启动数据收集
            if self.collector_script:
                self.collector_process = self._start_collector(
                    variant_name, 
                    str(variant_dir)
                )
            
            # 5. 等待数据收集
            print(f"收集数据中 ({duration}秒)...")
            time.sleep(duration)
            
            # 6. 停止进程
            self._stop_process(self.collector_process, "数据收集")
            self._stop_process(self.sim_process, "仿真")
            
            # 7. 处理收集的数据
            if self.collector_script:
                csv_file = self._find_latest_csv(str(variant_dir), variant_name)
                if csv_file:
                    print(f"处理CSV文件: {csv_file}")
                    self._convert_csv_to_dat(csv_file, str(variant_dir))
                else:
                    print("警告：未找到CSV文件")
            
            return True
            
        except Exception as e:
            print(f"收集数据时出错: {e}")
            return False
        
        finally:
            # 确保进程被停止
            self._stop_process(self.collector_process, "数据收集")
            self._stop_process(self.sim_process, "仿真")
    
    def collect_all_variants(self, 
                              duration: float = 30.0,
                              warmup_time: float = 3.0,
                              start_from: int = 0):
        """
        收集所有变体的数据
        
        参数:
            duration: 每个变体的数据收集时长
            warmup_time: 仿真预热时间
            start_from: 从哪个变体开始（用于断点续传）
        """
        # 查找所有变体目录
        variant_dirs = sorted([
            d for d in self.data_dir.iterdir() 
            if d.is_dir() and d.name.startswith('var_')
        ])
        
        if not variant_dirs:
            print(f"错误：在 {self.data_dir} 中没有找到变体目录")
            return
        
        print(f"找到 {len(variant_dirs)} 个变体目录")
        
        try:
            for i, variant_dir in enumerate(variant_dirs):
                if i < start_from:
                    print(f"跳过变体 {i}: {variant_dir.name}")
                    continue
                
                print(f"\n[{i+1}/{len(variant_dirs)}]")
                success = self.collect_single_variant(
                    str(variant_dir),
                    duration=duration,
                    warmup_time=warmup_time
                )
                
                if not success:
                    print(f"警告：变体 {variant_dir.name} 收集失败")
                
                # 短暂等待，确保资源释放
                time.sleep(2)
        
        finally:
            # 恢复配置文件
            self._restore_config()
        
        print(f"\n{'='*60}")
        print("所有变体数据收集完成！")
        print(f"{'='*60}")


def create_simple_collector_script(output_path: str):
    """创建简化版的数据收集脚本"""
    script_content = '''#!/usr/bin/env python3
"""
简化版数据收集脚本
用于收集仿真数据
"""

import time
import csv
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

class SimpleDataLogger:
    def __init__(self, base_name, record_duration=30):
        self.base_name = base_name
        self.record_duration = record_duration
        self.csv_file = None
        self.csv_writer = None
        self.start_time = None
        self.data_buffer = []
        self.running = True
        self.open_csv_file()

    def open_csv_file(self):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"{self.base_name}_{timestamp}.csv"
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # 写入表头
        columns = ['timestamp', 'odom_stamp_sec', 'odom_stamp_nanosec']
        columns += ['odom_position_x', 'odom_position_y', 'odom_position_z']
        columns += ['odom_velocity_x', 'odom_velocity_y', 'odom_velocity_z']
        columns += ['odom_imu_quaternion_w', 'odom_imu_quaternion_x', 
                   'odom_imu_quaternion_y', 'odom_imu_quaternion_z']
        columns += ['odom_foot_force_1', 'odom_foot_force_2']
        columns += ['low_tick']
        columns += ['low_imu_quat_w', 'low_imu_quat_x', 'low_imu_quat_y', 'low_imu_quat_z']
        columns += ['low_imu_gyro_x', 'low_imu_gyro_y', 'low_imu_gyro_z']
        columns += ['low_imu_accel_x', 'low_imu_accel_y', 'low_imu_accel_z']
        
        # 电机数据
        for i in range(12):
            columns += [f'low_motor_{i}_q', f'low_motor_{i}_dq', 
                       f'low_motor_{i}_ddq', f'low_motor_{i}_tau_est']
        
        self.csv_writer.writerow(columns)
        self.start_time = time.time()
        print(f"开始记录数据到: {filename}")

    def low_callback(self, msg):
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed >= self.record_duration:
            self.running = False
            return
        
        try:
            row = [current_time, 0, 0]  # timestamp, stamp
            row += [0, 0, 0]  # position (from odom)
            row += [0, 0, 0]  # velocity (from odom)
            row += [msg.imu_state.quaternion[0], msg.imu_state.quaternion[1],
                   msg.imu_state.quaternion[2], msg.imu_state.quaternion[3]]
            row += [0, 0]  # foot force
            row += [msg.tick]
            row += [msg.imu_state.quaternion[0], msg.imu_state.quaternion[1],
                   msg.imu_state.quaternion[2], msg.imu_state.quaternion[3]]
            row += [msg.imu_state.gyroscope[0], msg.imu_state.gyroscope[1], 
                   msg.imu_state.gyroscope[2]]
            row += [msg.imu_state.accelerometer[0], msg.imu_state.accelerometer[1], 
                   msg.imu_state.accelerometer[2]]
            
            for i in range(12):
                if i < len(msg.motor_state):
                    m = msg.motor_state[i]
                    row += [m.q, m.dq, m.ddq, m.tau_est]
                else:
                    row += [0, 0, 0, 0]
            
            self.csv_writer.writerow(row)
            
            if int(elapsed * 10) % 50 == 0:
                print(f"\\r已记录 {elapsed:.1f}/{self.record_duration}s", end='', flush=True)
                self.csv_file.flush()
                
        except Exception as e:
            print(f"处理消息时出错: {e}")

    def run(self):
        ChannelFactoryInitialize(0, "lo")
        low_sub = ChannelSubscriber("rt/lowstate", LowState_)
        low_sub.Init(self.low_callback, 10)
        
        print("开始订阅话题...")
        while self.running:
            time.sleep(0.1)
        
        print("\\n数据收集完成")
        self.csv_file.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python simple_collector.py <base_name> [duration]")
        sys.exit(1)
    
    base_name = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 30
    
    logger = SimpleDataLogger(base_name, duration)
    logger.run()
'''
    
    with open(output_path, 'w') as f:
        f.write(script_content)
    
    print(f"已创建简化版数据收集脚本: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='批量收集变体数据')
    parser.add_argument('--project-dir', type=str,
                        default='/home/ustczxh/humanoid/unitree_mujoco',
                        help='unitree_mujoco项目目录')
    parser.add_argument('--data-dir', type=str, required=True,
                        help='变体数据目录')
    parser.add_argument('--collector', type=str, default=None,
                        help='数据收集脚本路径')
    parser.add_argument('--duration', type=float, default=30.0,
                        help='每个变体的数据收集时长（秒）')
    parser.add_argument('--warmup', type=float, default=3.0,
                        help='仿真预热时间（秒）')
    parser.add_argument('--start-from', type=int, default=0,
                        help='从第几个变体开始（断点续传）')
    parser.add_argument('--single', type=str, default=None,
                        help='只收集单个变体（目录名）')
    parser.add_argument('--create-collector', action='store_true',
                        help='创建简化版数据收集脚本')
    
    args = parser.parse_args()
    
    # 创建收集器脚本
    if args.create_collector:
        collector_path = Path(args.data_dir) / "simple_collector.py"
        create_simple_collector_script(str(collector_path))
        return
    
    collector = BatchDataCollector(
        project_dir=args.project_dir,
        data_dir=args.data_dir,
        collector_script=args.collector,
        robot_name="g1"
    )
    
    if args.single:
        # 收集单个变体
        variant_path = Path(args.data_dir) / args.single
        if variant_path.exists():
            collector.collect_single_variant(
                str(variant_path),
                duration=args.duration,
                warmup_time=args.warmup
            )
        else:
            print(f"错误：变体目录不存在: {variant_path}")
    else:
        # 收集所有变体
        collector.collect_all_variants(
            duration=args.duration,
            warmup_time=args.warmup,
            start_from=args.start_from
        )


if __name__ == '__main__':
    main()
