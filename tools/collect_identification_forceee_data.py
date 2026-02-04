#!/usr/bin/env python3
"""
G1机器人惯性参数辨识数据采集（含末端接触力）
==============================================
基于 collect_identification_data.py 扩展：
- 临时旧字段布局下，将脚底六维力写入 g1_robot_ee_force.dat

使用方法:
    python collect_identification_forceee_data.py <output_dir> [--duration 30] [--domain-id 0]
"""

import sys
import time
import csv
import argparse
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

# Unitree SDK2 导入
try:
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
except ImportError:
    print("错误: 需要安装 unitree_sdk2py")
    print("请运行: pip install unitree_sdk2py")
    sys.exit(1)


class G1ForceEEDataCollector:
    """G1机器人数据采集器（含末端接触力）"""

    def __init__(self, output_dir: str, duration: float = 30.0,
                 domain_id: int = 0, interface: str = "lo",
                 motor_count: int = 35):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.duration = duration
        self.domain_id = domain_id
        self.interface = interface
        self.motor_count = motor_count

        # 数据缓存
        self.odom_data = None
        self.low_data = None

        # CSV文件
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None

        # 控制标志
        self.running = False
        self.start_time = None

    def _get_csv_columns(self):
        """获取CSV列名"""
        columns = [
            'timestamp',
            'odom_stamp_sec', 'odom_stamp_nanosec',
            'odom_mode',
            'odom_imu_quaternion_w', 'odom_imu_quaternion_x',
            'odom_imu_quaternion_y', 'odom_imu_quaternion_z',
            'odom_imu_angular_velocity_x', 'odom_imu_angular_velocity_y',
            'odom_imu_angular_velocity_z',
            'odom_imu_linear_acceleration_x', 'odom_imu_linear_acceleration_y',
            'odom_imu_linear_acceleration_z',
            'odom_imu_temperature',
            'odom_gait_type',
            'odom_position_mode',
            'odom_velocity_mode',
            'odom_yaw',
            'odom_position_x', 'odom_position_y', 'odom_position_z',
            'odom_yaw_speed',
            'odom_velocity_x', 'odom_velocity_y', 'odom_velocity_z',
            'odom_angular_speed',
            'odom_foot_position_1', 'odom_foot_position_2',
            'odom_foot_position_3', 'odom_foot_position_4',
            'odom_foot_contact_1', 'odom_foot_contact_2',
            'odom_foot_contact_3', 'odom_foot_contact_4',
            'odom_foot_force_1', 'odom_foot_force_2', 'odom_foot_force_3',
            'odom_foot_force_4', 'odom_foot_force_5', 'odom_foot_force_6',
            'odom_foot_force_7', 'odom_foot_force_8', 'odom_foot_force_9',
            'odom_foot_force_10', 'odom_foot_force_11', 'odom_foot_force_12',
            'odom_foot_position_x1', 'odom_foot_position_y1', 'odom_foot_position_z1',
            'odom_foot_position_x2', 'odom_foot_position_y2', 'odom_foot_position_z2',
            'odom_foot_position_x3', 'odom_foot_position_y3', 'odom_foot_position_z3',
            'odom_foot_position_x4', 'odom_foot_position_y4', 'odom_foot_position_z4',
        ]
        # Path points
        for i in range(10):
            for field in ['x', 'y', 'yaw', 'vx', 'vy', 'time']:
                columns.append(f'odom_path_point_{i+1}_{field}')

        # Low state
        columns += ['low_tick', 'low_version_0', 'low_version_1',
                    'low_mode_pr', 'low_mode_machine',
                    'low_imu_quat_w', 'low_imu_quat_x', 'low_imu_quat_y', 'low_imu_quat_z',
                    'low_imu_gyro_x', 'low_imu_gyro_y', 'low_imu_gyro_z',
                    'low_imu_accel_x', 'low_imu_accel_y', 'low_imu_accel_z',
                    'low_imu_roll', 'low_imu_pitch', 'low_imu_yaw', 'low_imu_temperature']

        # Motor states
        for i in range(self.motor_count):
            columns += [
                f'low_motor_{i}_mode', f'low_motor_{i}_q', f'low_motor_{i}_dq',
                f'low_motor_{i}_ddq', f'low_motor_{i}_tau_est',
                f'low_motor_{i}_temp_0', f'low_motor_{i}_temp_1',
                f'low_motor_{i}_sensor_0', f'low_motor_{i}_sensor_1',
                f'low_motor_{i}_vol', f'low_motor_{i}_motorstate'
            ] + [f'low_motor_{i}_reserve_{j}' for j in range(4)]

        columns += [f'low_wireless_remote_{i}' for i in range(40)]
        columns += [f'low_reserve_{i}' for i in range(4)]
        columns += ['low_crc']

        return columns

    def _open_csv(self):
        """打开CSV文件"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = self.output_dir / f"raw_data_{timestamp}.csv"
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self._get_csv_columns())
        print(f"CSV文件: {self.csv_path}")

    def _odom_callback(self, msg):
        """SportModeState回调"""
        try:
            stamp = getattr(msg, 'stamp', None)
            sec = stamp.sec if stamp else 0
            nanosec = stamp.nanosec if stamp else 0
            position_mode = getattr(msg, 'position_mode', 0)
            velocity_mode = getattr(msg, 'velocity_mode', 0)
            angular_speed = getattr(msg, 'angular_speed', 0.0)
            # 旧字段布局：foot_force(int16[4])用于接触；foot_speed_body(float[12])临时承载六维力
            foot_contact = msg.foot_force[:4] if len(msg.foot_force) >= 4 else [0] * 4
            foot_force = msg.foot_speed_body + [0] * (12 - len(msg.foot_speed_body)) if len(msg.foot_speed_body) < 12 else msg.foot_speed_body[:12]
            foot_position_body = msg.foot_position_body + [0] * (12 - len(msg.foot_position_body)) if len(msg.foot_position_body) < 12 else msg.foot_position_body[:12]

            odom_row = [
                sec, nanosec,
                msg.mode,
                msg.imu_state.quaternion[0], msg.imu_state.quaternion[1],
                msg.imu_state.quaternion[2], msg.imu_state.quaternion[3],
                msg.imu_state.gyroscope[0], msg.imu_state.gyroscope[1], msg.imu_state.gyroscope[2],
                msg.imu_state.accelerometer[0], msg.imu_state.accelerometer[1], msg.imu_state.accelerometer[2],
                msg.imu_state.temperature,
                msg.gait_type,
                position_mode,
                velocity_mode,
                msg.imu_state.rpy[2],
                msg.position[0], msg.position[1], msg.position[2],
                msg.yaw_speed,
                msg.velocity[0], msg.velocity[1], msg.velocity[2],
                angular_speed,
                foot_position_body[0], foot_position_body[1], foot_position_body[2], foot_position_body[3],
                foot_contact[0], foot_contact[1], foot_contact[2], foot_contact[3],
                *foot_force,
                foot_position_body[0], foot_position_body[1], foot_position_body[2],
                foot_position_body[3], foot_position_body[4], foot_position_body[5],
                foot_position_body[6], foot_position_body[7], foot_position_body[8],
                foot_position_body[9], foot_position_body[10], foot_position_body[11],
            ]
            # Path points
            for i in range(10):
                odom_row.extend([
                    msg.path_point[i].x, msg.path_point[i].y,
                    msg.path_point[i].yaw, msg.path_point[i].vx,
                    msg.path_point[i].vy, msg.path_point[i].t_from_start
                ])

            self.odom_data = odom_row
            self._write_row()
        except Exception as e:
            if self.running:  # 只在运行时报错
                print(f"Odom回调错误: {e}")

    def _low_callback(self, msg):
        """LowState回调"""
        try:
            low_row = [
                msg.tick, msg.version[0], msg.version[1],
                msg.mode_pr, msg.mode_machine
            ]
            imu = msg.imu_state
            low_row += [
                imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3],
                imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2],
                imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2],
                imu.rpy[0], imu.rpy[1], imu.rpy[2],
                imu.temperature
            ]

            for motor in msg.motor_state[:self.motor_count]:
                low_row += [
                    motor.mode, motor.q, motor.dq, motor.ddq, motor.tau_est,
                    motor.temperature[0], motor.temperature[1],
                    motor.sensor[0], motor.sensor[1],
                    motor.vol, motor.motorstate
                ] + list(motor.reserve)

            # 填充不足的电机
            for _ in range(self.motor_count - len(msg.motor_state)):
                low_row += [0] * 15

            low_row += list(msg.wireless_remote)
            low_row += list(msg.reserve)
            low_row += [msg.crc]

            self.low_data = low_row
            self._write_row()
        except Exception as e:
            if self.running:  # 只在运行时报错
                print(f"Low回调错误: {e}")

    def _write_row(self):
        """写入一行数据"""
        if not self.running or self.csv_writer is None or self.csv_file is None:
            return

        try:
            current_time = time.time()
            odom_len = 2 + 1 + 4 + 3 + 3 + 1 + 1 + 1 + 1 + 1 + 3 + 1 + 3 + 1 + 4 + 4 + 12 + 12 + 10 * 6
            low_len = 1 + 2 + 2 + 4 + 3 + 3 + 4 + self.motor_count * 15 + 40 + 4 + 1

            odom_row = self.odom_data if self.odom_data else [0] * odom_len
            low_row = self.low_data if self.low_data else [0] * low_len

            row = [current_time] + odom_row + low_row
            self.csv_writer.writerow(row)

            # 定期刷新
            if int(current_time * 1000) % 100 == 0:
                self.csv_file.flush()
        except Exception:
            pass  # 忽略写入错误（可能文件已关闭）

    def collect(self):
        """执行数据采集"""
        print(f"\n{'='*60}")
        print(f"开始数据采集")
        print(f"{'='*60}")
        print(f"输出目录: {self.output_dir}")
        print(f"采集时长: {self.duration} 秒")
        print(f"Domain ID: {self.domain_id}")
        print(f"网络接口: {self.interface}")
        print()

        self._open_csv()
        self.running = True
        self.start_time = time.time()

        # 初始化DDS
        ChannelFactoryInitialize(id=self.domain_id, networkInterface=self.interface)

        odom_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        low_sub = ChannelSubscriber("rt/lowstate", LowState_)

        odom_sub.Init(self._odom_callback, 10)
        low_sub.Init(self._low_callback, 10)

        print(f"正在采集数据... (按Ctrl+C提前结束)")

        try:
            while self.running:
                elapsed = time.time() - self.start_time
                if elapsed >= self.duration:
                    print(f"\n采集完成 ({self.duration}秒)")
                    break

                # 显示进度
                remaining = self.duration - elapsed
                print(f"\r剩余时间: {remaining:.1f}秒", end='', flush=True)
                time.sleep(0.5)
        except KeyboardInterrupt:
            print(f"\n采集被中断")
        finally:
            # 先停止采集，等待回调结束
            self.running = False
            time.sleep(0.1)  # 给回调一点时间完成

            # 然后关闭文件
            if self.csv_file:
                try:
                    self.csv_file.flush()
                    self.csv_file.close()
                except Exception:
                    pass
                self.csv_file = None
                self.csv_writer = None

        return self.csv_path

    def process_csv(self, csv_path: Path) -> Path:
        """处理CSV：计算加速度和接触状态"""
        print(f"\n{'='*60}")
        print(f"处理CSV数据")
        print(f"{'='*60}")

        print(f"读取: {csv_path}")
        df = pd.read_csv(csv_path)

        # 清洗数据
        timestamp_col = 'low_tick'
        if timestamp_col not in df.columns:
            print(f"警告: 缺少 {timestamp_col} 列")
            return csv_path

        raw_ticks = df[timestamp_col].values

        # 首行异常处理
        if len(df) > 1 and raw_ticks[0] == 0 and raw_ticks[1] > 1000:
            print(f"剔除首行异常跳变")
            df = df.iloc[1:].reset_index(drop=True)

        # 去重
        before = len(df)
        df = df.drop_duplicates(subset=timestamp_col, keep='first').reset_index(drop=True)
        after = len(df)
        if before - after > 0:
            print(f"剔除 {before - after} 行重复数据")

        # 时间转换
        time_scale = 1000.0
        time_values = df[timestamp_col].values / time_scale
        if np.any(np.diff(time_values) == 0):
            time_values = time_values + np.arange(len(time_values)) * 1e-9

        # 分段
        dt_array = np.diff(time_values, prepend=time_values[0])
        GAP_THRESHOLD = 0.5
        segment_ids = (dt_array > GAP_THRESHOLD).cumsum()
        df['segment_id'] = segment_ids

        num_segments = segment_ids[-1] + 1
        print(f"检测到 {num_segments} 个数据片段")

        # 处理每个片段
        print("计算加速度...")
        df = df.groupby('segment_id', group_keys=False).apply(
            lambda seg: self._process_segment(seg, timestamp_col, time_scale)
        )

        # 更新接触状态
        print("更新接触状态...")
        if 'low_motor_3_tau_est' in df.columns:
            df['odom_foot_contact_1'] = np.where(
                df['low_motor_3_tau_est'] <= -0.5, 1,
                np.where(df['low_motor_3_tau_est'] > -0.5, 2, 0)
            ).astype(int)

        if 'low_motor_9_tau_est' in df.columns:
            df['odom_foot_contact_2'] = np.where(
                df['low_motor_9_tau_est'] <= -0.5, 1,
                np.where(df['low_motor_9_tau_est'] > -0.5, 2, 0)
            ).astype(int)

        # 删除辅助列
        if 'segment_id' in df.columns:
            del df['segment_id']

        # 保存处理后的CSV
        processed_path = csv_path.parent / csv_path.name.replace('.csv', '_processed.csv')
        df.to_csv(processed_path, index=False, float_format='%.6f')
        print(f"保存: {processed_path}")

        return processed_path

    def _process_segment(self, df_seg, timestamp_col, time_scale):
        """处理单个数据片段"""
        if len(df_seg) < 32:
            for i in range(self.motor_count):
                df_seg[f'low_motor_{i}_ddq'] = 0.0
            for axis in ['x', 'y', 'z']:
                df_seg[f'body_ang_acceleration_{axis}'] = 0.0
            return df_seg

        t_seg = df_seg[timestamp_col].values / time_scale
        if np.any(np.diff(t_seg) <= 0):
            t_seg = t_seg + np.arange(len(t_seg)) * 1e-9

        t_start, t_end = t_seg[0], t_seg[-1]
        fs = 1000.0
        dt_uniform = 1.0 / fs
        t_uniform = np.arange(t_start, t_end, dt_uniform)

        def compute_col(col_name, res_col_name):
            if col_name not in df_seg.columns:
                df_seg[res_col_name] = 0.0
                return

            vals_orig = df_seg[col_name].values
            if np.isnan(vals_orig).any():
                vals_orig = pd.Series(vals_orig).fillna(method='ffill').fillna(method='bfill').values

            try:
                f_interp = interp1d(t_seg, vals_orig, kind='linear', fill_value="extrapolate")
                vals_uniform = f_interp(t_uniform)
                acc_uniform = savgol_filter(vals_uniform, window_length=31, polyorder=2,
                                           deriv=1, delta=dt_uniform)
                f_acc = interp1d(t_uniform, acc_uniform, kind='linear', fill_value="extrapolate")
                df_seg[res_col_name] = f_acc(t_seg)
            except Exception:
                df_seg[res_col_name] = 0.0

        # 计算电机加速度
        for i in range(self.motor_count):
            compute_col(f'low_motor_{i}_dq', f'low_motor_{i}_ddq')

        # 计算角加速度
        for axis in ['x', 'y', 'z']:
            compute_col(f'low_imu_gyro_{axis}', f'body_ang_acceleration_{axis}')

        return df_seg

    def csv_to_dat(self, csv_path: Path, num_motors: int = 12):
        """将CSV转换为DAT格式"""
        print(f"\n{'='*60}")
        print(f"转换为DAT格式")
        print(f"{'='*60}")

        print(f"读取: {csv_path}")
        df = pd.read_csv(csv_path)

        # 定义数据集
        datasets = {
            'g1_robot_low_q.dat': [
                'odom_position_x', 'odom_position_y', 'odom_position_z',
                'low_imu_quat_x', 'low_imu_quat_y', 'low_imu_quat_z', 'low_imu_quat_w'
            ] + [f'low_motor_{i}_q' for i in range(num_motors)],

            'g1_robot_odom_q.dat': [
                'odom_position_x', 'odom_position_y', 'odom_position_z',
                'odom_imu_quaternion_x', 'odom_imu_quaternion_y',
                'odom_imu_quaternion_z', 'odom_imu_quaternion_w'
            ] + [f'low_motor_{i}_q' for i in range(num_motors)],

            'g1_robot_dq.dat': [
                'odom_velocity_x', 'odom_velocity_y', 'odom_velocity_z',
                'low_imu_gyro_x', 'low_imu_gyro_y', 'low_imu_gyro_z'
            ] + [f'low_motor_{i}_dq' for i in range(num_motors)],

            'g1_robot_ddq.dat': [
                'low_imu_accel_x', 'low_imu_accel_y', 'low_imu_accel_z',
                'body_ang_acceleration_x', 'body_ang_acceleration_y', 'body_ang_acceleration_z'
            ] + [f'low_motor_{i}_ddq' for i in range(num_motors)],

            'g1_robot_tau.dat': [f'low_motor_{i}_tau_est' for i in range(num_motors)],

            'g1_robot_contact.dat': ['odom_foot_contact_1', 'odom_foot_contact_2'],

            # 新增：末端接触力（左脚6 + 右脚6）
            'g1_robot_ee_force.dat': [
                'odom_foot_force_1', 'odom_foot_force_2', 'odom_foot_force_3',
                'odom_foot_force_4', 'odom_foot_force_5', 'odom_foot_force_6',
                'odom_foot_force_7', 'odom_foot_force_8', 'odom_foot_force_9',
                'odom_foot_force_10', 'odom_foot_force_11', 'odom_foot_force_12',
            ],
        }

        output_dir = csv_path.parent

        for filename, cols in datasets.items():
            # 检查缺失列
            missing = [c for c in cols if c not in df.columns]
            if missing:
                print(f"警告: {filename} 缺少列: {missing}")
                continue

            data = df[cols].to_numpy().T  # 转置

            # 处理NaN和Inf
            has_nan = np.any(np.isnan(data))
            has_inf = np.any(np.isinf(data))
            if has_nan or has_inf:
                print(f"警告: {filename} 包含 NaN/Inf，已替换为0")
                data = np.nan_to_num(data, nan=0.0, posinf=0.0, neginf=0.0)

            # 保存
            output_path = output_dir / filename
            np.savetxt(output_path, data, delimiter='\t', fmt='%.6f')
            print(f"保存: {output_path}")

        print(f"\nDAT文件已保存到: {output_dir}")

    def run(self):
        """执行完整的数据采集流程"""
        # 1. 采集数据
        csv_path = self.collect()

        if csv_path is None or not csv_path.exists():
            print("错误: 数据采集失败")
            return False

        # 2. 处理CSV
        processed_path = self.process_csv(csv_path)

        # 3. 转换为DAT
        self.csv_to_dat(processed_path)

        print(f"\n{'='*60}")
        print(f"数据采集完成！")
        print(f"{'='*60}")
        print(f"输出目录: {self.output_dir}")
        print(f"文件列表:")
        for f in sorted(self.output_dir.iterdir()):
            print(f"  - {f.name}")

        return True


def main():
    parser = argparse.ArgumentParser(
        description='G1机器人惯性参数辨识数据采集一体化脚本（含末端接触力）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 采集30秒数据到指定目录
  python collect_identification_forceee_data.py ./data/varied_scenes/var_0000 --duration 30

  # 使用不同的domain_id
  python collect_identification_forceee_data.py ./data/test --duration 20 --domain-id 1

  # 只处理已有的CSV文件
  python collect_identification_forceee_data.py ./data/var_0000 --process-only raw_data_xxx.csv
"""
    )

    parser.add_argument('output_dir', type=str,
                        help='输出目录')
    parser.add_argument('--duration', type=float, default=30.0,
                        help='采集时长(秒)，默认30秒')
    parser.add_argument('--domain-id', type=int, default=0,
                        help='DDS Domain ID，默认0')
    parser.add_argument('--interface', type=str, default='lo',
                        help='网络接口，默认lo')
    parser.add_argument('--motor-count', type=int, default=35,
                        help='电机数量，默认35')
    parser.add_argument('--dat-motors', type=int, default=12,
                        help='DAT文件中包含的电机数量，默认12')
    parser.add_argument('--process-only', type=str, default=None,
                        help='只处理已有的CSV文件，不进行采集')

    args = parser.parse_args()

    collector = G1ForceEEDataCollector(
        output_dir=args.output_dir,
        duration=args.duration,
        domain_id=args.domain_id,
        interface=args.interface,
        motor_count=args.motor_count
    )

    if args.process_only:
        # 只处理已有CSV
        csv_path = Path(args.output_dir) / args.process_only
        if not csv_path.exists():
            print(f"错误: 文件不存在: {csv_path}")
            sys.exit(1)
        processed = collector.process_csv(csv_path)
        collector.csv_to_dat(processed, num_motors=args.dat_motors)
    else:
        # 完整采集流程
        success = collector.run()
        sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
