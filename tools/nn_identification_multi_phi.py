#!/usr/bin/env python3
"""
神经网络惯性参数辨识 (多phi版本)
================================
使用多组不同惯性参数的仿真数据训练神经网络

关键改进：
1. 支持加载多组不同phi的仿真数据
2. 每个样本使用其对应的phi_true作为标签（而非统一的phi_prior）
3. 网络学习从 (q, dq, ddq, tau) 预测该样本对应的 phi

数据结构:
    每个变体目录包含:
    - g1_robot_low_q.dat, g1_robot_dq.dat, g1_robot_ddq.dat, g1_robot_tau.dat
    - phi_true.npy: 该变体的真实惯性参数

作者：ustczxh
日期：2025-01-30
"""

import os
import sys
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split
from tqdm import tqdm
import matplotlib.pyplot as plt
from typing import Tuple, List, Optional, Dict
import json
from pathlib import Path


def set_seed(seed=42):
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


# ======================= 数据集定义（多phi版本） =======================

class MultiPhiTimeSeriesDataset(Dataset):
    """
    多phi时序数据集
    
    每个变体有自己的phi_true，网络需要学习从运动数据预测对应的phi
    
    输入: (q, dq, ddq, tau) 的时间序列窗口
    输出/标签: 该轨迹对应的 phi_true
    """
    
    def __init__(self, data_dirs: List[str], robot_name: str = "g1", 
                 window_size: int = 200, stride: int = 100,
                 normalize: bool = True):
        """
        参数:
            data_dirs: 数据目录列表，每个目录包含一个变体的数据
            robot_name: 机器人名称
            window_size: 滑动窗口大小
            stride: 滑动步长
            normalize: 是否归一化输入数据
        """
        self.window_size = window_size
        self.stride = stride
        self.normalize = normalize
        self.samples = []
        
        # 归一化统计量
        self.q_mean, self.q_std = None, None
        self.dq_mean, self.dq_std = None, None
        self.ddq_mean, self.ddq_std = None, None
        self.tau_mean, self.tau_std = None, None
        
        # 加载所有变体数据
        all_data = []
        for data_dir in data_dirs:
            data = self._load_variant_data(data_dir, robot_name)
            if data is not None:
                all_data.append(data)
        
        if len(all_data) == 0:
            print("警告：没有成功加载任何数据！")
            return
        
        # 计算归一化统计量
        if normalize:
            self._compute_normalization_stats(all_data)
        
        # 创建样本
        for data in all_data:
            self._create_samples(data)
        
        print(f"总共创建了 {len(self.samples)} 个训练样本")
        print(f"涵盖 {len(all_data)} 组不同的惯性参数配置")
    
    def _load_variant_data(self, data_dir: str, robot_name: str) -> Optional[Dict]:
        """加载一个变体的数据"""
        data_dir = Path(data_dir)
        
        try:
            q = np.loadtxt(data_dir / f"{robot_name}_robot_low_q.dat", 
                           delimiter='\t', dtype=np.float32)
            dq = np.loadtxt(data_dir / f"{robot_name}_robot_dq.dat", 
                            delimiter='\t', dtype=np.float32)
            ddq = np.loadtxt(data_dir / f"{robot_name}_robot_ddq.dat", 
                             delimiter='\t', dtype=np.float32)
            tau = np.loadtxt(data_dir / f"{robot_name}_robot_tau.dat", 
                             delimiter='\t', dtype=np.float32)
            
            # 加载该变体对应的phi_true
            phi_true_path = data_dir / "phi_true.npy"
            if phi_true_path.exists():
                phi_true = np.load(phi_true_path).astype(np.float32)
            else:
                print(f"警告: {data_dir} 没有找到 phi_true.npy，跳过")
                return None
            
            print(f"从 {data_dir} 加载数据:")
            print(f"  q={q.shape}, dq={dq.shape}, ddq={ddq.shape}, tau={tau.shape}")
            print(f"  phi_true shape: {phi_true.shape}")
            
            return {
                'q': q,        # (num_joints, num_samples) 或 (num_samples, num_joints)
                'dq': dq,
                'ddq': ddq,
                'tau': tau,
                'phi_true': phi_true,  # 该变体的真实惯性参数
                'source': str(data_dir)
            }
            
        except Exception as e:
            print(f"加载 {data_dir} 失败: {e}")
            return None
    
    def _compute_normalization_stats(self, all_data: List[Dict]):
        """计算归一化统计量"""
        # 确保数据格式为 (num_joints, num_samples)
        all_q = np.concatenate([d['q'] if d['q'].shape[0] < d['q'].shape[1] else d['q'].T 
                                 for d in all_data], axis=1)
        all_dq = np.concatenate([d['dq'] if d['dq'].shape[0] < d['dq'].shape[1] else d['dq'].T 
                                  for d in all_data], axis=1)
        all_ddq = np.concatenate([d['ddq'] if d['ddq'].shape[0] < d['ddq'].shape[1] else d['ddq'].T 
                                   for d in all_data], axis=1)
        all_tau = np.concatenate([d['tau'] if d['tau'].shape[0] < d['tau'].shape[1] else d['tau'].T 
                                   for d in all_data], axis=1)
        
        self.q_mean = all_q.mean(axis=1, keepdims=True)
        self.q_std = all_q.std(axis=1, keepdims=True) + 1e-8
        self.dq_mean = all_dq.mean(axis=1, keepdims=True)
        self.dq_std = all_dq.std(axis=1, keepdims=True) + 1e-8
        self.ddq_mean = all_ddq.mean(axis=1, keepdims=True)
        self.ddq_std = all_ddq.std(axis=1, keepdims=True) + 1e-8
        self.tau_mean = all_tau.mean(axis=1, keepdims=True)
        self.tau_std = all_tau.std(axis=1, keepdims=True) + 1e-8
        
        print("归一化统计量已计算")
    
    def _create_samples(self, data: Dict):
        """创建滑动窗口样本"""
        q, dq, ddq, tau = data['q'], data['dq'], data['ddq'], data['tau']
        phi_true = data['phi_true']
        
        # 确保数据格式为 (num_joints, num_samples)
        if q.shape[0] > q.shape[1]:
            q, dq, ddq, tau = q.T, dq.T, ddq.T, tau.T
        
        num_samples = q.shape[1]
        
        for start in range(0, num_samples - self.window_size, self.stride):
            end = start + self.window_size
            
            sample = {
                'q': q[:, start:end].T,      # (window_size, num_joints)
                'dq': dq[:, start:end].T,
                'ddq': ddq[:, start:end].T,
                'tau': tau[:, start:end].T,
                'phi_true': phi_true,         # 该样本对应的真实phi
                'source': data['source']
            }
            self.samples.append(sample)
    
    def __len__(self):
        return len(self.samples)
    
    def __getitem__(self, idx):
        sample = self.samples[idx]
        
        q = sample['q'].copy()
        dq = sample['dq'].copy()
        ddq = sample['ddq'].copy()
        tau = sample['tau'].copy()
        phi_true = sample['phi_true'].copy()
        
        # 归一化输入
        if self.normalize and self.q_mean is not None:
            q = (q - self.q_mean.T) / self.q_std.T
            dq = (dq - self.dq_mean.T) / self.dq_std.T
            ddq = (ddq - self.ddq_mean.T) / self.ddq_std.T
            tau = (tau - self.tau_mean.T) / self.tau_std.T
        
        # 拼接输入: (window_size, num_joints * 4)
        features = np.concatenate([q, dq, ddq, tau], axis=1)
        
        return {
            'features': torch.FloatTensor(features),    # (seq_len, input_dim)
            'phi_true': torch.FloatTensor(phi_true),    # 该样本的真实phi (标签)
            'tau_raw': torch.FloatTensor(sample['tau']),  # 原始tau
            'q_raw': torch.FloatTensor(sample['q']),
            'dq_raw': torch.FloatTensor(sample['dq']),
            'ddq_raw': torch.FloatTensor(sample['ddq']),
        }
    
    def get_normalization_params(self):
        """获取归一化参数"""
        return {
            'q_mean': self.q_mean, 'q_std': self.q_std,
            'dq_mean': self.dq_mean, 'dq_std': self.dq_std,
            'ddq_mean': self.ddq_mean, 'ddq_std': self.ddq_std,
            'tau_mean': self.tau_mean, 'tau_std': self.tau_std,
        }


# ======================= 模型定义 =======================

class PhiPredictionLSTM(nn.Module):
    """
    LSTM模型：从时序数据预测惯性参数
    
    直接预测phi（不使用先验残差），因为不同样本有不同的phi_true
    """
    
    def __init__(self, input_dim: int, output_dim: int, 
                 hidden_dim: int = 256, num_layers: int = 2,
                 dropout: float = 0.1, bidirectional: bool = True):
        super().__init__()
        
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        self.bidirectional = bidirectional
        
        # LSTM编码器
        self.lstm = nn.LSTM(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            dropout=dropout if num_layers > 1 else 0,
            batch_first=True,
            bidirectional=bidirectional
        )
        
        lstm_output_dim = hidden_dim * 2 if bidirectional else hidden_dim
        
        # 注意力机制
        self.attention = nn.Sequential(
            nn.Linear(lstm_output_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 1)
        )
        
        # 输出投影
        self.output_proj = nn.Sequential(
            nn.Linear(lstm_output_dim, hidden_dim),
            nn.LayerNorm(hidden_dim),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim // 2, output_dim)
        )
        
    def forward(self, x):
        """
        参数:
            x: (batch, seq_len, input_dim) 时间序列
        返回:
            phi: (batch, num_params) 预测的惯性参数
        """
        # LSTM编码
        lstm_out, _ = self.lstm(x)
        
        # 注意力加权
        attn_weights = self.attention(lstm_out)
        attn_weights = torch.softmax(attn_weights, dim=1)
        
        # 加权求和
        context = torch.sum(lstm_out * attn_weights, dim=1)
        
        # 输出投影
        phi = self.output_proj(context)
        
        return phi


class PhiPredictionTransformer(nn.Module):
    """
    Transformer模型：从时序数据预测惯性参数
    """
    
    def __init__(self, input_dim: int, output_dim: int, 
                 d_model: int = 128, nhead: int = 4, num_layers: int = 3,
                 dropout: float = 0.1, max_seq_len: int = 500):
        super().__init__()
        
        self.d_model = d_model
        
        # 输入投影
        self.input_proj = nn.Linear(input_dim, d_model)
        
        # 位置编码
        self.pos_encoding = self._create_positional_encoding(max_seq_len, d_model)
        
        # Transformer编码器
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=d_model, 
            nhead=nhead, 
            dim_feedforward=d_model * 4,
            dropout=dropout, 
            batch_first=True,
            activation='gelu'
        )
        self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        
        # 全局池化
        self.global_pool = nn.Sequential(
            nn.Linear(d_model, d_model),
            nn.LayerNorm(d_model),
            nn.GELU(),
        )
        
        # 输出投影
        self.output_proj = nn.Sequential(
            nn.Linear(d_model, d_model),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(d_model, output_dim)
        )
    
    def _create_positional_encoding(self, max_len, d_model):
        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2).float() * (-np.log(10000.0) / d_model))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        return nn.Parameter(pe.unsqueeze(0), requires_grad=False)
        
    def forward(self, x):
        batch_size, seq_len, _ = x.shape
        
        x = self.input_proj(x)
        x = x + self.pos_encoding[:, :seq_len, :]
        x = self.transformer(x)
        x = x.mean(dim=1)
        x = self.global_pool(x)
        phi = self.output_proj(x)
        
        return phi


# ======================= 损失函数 =======================

class MultiPhiLoss(nn.Module):
    """
    多phi训练的损失函数
    
    损失组成:
    1. 参数预测误差: ||phi_pred - phi_true||^2 (主要监督信号)
    2. 物理约束: 质量非负、惯量正定等
    """
    
    def __init__(self, lambda_mse: float = 1.0, lambda_physics: float = 0.1,
                 num_links: int = 13, params_per_link: int = 11,
                 mass_bounds: Tuple[float, float] = (0.01, 50.0)):
        super().__init__()
        self.lambda_mse = lambda_mse
        self.lambda_physics = lambda_physics
        self.num_links = num_links
        self.params_per_link = params_per_link
        self.mass_min, self.mass_max = mass_bounds
        
    def forward(self, phi_pred: torch.Tensor, phi_true: torch.Tensor) -> Tuple[torch.Tensor, dict]:
        """
        计算损失
        
        参数:
            phi_pred: (batch, num_params) 预测的惯性参数
            phi_true: (batch, num_params) 真实参数
        """
        # 1. MSE损失
        loss_mse = nn.functional.mse_loss(phi_pred, phi_true)
        
        # 2. 相对误差（可选，用于监控）
        rel_error = torch.abs(phi_pred - phi_true) / (torch.abs(phi_true) + 1e-6)
        mean_rel_error = rel_error.mean()
        
        # 3. 物理约束惩罚
        loss_physics = torch.tensor(0.0, device=phi_pred.device)
        
        for i in range(min(self.num_links, phi_pred.shape[1] // self.params_per_link)):
            idx = i * self.params_per_link
            
            if idx >= phi_pred.shape[1]:
                break
            
            mass = phi_pred[:, idx]
            
            # 质量边界约束
            loss_physics = loss_physics + torch.relu(self.mass_min - mass).mean()
            loss_physics = loss_physics + torch.relu(mass - self.mass_max).mean()
            
            # 惯量非负约束 (I_xx, I_yy, I_zz)
            if idx + 9 < phi_pred.shape[1]:
                I_xx = phi_pred[:, idx + 4]
                I_yy = phi_pred[:, idx + 6]
                I_zz = phi_pred[:, idx + 9]
                loss_physics = loss_physics + torch.relu(-I_xx).mean()
                loss_physics = loss_physics + torch.relu(-I_yy).mean()
                loss_physics = loss_physics + torch.relu(-I_zz).mean()
        
        # 总损失
        loss = self.lambda_mse * loss_mse + self.lambda_physics * loss_physics
        
        loss_dict = {
            'loss_mse': loss_mse.item(),
            'loss_physics': loss_physics.item(),
            'mean_rel_error': mean_rel_error.item(),
            'loss_total': loss.item()
        }
        
        return loss, loss_dict


# ======================= 训练器 =======================

class MultiPhiTrainer:
    """多phi数据训练器"""
    
    def __init__(self, model: nn.Module, train_loader: DataLoader, val_loader: DataLoader,
                 loss_fn: nn.Module, optimizer: optim.Optimizer, scheduler=None,
                 device: str = 'cuda' if torch.cuda.is_available() else 'cpu',
                 save_dir: str = './checkpoints'):
        
        self.model = model.to(device)
        self.train_loader = train_loader
        self.val_loader = val_loader
        self.loss_fn = loss_fn
        self.optimizer = optimizer
        self.scheduler = scheduler
        self.device = device
        self.save_dir = save_dir
        
        os.makedirs(save_dir, exist_ok=True)
        
        self.train_losses = []
        self.val_losses = []
        self.best_val_loss = float('inf')
        
    def train_epoch(self) -> dict:
        """训练一个epoch"""
        self.model.train()
        epoch_losses = []
        
        for batch in tqdm(self.train_loader, desc="Training", leave=False):
            self.optimizer.zero_grad()
            
            features = batch['features'].to(self.device)
            phi_true = batch['phi_true'].to(self.device)
            
            # 前向传播
            phi_pred = self.model(features)
            
            # 计算损失
            loss, loss_dict = self.loss_fn(phi_pred, phi_true)
            
            loss.backward()
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
            self.optimizer.step()
            
            epoch_losses.append(loss_dict)
        
        avg_losses = {k: np.mean([l[k] for l in epoch_losses]) for k in epoch_losses[0]}
        return avg_losses
    
    @torch.no_grad()
    def validate(self) -> dict:
        """验证"""
        self.model.eval()
        epoch_losses = []
        
        for batch in self.val_loader:
            features = batch['features'].to(self.device)
            phi_true = batch['phi_true'].to(self.device)
            
            phi_pred = self.model(features)
            loss, loss_dict = self.loss_fn(phi_pred, phi_true)
            
            epoch_losses.append(loss_dict)
        
        avg_losses = {k: np.mean([l[k] for l in epoch_losses]) for k in epoch_losses[0]}
        return avg_losses
    
    def train(self, num_epochs: int):
        """完整训练流程"""
        print(f"开始训练，设备: {self.device}")
        print(f"训练集大小: {len(self.train_loader.dataset)}, 验证集大小: {len(self.val_loader.dataset)}")
        
        for epoch in range(num_epochs):
            train_loss = self.train_epoch()
            val_loss = self.validate()
            
            self.train_losses.append(train_loss)
            self.val_losses.append(val_loss)
            
            if self.scheduler:
                self.scheduler.step()
            
            print(f"Epoch {epoch+1}/{num_epochs}")
            print(f"  Train: MSE={train_loss['loss_mse']:.6f}, Physics={train_loss['loss_physics']:.6f}, "
                  f"RelErr={train_loss['mean_rel_error']:.4f}, Total={train_loss['loss_total']:.6f}")
            print(f"  Val:   MSE={val_loss['loss_mse']:.6f}, Physics={val_loss['loss_physics']:.6f}, "
                  f"RelErr={val_loss['mean_rel_error']:.4f}, Total={val_loss['loss_total']:.6f}")
            
            if val_loss['loss_total'] < self.best_val_loss:
                self.best_val_loss = val_loss['loss_total']
                self.save_checkpoint('best_model.pth')
                print(f"  ★ 保存最佳模型 (val_loss={val_loss['loss_total']:.6f})")
        
        self.save_checkpoint('final_model.pth')
        self.plot_losses()
    
    def save_checkpoint(self, filename: str):
        """保存检查点"""
        path = os.path.join(self.save_dir, filename)
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'train_losses': self.train_losses,
            'val_losses': self.val_losses,
        }, path)
    
    def load_checkpoint(self, filename: str):
        """加载检查点"""
        path = os.path.join(self.save_dir, filename)
        checkpoint = torch.load(path, map_location=self.device, weights_only=False)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        print(f"加载模型: {path}")
    
    def plot_losses(self):
        """绘制损失曲线"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        axes = axes.flatten()
        
        keys = ['loss_mse', 'loss_physics', 'mean_rel_error', 'loss_total']
        titles = ['MSE Loss', 'Physics Constraints', 'Mean Relative Error', 'Total Loss']
        
        for ax, key, title in zip(axes, keys, titles):
            train_vals = [l[key] for l in self.train_losses]
            val_vals = [l[key] for l in self.val_losses]
            
            ax.plot(train_vals, label='Train')
            ax.plot(val_vals, label='Val')
            ax.set_xlabel('Epoch')
            ax.set_ylabel('Loss')
            ax.set_title(title)
            ax.legend()
            ax.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.save_dir, 'loss_curves.png'))
        plt.close()
        print(f"损失曲线已保存到 {self.save_dir}/loss_curves.png")


# ======================= 主函数 =======================

def main():
    """主函数"""
    import argparse
    parser = argparse.ArgumentParser(description='多phi神经网络惯性参数辨识')
    parser.add_argument('--data-dir', type=str, default='./data/varied_inertia',
                        help='数据目录（包含多个变体子目录）')
    parser.add_argument('--epochs', type=int, default=100, help='训练轮数')
    parser.add_argument('--batch-size', type=int, default=64, help='批大小')
    parser.add_argument('--lr', type=float, default=1e-3, help='学习率')
    parser.add_argument('--model-type', type=str, choices=['lstm', 'transformer'], 
                        default='lstm', help='模型类型')
    parser.add_argument('--hidden-dim', type=int, default=256, help='隐藏层维度')
    parser.add_argument('--window-size', type=int, default=200, help='时间窗口大小')
    parser.add_argument('--stride', type=int, default=100, help='滑动步长')
    parser.add_argument('--save-dir', type=str, default='./nn_checkpoints_multi_phi',
                        help='模型保存目录')
    parser.add_argument('--skip-train', action='store_true', help='跳过训练')
    args = parser.parse_args()
    
    set_seed(42)
    
    print("=" * 60)
    print("神经网络惯性参数辨识 (多phi版本)")
    print("=" * 60)
    
    # 查找所有变体目录
    data_base = Path(args.data_dir)
    if not data_base.exists():
        print(f"错误：数据目录不存在: {data_base}")
        print("\n请先运行 generate_varied_inertia_data.py 生成数据")
        return
    
    data_dirs = sorted([str(d) for d in data_base.iterdir() if d.is_dir() and d.name.startswith('var_')])
    
    if len(data_dirs) == 0:
        print(f"错误：在 {data_base} 中没有找到变体数据目录 (var_xxxx)")
        return
    
    print(f"\n找到 {len(data_dirs)} 个变体目录:")
    for d in data_dirs[:5]:
        print(f"  - {d}")
    if len(data_dirs) > 5:
        print(f"  ... 等共 {len(data_dirs)} 个")
    
    # 创建数据集
    print("\n" + "=" * 60)
    print("创建数据集")
    print("=" * 60)
    
    dataset = MultiPhiTimeSeriesDataset(
        data_dirs=data_dirs,
        robot_name='g1',
        window_size=args.window_size,
        stride=args.stride,
        normalize=True
    )
    
    if len(dataset) == 0:
        print("错误：数据集为空")
        return
    
    # 划分训练集和验证集
    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = random_split(dataset, [train_size, val_size])
    
    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True, num_workers=4)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size, num_workers=4)
    
    # 推断维度
    sample = dataset[0]
    input_dim = sample['features'].shape[1]
    output_dim = sample['phi_true'].shape[0]
    
    print(f"输入维度: {input_dim}")
    print(f"输出维度 (phi): {output_dim}")
    print(f"训练集大小: {len(train_dataset)}")
    print(f"验证集大小: {len(val_dataset)}")
    
    # 创建模型
    print("\n" + "=" * 60)
    print("创建模型")
    print("=" * 60)
    
    if args.model_type == 'lstm':
        model = PhiPredictionLSTM(
            input_dim=input_dim,
            output_dim=output_dim,
            hidden_dim=args.hidden_dim,
            num_layers=2,
            dropout=0.1
        )
    else:
        model = PhiPredictionTransformer(
            input_dim=input_dim,
            output_dim=output_dim,
            d_model=128,
            nhead=4,
            num_layers=3,
            dropout=0.1,
            max_seq_len=args.window_size
        )
    
    print(f"模型类型: {args.model_type}")
    print(f"模型参数量: {sum(p.numel() for p in model.parameters()):,}")
    
    # 训练
    loss_fn = MultiPhiLoss(
        lambda_mse=1.0,
        lambda_physics=0.1,
        num_links=output_dim // 11,
        params_per_link=11
    )
    
    optimizer = optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)
    
    trainer = MultiPhiTrainer(
        model=model,
        train_loader=train_loader,
        val_loader=val_loader,
        loss_fn=loss_fn,
        optimizer=optimizer,
        scheduler=scheduler,
        save_dir=args.save_dir
    )
    
    if not args.skip_train:
        print("\n" + "=" * 60)
        print("开始训练")
        print("=" * 60)
        trainer.train(num_epochs=args.epochs)
    else:
        trainer.load_checkpoint('best_model.pth')
    
    # 评估
    print("\n" + "=" * 60)
    print("评估模型")
    print("=" * 60)
    
    model.eval()
    with torch.no_grad():
        # 取一个验证样本
        sample = val_dataset[0]
        features = sample['features'].unsqueeze(0).to(trainer.device)
        phi_true = sample['phi_true'].numpy()
        
        phi_pred = model(features).cpu().numpy().squeeze()
        
        print("\n惯性参数预测对比 (前几个参数):")
        print("-" * 60)
        print(f"{'参数索引':<10} {'真值':<15} {'预测值':<15} {'相对误差':<15}")
        print("-" * 60)
        for i in range(min(20, len(phi_true))):
            rel_err = abs(phi_pred[i] - phi_true[i]) / (abs(phi_true[i]) + 1e-6) * 100
            print(f"{i:<10} {phi_true[i]:<15.6f} {phi_pred[i]:<15.6f} {rel_err:<15.2f}%")
    
    print("\n" + "=" * 60)
    print("完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()
