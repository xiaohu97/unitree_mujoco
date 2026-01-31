# 惯性参数辨识数据集生成指南

## 概述

为神经网络惯性参数辨识生成多组不同惯性参数(phi)的仿真数据。

## 核心问题与解决方案

### 问题
你的神经网络需要学习从 `(q, dq, ddq, tau)` 预测惯性参数 `phi`。但如果所有训练数据都来自同一个URDF/模型，那么所有样本的 `phi_true` 都相同，网络无法学习到有效的映射关系。

### 解决方案
生成多组**不同惯性参数**的仿真数据：
1. 修改机器人模型的惯性参数（质量、质心、惯量等）
2. 运行仿真收集 `(q, dq, ddq, tau)`
3. 记录对应的 `phi_true` 作为标签
4. 重复上述步骤，生成多组不同 phi 的数据

## 两种实现方法

### 方法一：运行时修改MuJoCo模型参数（推荐）

直接在代码中修改 `mjModel` 结构体的惯性参数：

```python
import mujoco

# 加载模型
model = mujoco.MjModel.from_xml_path("path/to/model.xml")
data = mujoco.MjData(model)

# 修改惯性参数
model.body_mass[body_id] = new_mass           # 质量
model.body_ipos[body_id] = [x, y, z]          # 质心位置
model.body_inertia[body_id] = [Ixx, Iyy, Izz] # 主惯量
model.dof_armature[joint_id] = new_armature   # 转子惯量

# 重要：修改后必须更新派生常量！
mujoco.mj_setConst(model, data)
```

**关键MuJoCo模型参数：**

| 参数 | 描述 | 维度 |
|------|------|------|
| `model.body_mass[i]` | 第i个body的质量 | 标量 |
| `model.body_ipos[i]` | 第i个body的惯性中心相对于body frame的位置 | (3,) |
| `model.body_iquat[i]` | 惯性主轴方向（四元数） | (4,) |
| `model.body_inertia[i]` | 主惯量 [I_xx, I_yy, I_zz] | (3,) |
| `model.dof_armature[j]` | 第j个自由度的转子惯量 | 标量 |

### 方法二：生成多个MJCF文件

修改MJCF/URDF文件中的惯性参数，每次仿真加载不同的文件：

```xml
<!-- 在MJCF中，惯性参数定义在body的inertial子元素中 -->
<body name="left_hip_pitch_link" pos="0 0.064452 -0.1027">
    <inertial pos="0.002741 0.047791 -0.02606" 
              quat="0.954862 0.293964 0.0302556 0.030122"
              mass="1.35"   <!-- 修改这里 -->
              diaginertia="0.00181517 0.00153422 0.00116212"/>  <!-- 和这里 -->
    ...
</body>
```

## 应该变化哪些参数？

### 推荐的参数变化策略

| 参数 | 变化方式 | 典型范围 | 备注 |
|------|----------|----------|------|
| **质量 (mass)** | 乘性噪声 + 全局缩放 | ±10~20% | 保持正值 |
| **质心位置 (com)** | 加性噪声 | ±1~2 cm | 物理合理范围 |
| **惯量 (inertia)** | 乘性噪声 | ±10~20% | 保持正定性 |
| **转子惯量 (armature)** | 乘性噪声 | ±10~30% | 电机特性变化 |

### 参数变化代码示例

```python
def generate_varied_params(original_params, seed=42):
    np.random.seed(seed)
    params = copy.deepcopy(original_params)
    
    # 全局质量缩放 (模拟负载变化)
    global_mass_scale = np.random.uniform(0.85, 1.15)
    
    for i in range(num_bodies):
        # 质量：乘性噪声
        mass_noise = 1.0 + np.random.randn() * 0.1  # 10% 标准差
        mass_noise = np.clip(mass_noise, 0.7, 1.3)
        params['mass'][i] *= mass_noise * global_mass_scale
        
        # 质心位置：加性噪声
        params['com'][i] += np.random.randn(3) * 0.01  # 1cm 标准差
        
        # 惯量：乘性噪声（保持正定性）
        inertia_noise = 1.0 + np.random.randn(3) * 0.1
        inertia_noise = np.clip(inertia_noise, 0.7, 1.3)
        params['inertia'][i] *= inertia_noise
    
    return params
```

## 完整工作流程

### 1. 生成多变体数据

```bash
cd /home/ustczxh/humanoid/unitree_mujoco

# 使用运行时参数修改方法（推荐）
python tools/generate_varied_inertia_data.py \
    --model ./unitree_robots/g1/scene_29dof.xml \
    --output ./data/varied_inertia \
    --num-variations 50 \
    --duration 10.0 \
    --mass-noise 0.15 \
    --com-noise 0.01 \
    --inertia-noise 0.15 \
    --method runtime
```

### 2. 训练神经网络

```bash
# 使用多phi版本的训练脚本
python tools/nn_identification_multi_phi.py \
    --data-dir ./data/varied_inertia \
    --epochs 100 \
    --batch-size 64 \
    --model-type lstm \
    --save-dir ./nn_checkpoints_multi_phi
```

### 3. 数据目录结构

```
data/varied_inertia/
├── var_0000/
│   ├── g1_robot_low_q.dat
│   ├── g1_robot_dq.dat
│   ├── g1_robot_ddq.dat
│   ├── g1_robot_tau.dat
│   ├── g1_robot_contact.dat
│   ├── phi_true.npy          # 该变体的真实惯性参数
│   └── meta.json
├── var_0001/
│   └── ...
├── var_0002/
│   └── ...
└── README.md
```

## 关键代码解释

### phi 向量格式（每个link 11个参数）

```
phi = [
    mass,           # 质量
    h_x, h_y, h_z,  # 一阶矩 (h = mass * com)
    I_xx, I_xy, I_yy, I_xz, I_yz, I_zz,  # 惯量张量
    rotor_inertia   # 转子惯量
] × num_links
```

### 将MuJoCo参数转换为phi

```python
def mujoco_params_to_phi(model):
    phi = []
    for i in range(model.nbody):
        mass = model.body_mass[i]
        ipos = model.body_ipos[i]   # 质心位置
        inertia = model.body_inertia[i]  # 主惯量
        
        # 一阶矩
        h = mass * ipos
        
        # 惯量（假设主轴对齐）
        I_xx, I_yy, I_zz = inertia
        I_xy, I_xz, I_yz = 0, 0, 0
        
        # 转子惯量（需要根据关节-连杆映射）
        rotor = 0.0  # 简化处理
        
        phi.extend([mass, h[0], h[1], h[2], 
                   I_xx, I_xy, I_yy, I_xz, I_yz, I_zz, rotor])
    
    return np.array(phi)
```

## 激励轨迹设计

为了更好地辨识惯性参数，需要使用**激励轨迹**（exciting trajectory）：

```python
def exciting_trajectory_controller(q, dq, t, model):
    """
    多频率正弦叠加轨迹
    可以激励系统的各种动力学模式
    """
    nu = model.nu
    frequencies = [0.1, 0.2, 0.5, 1.0, 2.0]  # Hz
    amplitudes = [0.3, 0.2, 0.1, 0.05, 0.02]  # rad
    
    q_target = np.zeros(nu)
    for freq, amp in zip(frequencies, amplitudes):
        phase = np.random.rand(nu) * 2 * np.pi
        q_target += amp * np.sin(2 * np.pi * freq * t + phase)
    
    # PD 控制
    kp, kd = 100.0, 10.0
    return kp * (q_target - q_joint) + kd * (0 - dq_joint)
```

## 注意事项

1. **mj_setConst 必须调用**：修改惯性参数后，必须调用 `mj_setConst()` 更新MuJoCo的内部派生常量

2. **保持物理合理性**：
   - 质量必须为正
   - 惯量矩阵必须正定
   - 参数变化不要太极端

3. **phi_true 对齐**：确保每个变体保存的 `phi_true.npy` 与实际使用的参数完全一致

4. **接触约束**：如果机器人有接触（如双足站立），接触约束会影响动力学，需要考虑

## 与你现有代码的集成

你的 `unitree_mujoco` 仿真框架主要是 C++ 的。如果要在现有框架中集成参数变化功能，可以：

1. **C++ 中直接修改参数**：
```cpp
// 在 main.cc 或 unitree_sdk2_bridge.h 中
m->body_mass[body_id] = new_mass;
m->body_ipos[3*body_id + 0] = new_cx;
m->body_ipos[3*body_id + 1] = new_cy;
m->body_ipos[3*body_id + 2] = new_cz;
mj_setConst(m, d);
```

2. **通过配置文件**：扩展 `config.yaml` 添加参数变化配置

3. **使用 Python 包装器**：保持现有 C++ 框架不变，用 Python 脚本生成数据

## 文件位置

- 数据生成脚本：`/home/ustczxh/humanoid/unitree_mujoco/tools/generate_varied_inertia_data.py`
- 多phi训练脚本：`/home/ustczxh/humanoid/unitree_mujoco/tools/nn_identification_multi_phi.py`
- 本说明文档：`/home/ustczxh/humanoid/unitree_mujoco/tools/README_inertia_identification.md`
