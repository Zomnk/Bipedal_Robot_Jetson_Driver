# Bipedal Robot Jetson RL Inference

基于Jetson的双足机器人强化学习推理程序，通过UDP与ODroid-C4上位机通信，实现实时运动控制策略部署。

## 项目概述

本项目为双足机器人系统的边缘计算模块，负责：
- 加载PyTorch JIT编译的强化学习模型
- 接收来自ODroid的机器人状态观测数据
- 实时推理输出关节控制动作
- 支持CUDA加速推理（Jetson平台）

**系统架构**:
```
STM32 (电机驱动) <--SPI--> ODroid-C4 (运控上位机) <--UDP--> Jetson (RL推理)
```

## 目录结构

```
sim2sim_lcm/
├── CMakeLists.txt              # CMake构建配置
├── README.md                   # 本文件
├── model_jitt.pt               # PyTorch JIT编译的RL模型
├── modelt.pt                   # 备用模型
├── include/                    # 头文件
│   ├── udp_publish_tinker.h    # UDP通信主程序头文件
│   ├── udp_publish.h           # UDP通信备用版本
│   ├── Request.hpp             # 观测数据消息定义
│   ├── Response.hpp            # 动作数据消息定义
│   ├── mathTools.h             # 数学工具函数
│   ├── mathTypes.h             # 数学类型定义
│   ├── LowPassFilter.h         # 低通滤波器
│   ├── enumClass.h             # 枚举类定义
│   ├── lcm_publish.h           # LCM发布接口（可选）
│   └── lcm_service.h           # LCM服务接口（可选）
├── src/                        # 源文件
│   ├── udp_publisher_tinker.cpp  # UDP通信主程序（推荐使用）
│   ├── udp_publisher.cpp         # UDP通信备用版本（12关节）
│   ├── lcm_publisher.cpp         # LCM发布程序（仿真测试用）
│   ├── lcm_service.cpp           # LCM服务程序（仿真测试用）
│   ├── lcm_client.py             # Python LCM客户端示例
│   └── back/                     # 历史备份文件
├── lcm_types/                  # LCM消息类型定义（用于仿真）
│   ├── request.lcm             # 观测数据LCM定义
│   ├── response.lcm            # 动作数据LCM定义
│   └── my_lcm/                 # 生成的LCM C++/Python绑定
└── build/                      # 编译输出目录
```

## 核心文件说明

### 1. **udp_publisher_tinker.cpp** (主程序)
- **功能**: 实时UDP服务器，接收ODroid的观测数据，推理并返回动作
- **关节数量**: 10个（双足机器人，每腿5个）
- **通信端口**: 10000 (默认)
- **观测维度**: 39维 (角速度3 + 姿态3 + 指令3 + 关节位置10 + 关节速度10 + 上次动作10)
- **推理模式**: 支持历史观测缓冲（history_length=10）

### 2. **udp_publisher.cpp** (备用版本)
- **关节数量**: 12个（可能用于四足或其他配置）
- **观测维度**: 45维

### 3. **消息定义**

#### Request (ODroid → Jetson)
```cpp
struct _msg_request {
    float trigger;          // 触发标志 (1=有效数据)
    float command[4];       // 控制指令 [vx, vy, yaw_rate, ...]
    float eu_ang[3];        // 欧拉角 [roll, pitch, yaw] (rad)
    float omega[3];         // 角速度 [x, y, z] (rad/s)
    float acc[3];           // 加速度 [x, y, z] (m/s^2)
    float q[10];            // 关节位置 (rad)
    float dq[10];           // 关节速度 (rad/s)
    float tau[10];          // 关节力矩 (Nm) - 预留
    float init_pos[10];     // 初始站立位置 (rad)
};
```

#### Response (Jetson → ODroid)
```cpp
struct _msg_response {
    float q_exp[10];        // 期望关节位置 (rad)
    float dq_exp[10];       // 期望关节速度 (rad/s) - 预留
    float tau_exp[10];      // 期望力矩 (Nm) - 预留
};
```

## 系统要求

### 硬件
- **推荐**: NVIDIA Jetson Nano/TX2/Xavier/Orin
- **最低**: 支持CUDA的ARM64平台
- **内存**: ≥4GB RAM

### 软件依赖
- **操作系统**: Ubuntu 18.04/20.04 (ARM64)
- **编译器**: GCC 7.5+, CMake 3.25+
- **CUDA**: 10.2+ (JetPack自带)
- **PyTorch**: 1.x (C++ LibTorch)
- **LCM**: 1.4+ (可选，用于仿真测试)

## 安装步骤

### 1. 安装LibTorch (C++)

```bash
# 下载适用于Jetson的LibTorch预编译包
# 注意：需要与Python PyTorch版本匹配
pip3 install torch torchvision

# 找到torch安装路径
python3 -c "import torch; print(torch.__path__)"

# 在CMakeLists.txt中更新路径
# set(Torch_DIR "/home/jetson/.local/lib/python3.6/site-packages/torch/share/cmake/Torch")
```

### 2. 安装CUDA (如果未安装)

```bash
# Jetson平台通常随JetPack自带CUDA
# 检查CUDA版本
nvcc --version
```

### 3. 安装LCM (可选，用于仿真测试)

```bash
sudo apt-get update
sudo apt-get install -y build-essential libglib2.0-dev
cd /tmp
git clone https://github.com/lcm-proj/lcm.git
cd lcm
mkdir build && cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
```

## 编译

```bash
# 激活anaconda环境
source ~/archiconda3/bin/activate

# 进入lcm环境
conda activate lcm

# 进入相应文件夹
cd sim2sim_lcm
mkdir -p build && cd build

# 配置（请根据实际路径修改CMakeLists.txt中的LibTorch路径）
cmake ..

# 编译
make -j4
```

## 运行

### 1. 准备模型文件

确保 `model_jitt.pt` 在项目根目录，或修改源码中的模型路径：

```cpp
// 在 udp_publisher_tinker.cpp 中修改
model_path = "/path/to/your/model_jitt.pt";
```

### 2. 配置网络参数

编辑 `src/udp_publisher_tinker.cpp` 中的IP和端口：

```cpp
string UDP_IP = "192.168.1.11";  // ODroid的IP地址
int SERV_PORT = 10000;           // UDP端口
```

### 3. 运行推理程序

```bash
cd build
./udp_publisher_tinker
```

**预期输出**:
```
RL model thread start
cuda_is_available: 1
cudnn_is_available: 1
load model is successed!
LibTorch Version: 1.x.x
load model to device!
Thread UDP RL-Tinker

act send: -0.16 0.68 1.3 0.16 0.68 1.3 -0.16 0.68 1.3 0.16
...
```

### 4. 调试模式

修改代码中的 `#if 0` 为 `#if 1` 可以启用调试输出：

```cpp
#if 1  // 启用调试输出
    cout << "act send: ";
    for(int i = 0; i < 10; i++)
        cout << msg_response.q_exp[i] << " ";
    cout << endl;
#endif
```

## 配置参数

在 `include/udp_publish_tinker.h` 中可以调整以下参数：

```cpp
// 历史观测长度
int history_length = 10;

// 初始站立姿态（双足机器人10个关节）
// 格式: [左腿5个, 右腿5个]
float init_pos[10] = {
    0.0, -0.07, 0.57, -1.12, 0.56,   // 左腿: Yaw, Roll, Pitch, Knee, Ankle
    0.0,  0.07, 0.57, -1.12, 0.56    // 右腿: Yaw, Roll, Pitch, Knee, Ankle
};

// 观测数据缩放系数
float eu_ang_scale = 1.0;      // 姿态角缩放
float omega_scale = 0.25;      // 角速度缩放
float pos_scale = 1.0;         // 位置缩放
float vel_scale = 0.05;        // 速度缩放
float lin_vel = 2.0;           // 线速度命令缩放
float ang_vel = 0.25;          // 角速度命令缩放

// 指令平滑
float smooth = 0.03;           // 平滑系数
float dead_zone = 0.01;        // 死区阈值
```

## 关节映射

**10关节双足机器人配置**:

| 索引 | 关节 | 电机型号 | 左腿 | 右腿 |
|------|------|----------|------|------|
| 0/5  | Yaw  | DM6006   | L_YAW | R_YAW |
| 1/6  | Roll | DM4340   | L_ROLL | R_ROLL |
| 2/7  | Pitch| DM8006   | L_PITCH | R_PITCH |
| 3/8  | Knee | DM6006   | L_KNEE | R_KNEE |
| 4/9  | Ankle| DM6006   | L_ANKLE | R_ANKLE |

## 通信协议

### UDP通信模式
- **类型**: 全双工
- **协议**: UDP (无连接)
- **频率**: ~500Hz
- **超时**: 2ms

### 数据流
```
ODroid ---(Request)---> Jetson
  ↑                        ↓
  +-------(Response)-------+
```

## 故障排查

### 1. 编译错误

**问题**: `Could not find Torch`
```bash
# 解决方法: 检查LibTorch路径
python3 -c "import torch; print(torch.__path__)"
# 更新CMakeLists.txt中的Torch_DIR
```

**问题**: CUDA相关错误
```bash
# 检查CUDA安装
nvcc --version
# 更新CMakeLists.txt中的CUDA_TOOLKIT_ROOT_DIR
```

### 2. 运行时错误

**问题**: `Failed to load model`
```bash
# 确保模型文件存在
ls -lh model_jitt.pt
# 检查模型是否为TorchScript格式（.pt或.pth）
```

**问题**: UDP通信失败
```bash
# 检查网络连接
ping 192.168.1.11  # ODroid的IP
# 检查防火墙设置
sudo ufw allow 10000/udp
```

**问题**: NaN检测触发
```
NaN detected in obs. Press any key to continue...
```
- 检查ODroid发送的观测数据是否有效
- 检查IMU数据是否正常
- 检查关节角度是否在合理范围

### 3. 性能问题

**问题**: 推理延迟过高
- 启用CUDA加速: 确保 `torch::cuda::is_available()` 返回true
- 使用FP16推理: 代码中已使用 `to(torch::kHalf)`
- 降低 `history_length` 参数
- 使用Jetson的性能模式: `sudo nvpmodel -m 0; sudo jetson_clocks`

## 开发指南

### 修改关节数量

如果需要适配不同的机器人配置（如12关节四足），修改以下位置：

1. **消息定义** (`include/udp_publish_tinker.h`):
```cpp
struct _msg_request {
    float q[12];    // 修改关节数量
    float dq[12];
    // ...
};
```

2. **观测维度** (`src/udp_publisher_tinker.cpp`):
```cpp
torch::Tensor obs_tensor = torch::from_blob(obs.data(), {1, 45}, options);
// 45 = 3(omega) + 3(euler) + 3(cmd) + 12(q) + 12(dq) + 12(last_action)
```

3. **初始化缓冲区**:
```cpp
action_buf = torch::zeros({history_length, 12}, device);
obs_buf = torch::zeros({history_length, 45}, device);
```

### 添加自定义传感器

在 `handleMessage()` 函数中添加新的观测数据：

```cpp
// 示例: 添加足端力传感器
obs.push_back(request.force_left * force_scale);
obs.push_back(request.force_right * force_scale);
```

## 相关项目

- [F405_Driver_CLion](../F405_Driver_CLion) - STM32电机驱动固件
- [ODroid](../ODroid) - ODroid-C4运控上位机程序

## 许可证

请参考项目根目录的LICENSE文件。

## 致谢

- Unitree Robotics - 数学工具库参考
- PyTorch团队 - LibTorch C++ API

## 联系方式

如有问题或建议，请提交Issue或Pull Request。

---

**最后更新**: 2026-02-03
