# sim2sim_lcm 项目代码结构说明

## 代码组成结构

### 一、核心程序文件

#### 1. UDP通信主程序（推荐使用）
- **文件**: `src/udp_publisher_tinker.cpp` + `include/udp_publish_tinker.h`
- **功能**: 实时UDP服务器，与ODroid-C4进行双向通信
- **关节配置**: 10个关节（双足机器人）
- **特点**:
  - 支持CUDA加速推理
  - 历史观测缓冲（history_length=10）
  - 动作平滑滤波（0.8*new + 0.2*old）
  - 自动NaN检测

#### 2. UDP通信备用版本
- **文件**: `src/udp_publisher.cpp` + `include/udp_publish.h`
- **功能**: 12关节版本（可能用于四足或其他配置）
- **观测维度**: 45维

#### 3. LCM通信程序（仿真测试用）
- **文件**: 
  - `src/lcm_service.cpp` + `include/lcm_service.h` - LCM服务端
  - `src/lcm_publisher.cpp` + `include/lcm_publish.h` - LCM发布端
  - `src/lcm_client.py` - Python客户端示例
- **用途**: 离线仿真、测试环境下的消息通信

### 二、消息定义

#### Request消息（观测数据: ODroid → Jetson）
```cpp
struct _msg_request {
    float trigger;          // 触发标志
    float command[4];       // 用户控制指令 [vx, vy, yaw_rate, ...]
    float eu_ang[3];        // 欧拉角 (rad)
    float omega[3];         // 角速度 (rad/s)
    float acc[3];           // 加速度 (m/s^2)
    float q[10];            // 关节位置 (rad)
    float dq[10];           // 关节速度 (rad/s)
    float tau[10];          // 关节力矩 (Nm)
    float init_pos[10];     // 初始站立位置
};
```

#### Response消息（动作数据: Jetson → ODroid）
```cpp
struct _msg_response {
    float q_exp[10];        // 期望关节位置 (rad)
    float dq_exp[10];       // 期望关节速度 (rad/s) - 预留
    float tau_exp[10];      // 期望力矩 (Nm) - 预留
};
```

#### LCM消息定义（用于仿真）
- **文件**: `lcm_types/request.lcm`, `lcm_types/response.lcm`
- **生成代码**: `lcm_types/my_lcm/` (自动生成的C++/Python绑定)

### 三、工具库

#### 数学工具
- `include/mathTools.h` - 数学函数库（饱和、归一化、滤波等）
- `include/mathTypes.h` - 数学类型定义（向量、矩阵）
- `include/LowPassFilter.h` - 低通滤波器实现

#### 其他工具
- `include/enumClass.h` - 枚举类型定义

### 四、模型文件

- `model_jitt.pt` - PyTorch JIT编译的强化学习模型（主模型）
- `modelt.pt` - 备用模型

## 代码工作流程

### 主程序执行流程（udp_publisher_tinker）

```
启动
  ↓
1. 初始化RL策略（init_policy）
   - 加载PyTorch模型
   - 检测CUDA可用性
   - 初始化历史观测缓冲区（10×39维）
   - 初始化动作缓冲区（10×10维）
   ↓
2. 创建UDP Socket
   - 绑定IP: 192.168.1.11（ODroid）
   - 端口: 10000
   ↓
3. 主循环（~500Hz）
   ├─ 发送动作数据（Response）
   │   └─ 发送期望关节位置 q_exp[10]
   ├─ 接收观测数据（Request）
   │   ├─ IMU数据：姿态、角速度、加速度
   │   ├─ 关节数据：位置、速度、力矩
   │   └─ 控制指令：vx, vy, yaw_rate
   ↓
4. 数据处理（handleMessage）
   ├─ 更新初始位置（init_pos）
   ├─ 构建观测向量（39维）
   │   ├─ 角速度 × 3 (scaled by 0.25)
   │   ├─ 欧拉角 × 3 (scaled by 1.0)
   │   ├─ 指令平滑 × 3 (scaled by 2.0/0.25)
   │   ├─ 关节位置偏差 × 10 (q - init_pos)
   │   ├─ 关节速度 × 10 (scaled by 0.05)
   │   └─ 上次动作 × 10
   ↓
5. 神经网络推理
   ├─ 输入: obs_tensor (1×39), obs_buf (1×10×39)
   ├─ 模型前向传播
   ├─ 输出: action_tensor (1×10)
   └─ 动作平滑: 0.8*new + 0.2*old
   ↓
6. 动作后处理
   ├─ 类型转换: FP16 → FP32
   ├─ 设备转移: GPU → CPU
   ├─ 限幅: [-15, 15]
   └─ 更新 action_refresh 标志
   ↓
返回步骤3（循环）
```

## 关键数据维度

### 观测空间（39维）
```
Index  | Content              | Size | Description
-------|----------------------|------|------------------
0-2    | omega (scaled)       | 3    | 角速度 × 0.25
3-5    | eu_ang (scaled)      | 3    | 欧拉角 × 1.0
6-8    | command (smoothed)   | 3    | 平滑后的控制指令
9-18   | q - init_pos         | 10   | 关节位置偏差
19-28  | dq (scaled)          | 10   | 关节速度 × 0.05
29-38  | last_action          | 10   | 上一时刻动作
```

### 历史缓冲
- **观测历史**: `obs_buf` (10×39维)
- **动作历史**: `action_buf` (10×10维)

### 动作空间（10维）
```
Index | Joint      | Range      | Motor
------|------------|------------|--------
0     | L_YAW      | [-15, 15]  | DM6006
1     | L_ROLL     | [-15, 15]  | DM4340
2     | L_PITCH    | [-15, 15]  | DM8006
3     | L_KNEE     | [-15, 15]  | DM6006
4     | L_ANKLE    | [-15, 15]  | DM6006
5     | R_YAW      | [-15, 15]  | DM6006
6     | R_ROLL     | [-15, 15]  | DM4340
7     | R_PITCH    | [-15, 15]  | DM8006
8     | R_KNEE     | [-15, 15]  | DM6006
9     | R_ANKLE    | [-15, 15]  | DM6006
```

## 网络架构（模型）

模型输入：
- 当前观测: (1, 39)
- 历史观测: (1, 10, 39)

模型输出：
- 动作: (1, 10)

## 编译目标

```bash
make udp_publisher_tinker   # 主程序（10关节，推荐）
make udp_publisher          # 备用版本（12关节）
make lcm_service            # LCM服务端（仿真测试）
make lcm_publisher          # LCM发布端（仿真测试）
```

## 配置要点

### 关键参数（udp_publish_tinker.h）
```cpp
history_length = 10;         // 历史观测长度
init_pos[10] = {...};        // 初始站立姿态
omega_scale = 0.25;          // 角速度缩放
pos_scale = 1.0;             // 位置缩放
vel_scale = 0.05;            // 速度缩放
lin_vel = 2.0;               // 线速度命令缩放
ang_vel = 0.25;              // 角速度命令缩放
smooth = 0.03;               // 指令平滑系数
dead_zone = 0.01;            // 死区阈值
```

### 网络配置
```cpp
UDP_IP = "192.168.1.11";     // ODroid的IP
SERV_PORT = 10000;           // UDP端口
```

### 模型路径
```cpp
model_path = "/path/to/model_jitt.pt";
```

## 调试技巧

1. **启用详细输出**: 修改 `#if 0` 为 `#if 1`
2. **检查CUDA**: 观察启动时的 `cuda_is_available` 输出
3. **监控NaN**: 代码内置NaN检测，会暂停并提示
4. **验证通信**: 启用调试输出查看发送/接收数据

## 与其他模块的接口

### STM32 ← SPI → ODroid ← UDP → Jetson

- **STM32**: 电机实际控制，SPI从机
- **ODroid**: 协议转换中枢
  - SPI接收STM32的传感器数据
  - UDP发送观测数据给Jetson
  - UDP接收Jetson的动作指令
  - SPI发送控制指令给STM32
- **Jetson**: RL策略推理
  - UDP接收观测数据
  - 模型推理计算动作
  - UDP发送动作数据

---

**文档版本**: 1.0  
**最后更新**: 2026-02-03
