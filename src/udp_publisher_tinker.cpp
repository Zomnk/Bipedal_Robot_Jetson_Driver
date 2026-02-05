
/**
 * @file udp_publisher_tinker.cpp
 * @brief Jetson端RL推理程序 - 10关节双足机器人控制
 * @author Original Team
 * @date 2026-02-03
 * 
 * @note 功能说明：
 *       1. 通过UDP接收ODroid发送的39维观测数据（Request消息）
 *       2. 使用PyTorch JIT模型进行强化学习推理
 *       3. 输出10维动作指令发送给ODroid（Response消息）
 *       4. 支持CUDA加速，使用半精度（FP16）推理
 * 
 * @note 数据流：
 *       STM32 -> ODroid -> [UDP] -> Jetson(本程序) -> [推理] -> ODroid -> STM32
 * 
 * @note 观测维度：39维
 *       - 角速度(3) + 欧拉角(3) + 控制指令(3) + 关节位置(10) + 关节速度(10) + 上次动作(10)
 * 
 * @note 动作维度：10维
 *       - 左腿5关节 + 右腿5关节 (Yaw, Roll, Pitch, Knee, Ankle)
 */

#include "udp_publish_tinker.h"
/**
 * @brief 数值限幅函数
 * @param input 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
float limit(float input, float min, float max) {
    if(input > max)
        return max;
    if(input < min)
        return min;
    return input;
} 


/**
 * @brief 处理接收到的观测数据并进行RL推理
 * @param request 从ODroid接收的观测数据
 * 
 * @note 处理流程：
 *       1. 检查trigger标志（是否需要推理）
 *       2. 构造39维观测向量
 *       3. 历史观测缓存更新
 *       4. 模型前向推理
 *       5. 动作滤波（0.8*new + 0.2*old）
 *       6. 输出10维关节动作
 * 
 * @note 观测向量构成（39维）：
 *       [0-2]   角速度 omega (rad/s) * omega_scale
 *       [3-5]   欧拉角 eu_ang (rad) * eu_ang_scale
 *       [6-8]   控制指令 command (vx, vy, yaw_rate)
 *       [9-18]  关节位置偏差 (q - init_pos) * pos_scale
 *       [19-28] 关节速度 dq * vel_scale
 *       [29-38] 上次动作 last_action
 */
void RL_Tinymal_UDP::handleMessage(_msg_request request)
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <sys/shm.h>
#include <arpa/inet.h>
#include <time.h>

{              
    // ===== Python代码参考 =====
    // obs[0, 0:3]   = omega * cfg.normalization.obs_scales.ang_vel
    // obs[0, 3:6]   = eu_ang * cfg.normalization.obs_scales.quat
    // obs[0, 6:9]   = cmd (vx, vy, dyaw) * cfg.normalization.obs_scales
    // obs[0, 9:21]  = (q - default_dof_pos) * cfg.normalization.obs_scales.dof_pos
    // obs[0, 21:33] = dq * cfg.normalization.obs_scales.dof_vel
    // obs[0, 33:45] = last_actions
    
    #if 0  // 调试打印（默认关闭）
        cout<<"cmd:";
        cout<<request.command[0]<<" ";
        cout<<request.command[1]<<" ";
        cout<<request.command[2]<<" ";
        cout<<endl;
        cout<<"att:";
        cout<<request.eu_ang[0]<<" ";
        cout<<request.eu_ang[1]<<" ";
        cout<<request.eu_ang[2]<<" ";
        cout<<endl;
        cout<<"rate:";
        cout<<request.omega[0]<<" ";
        cout<<request.omega[1]<<" ";
        cout<<request.omega[2]<<" ";
        cout<<endl;
        cout<<"q:";
        for(int i=0;i<12;i++)
            cout<<request.q[i]<<" ";
        cout<<endl;
        cout<<"dq:";
        for(int i=0;i<12;i++)
            cout<<request.dq[i]<<" ";
        cout<<endl;    
        cout<<"trigger:"<<request.trigger<<" ";
    #endif
    
    // ===== 1. 检查trigger标志，判断是否需要进行推理 =====
    if(request.trigger == 1) {
        request.trigger = 0;
        
        // 更新初始位置（用于计算关节位置偏差）
        for(int i = 0; i < 10; i++)
            init_pos[i] = request.init_pos[i];
            
        // ===== 2. 构造观测向量（39维） =====
        std::vector<float> obs;
        
        // [0-2] 角速度（IMU角速度，rad/s）
        obs.push_back(request.omega[0] * omega_scale);
        obs.push_back(request.omega[1] * omega_scale);
        obs.push_back(request.omega[2] * omega_scale);

        // [3-5] 欧拉角姿态（Roll, Pitch, Yaw，rad）
        obs.push_back(request.eu_ang[0] * eu_ang_scale);
        obs.push_back(request.eu_ang[1] * eu_ang_scale);
        obs.push_back(request.eu_ang[2] * eu_ang_scale);

        // [6-8] 控制指令（vx, vy, yaw_rate）带死区和平滑滤波"q:";
        for(int i=0;i<12;i++)
            cout<<request.q[i]<<" ";
        cout<<endl;
        cout<<"dq:";
        for(int i=0;i<12;i++)
            cout<<request.dq[i]<<" ";
        cout<<endl;    
        cout<<"trigger:"<<request.trigger<<" ";
        
    #endif
    // 将 data 转为 tensor 类型，输入到模型
    if(request.trigger==1){
        request.trigger=0;
        std::vector<float> obs;
        for(int i=0;i<10;i++)
            init_pos[i]=request.init_pos[i];
            
        //---------------Push data into obsbuf--------------------
        obs.push_back(request.omega[0]*omega_scale);
        obs.push_back(request.omega[1]*omega_scale);
        obs.push_back(request.omega[2]*omega_scale);

        obs.push_back(request.eu_ang[0]*eu_ang_scale);
        obs.push_back(request.eu_ang[1]*eu_ang_scale);
        obs.push_back(request.eu_ang[2]*eu_ang_scale);

        // [6-8] 控制指令（vx, vy, yaw_rate）带死区和平滑滤波
        float max = 1.0;
        float min = -1.0;
        
        // 一阶低通滤波：new = old * (1-smooth) + input * smooth
        // 死区处理：小于dead_zone的输入视为0
        cmd_x = cmd_x * (1 - smooth) + (std::fabs(request.command[0]) < dead_zone ? 0.0 : request.command[0]) * smooth;
        cmd_y = cmd_y * (1 - smooth) + (std::fabs(request.command[1]) < dead_zone ? 0.0 : request.command[1]) * smooth;
        cmd_rate = cmd_rate * (1 - smooth) + (std::fabs(request.command[2]) < dead_zone ? 0.0 : request.command[2]) * smooth;

        obs.push_back(cmd_x * lin_vel);      // X方向线速度指令
        obs.push_back(cmd_y * lin_vel);      // Y方向线速度指令
        obs.push_back(cmd_rate * ang_vel);   // Yaw角速度指令

        // [9-18] 关节位置偏差（当前位置 - 初始位置）
        for (int i = 0; i < 10; ++i)
        {
            float pos = (request.q[i] - init_pos[i]) * pos_scale;
            obs.push_back(pos);
        }
        
        // [19-28] 关节速度
        for (int i = 0; i < 10; ++i)
        {
            float vel = request.dq[i] * vel_scale;
            obs.push_back(vel);
        }
        
        // [29-38] 上次动作（历史动作，用于时序信息）
        for (int i = 0; i < 10; ++i)
        {
            obs.push_back(action_temp[i]);
        }

        // ===== 3. 将观测向量转换为Tensor =====
        auto options = torch::TensorOptions().dtype(torch::kFloat32);
        torch::Tensor obs_tensor = torch::from_blob(obs.data(), {1, 39}, options).to(device);
        
        // ===== 4. 准备模型输入（当前观测 + 历史观测缓存） =====
        auto obs_buf_batch = obs_buf.unsqueeze(0);  // 添加batch维度

        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(obs_tensor.to(torch::kHalf));      // 当前观测（FP16）
        inputs.push_back(obs_buf_batch.to(torch::kHalf));   // 历史观测（FP16）
        
        // ===== 5. 模型前向推理 =====
        torch::Tensor action_tensor = model.forward(inputs).toTensor();
        
        // 更新动作历史缓存（移除最旧的，添加最新的）
        // ===== 6. NaN检测（防止异常数据导致推理失败） =====
        bool has_nan = false;
        for (float val : obs) {
            if (std::isnan(val)) {
                has_nan = true;
            }
        }
        if (has_nan) {
            cout << "NaN detected in obs. Press any key to continue..." << endl;
            getchar();  // 暂停程序，等待人工检查
        }

        // ===== 7. 动作滤波（平滑输出，避免抖动） =====
        // 加权平均：80%当前动作 + 20%上次动作
        torch::Tensor action_blend_tensor = 0.8 * action_tensor + 0.2 * last_action;
        last_action = action_tensor.clone();  // 保存当前动作用于下次滤波
    
        // ===== 8. 更新观测历史缓存 =====
        // 移除最旧的观测，添加最新的观测（滑动窗口）
        this->obs_buf = torch::cat({this->obs_buf.index({Slice(1, None), Slice()}), obs_tensor}, 0);
        
        // ===== 9. 提取动作数据并转换到CPU =====
        torch::Tensor action_raw = action_blend_tensor.squeeze(0);  // 移除batch维度
        action_raw = action_raw.to(torch::kFloat32);  // FP16 -> FP32
        action_raw = action_raw.to(torch::kCPU);       // GPU -> CPU
        
        // ===== 10. 动作限幅并存储 =====
        auto action_getter = action_raw.accessor<float, 1>();
        for (int j = 0; j < 10; j++)
        {
            action[j] = limit(action_getter[j], -15, 15);       // 限制在[-15, 15]范围内
            action_temp[j] = limit(action_getter[j], -15, 15);  // 保存原始值用于下次观测
        }
        
        // 保存观测向量用于调试输出
        this->last_obs = obs;

        action_refresh = 1;  // 标记动作已更新，可以发送
    }
}

/**
 * @brief 获取最后一次的观测向量
 * @param obs 输出的观测数组（39维）
 */
void RL_Tinymal_UDP::getLastObservation(float* obs) {
    if (last_obs.size() == 39) {
        for (int i = 0; i < 39; i++) {
            obs[i] = last_obs[i];
        }
    } else {
        // 如果还没有观测数据，返回全零
        for (int i = 0; i < 39; i++) {
            obs[i] = 0.0f;
        }
    }
}

/**
 * @brief 加载PyTorch JIT模型
 * @return 0表示成功
 * 
 * @note 模型配置：
 *       - 优先使用CUDA加速（如果可用）
 *       - 使用半精度（FP16）推理以提高速度
 *       - 模型设置为评估模式（model.eval()）
 */
int RL_Tinymal_UDP::load_policy()
{   
    std::cout << model_path << std::endl;
    
    // ===== 1. 检测CUDA可用性 =====
    std::cout << "cuda::is_available():" << torch::cuda::is_available() << std::endl;
    device = torch::kCPU;
    if (torch::cuda::is_available() && 1) {
        device = torch::kCUDA;
        printf("device = torch::kCUDA\n");
    }
    std::cout << "device:" << device << endl;
    
    // ===== 2. 加载JIT模型 =====
    model = torch::jit::load(model_path);
    std::cout << "load model is successed!" << std::endl;
    
    // ===== 3. 模型配置 =====
    model.to(device);  // 将模型移动到GPU/CPU
    std::cout << "LibTorch Version: " << TORCH_VERSION_MAJOR << "." 
              << TORCH_VERSION_MINOR << "." 
              << TORCH_VERSION_PATCH << std::endl;
    
    model.to(torch::kHalf);  // 转换为FP16半精度
    std::cout << "load model to device!" << std::endl;
    
    model.eval();  // 设置为评估模式（关闭dropout等）
}

/**
 * @brief 初始化RL策略（加载模型、初始化缓存）
 * @return 0表示成功
 * 
 * @note 初始化内容：
 *       1. 加载PyTorch模型
 *       2. 初始化历史观测缓存（obs_buf）
 *       3. 初始化历史动作缓存（action_buf）
 *       4. 预热模型
 */
int RL_Tinymal_UDP::init_policy() {
    // ===== 1. 打印环境信息 =====
    std::cout << "RL model thread start" << endl;
    cout << "cuda_is_available:" << torch::cuda::is_available() << endl;
    cout << "cudnn_is_available:" << torch::cuda::cudnn_is_available() << endl;
    
    // ===== 2. 设置模型路径 =====
    // 使用相对路径（相对于build目录）
    model_path = "../model_jitt.pt";
    // 或使用绝对路径（根据实际部署位置修改）：
    // model_path = "/home/jetson/sim2sim_lcm/model_jitt.pt";
    
    cout << "Loading model from: " << model_path << endl;
    load_policy();  // 加载模型

    // ===== 3. 初始化历史缓存 =====
    action_buf = torch::zeros({history_length, 10}, device);   // 动作历史缓存
    obs_buf = torch::zeros({history_length, 39}, device);      // 观测历史缓存
    last_action = torch::zeros({1, 10}, device);               // 上次动作（用于滤波）

    // 转换为FP16
    action_buf.to(torch::kHalf);
    obs_buf.to(torch::kHalf);
    last_action.to(torch::kHalf);

    // ===== 4. 初始化动作向量 =====
    for (int j = 0; j < 10; j++)
    {
        action_temp.push_back(0.0);        // 临时动作缓存
        action.push_back(init_pos[j]);     // 当前动作（初始化为init_pos）
        prev_action.push_back(init_pos[j]); // 前一次动作
    }
    
    // ===== 5. 预热：填充历史观测缓存 =====
    for (int i = 0; i < history_length; i++)
    {
        std::vector<float> obs;
        
        // 角速度（全零）
        obs.push_back(0);
        obs.push_back(0);
        obs.push_back(0);

        // 欧拉角（全零）
        obs.push_back(0);
        obs.push_back(0);
        obs.push_back(0);

        // 控制指令（全零）
        obs.push_back(0);
        obs.push_back(0);
        obs.push_back(0);

        // 关节位置偏差（全零）
        for (int i = 0; i < 10; ++i)
        {
            float pos = 0;
            obs.push_back(pos);
            action[i] = init_pos[i];
        }
        
        // 关节速度（全零）
        for (int i = 0; i < 10; ++i)
        {
            float vel = 0;
            obs.push_back(vel);
        }
        
        // 上次动作（全零）
        for (int i = 0; i < 10; ++i)
        {
            obs.push_back(0);
        }
        
        auto options = torch::TensorOptions().dtype(torch::kFloat32);
        torch::Tensor obs_tensor = torch::from_blob(obs.data(), {1, 39}, options).to(device);
    }

    // ===== 6. 预热推理（可选，用于预加载CUDA kernels） =====
    for (int i = 0; i < 200; i++)
    {
        // tinymal_rl.model_infer();  // 可选的预热推理
    }
}

/**
 * @brief 主函数 - UDP通信主循环
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 0表示正常退出
 * 
 * @note 通信流程（500Hz循环）：
 *       1. 发送Response（动作指令）给ODroid
 *       2. 接收Request（观测数据）从ODroid
 *       3. 调用handleMessage进行推理
 *       4. 更新msg_response用于下次发送
 */
int main(int argc, char** argv) {
    // ===== 1. 创建UDP socket =====
    int sock_fd;
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        exit(1);
    }

    // ===== 2. 配置目标地址（ODroid） =====
    struct sockaddr_in addr_serv;
    int len;
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    
#if 0
    string UDP_IP = "127.0.0.1";  // 本地测试
    int SERV_PORT = 8888;
#else
    string UDP_IP = "192.168.5.159";  // ODroid IP
    int SERV_PORT = 10000;
#endif
    
    addr_serv.sin_addr.s_addr = inet_addr(UDP_IP.c_str());
    addr_serv.sin_port = htons(SERV_PORT);
    len = sizeof(addr_serv);

    int recv_num = 0, send_num = 0;
    int connect = 0, loss_cnt = 0;
    char send_buf[500] = {0}, recv_buf[500] = {0};

    // ===== 3. 初始化RL策略（加载模型） =====
    tinymal_rl.init_policy();
    
    // 初始化Response消息
    for(int i = 0; i < 10; i++)
        msg_response.q_exp[i] = tinymal_rl.action[i];
        
    printf("======================================\n");
    printf("Thread UDP RL-Tinker\n");
    printf("目标IP: %s, 端口: %d\n", UDP_IP.c_str(), SERV_PORT);
    printf("消息大小: Request=%zu bytes, Response=%zu bytes\n", 
           sizeof(msg_request), sizeof(msg_response));
    printf("开始UDP通信测试...\n");
    printf("======================================\n");
    
    int cnt_p = 0;
    int send_cnt = 0;
    
    // ===== 4. 主循环（500Hz，每2ms一次） =====
    while (1)
    {
        // ===== 4.1 更新Response消息（如果有新动作） =====
        if(tinymal_rl.action_refresh) {
            tinymal_rl.action_refresh = 0;
            
            for(int i = 0; i < 10; i++)
                msg_response.q_exp[i] = tinymal_rl.action[i];
                
            std::cout.precision(2);
            #if 1  // 打印发送的动作
                cout << endl;
                cout << "act send:";
                for(int i = 0; i < 10; i++)
                    cout << msg_response.q_exp[i] << " ";
                cout << endl;
            #endif
            cnt_p++;
        }
        
        // ===== 4.2 发送Response（动作指令）给ODroid =====
        memcpy(send_buf, &msg_response, sizeof(msg_response));
        send_num = sendto(sock_fd, send_buf, sizeof(msg_response), MSG_WAITALL, 
                         (struct sockaddr *)&addr_serv, len);
 
        if(send_num < 0)
        {
            perror("Robot sendto error:");
            exit(1);
        }
        
        send_cnt++;
        if(send_cnt % 250 == 0) {  // 每0.5秒打印一次统计
            cout << "[UDP发送 #" << send_cnt << "] 已发送 " << send_num << " 字节" << endl;
        }
        
        // ===== 4.3 接收Request（观测数据）从ODroid =====
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_WAITALL, 
                           (struct sockaddr *)&addr_serv, (socklen_t *)&len);
        if(recv_num > 0)
        {
            memcpy(&msg_request, recv_buf, sizeof(msg_request));
            
            // 打印接收到的观测（调试用）
            static int recv_cnt = 0;
            recv_cnt++;
            if(recv_cnt % 250 == 0) {  // 每0.5秒打印一次
                cout << "[UDP接收 #" << recv_cnt << "] ";
                cout << "trigger=" << msg_request.trigger << ", ";
                cout << "cmd=[" << msg_request.command[0] << "," 
                     << msg_request.command[1] << "," 
                     << msg_request.command[2] << "], ";
                cout << "姿态=[" << msg_request.eu_ang[0] << "," 
                     << msg_request.eu_ang[1] << "," 
                     << msg_request.eu_ang[2] << "], ";
                cout << "角速度=[" << msg_request.omega[0] << "," 
                     << msg_request.omega[1] << "," 
                     << msg_request.omega[2] << "]" << endl;
                cout << "         q[0-4]=[" << msg_request.q[0] << "," 
                     << msg_request.q[1] << "," 
                     << msg_request.q[2] << "," 
                     << msg_request.q[3] << "," 
                     << msg_request.q[4] << "]" << endl;
            }
            
            // ===== 4.4 调用推理处理 =====
            tinymal_rl.handleMessage(msg_request);
        }
        
        // ===== 4.5 延时2ms（维持500Hz频率） =====
        usleep(2 * 
            tinymal_rl.handleMessage(msg_request);
        }
        usleep(2*1000);
    }
    return 0;
}

