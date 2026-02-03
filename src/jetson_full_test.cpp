/**
 * @file jetson_full_test.cpp
 * @brief Jetson端完整数据流测试（不加载模型）
 * @author Zomnk
 * @date 2026-02-03
 * 
 * @note 测试功能：
 *       1. 生成正弦位置指令发送给ODroid（模拟RL输出）
 *       2. 接收ODroid的观测反馈并打印（验证数据流）
 *       3. 不加载模型，不进行推理，纯数据流测试
 */

#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>

using namespace std;

// 消息结构定义（与ODroid保持一致）
struct _msg_request {
    float trigger;
    float command[4];
    float eu_ang[3];
    float omega[3];
    float acc[3];
    float q[10];
    float dq[10];
    float tau[10];
    float init_pos[10];
};

struct _msg_response {
    float q_exp[10];
    float dq_exp[10];
    float tau_exp[10];
};

// 全局运行标志
volatile bool g_running = true;

void signal_handler(int sig) {
    cout << "\n收到信号 " << sig << ", 准备退出..." << endl;
    g_running = false;
}

// 打印观测信息（39维）
void print_observation(const _msg_request& obs, int count) {
    cout << "\n========== 观测信息 #" << count << " ==========" << endl;
    
    // 角速度 (3维)
    cout << "[角速度 omega] (rad/s)" << endl;
    cout << "  ω_x = " << fixed << setprecision(4) << obs.omega[0] << endl;
    cout << "  ω_y = " << obs.omega[1] << endl;
    cout << "  ω_z = " << obs.omega[2] << endl;
    
    // 欧拉角姿态 (3维)
    cout << "[欧拉角 eu_ang] (rad)" << endl;
    cout << "  Roll  = " << obs.eu_ang[0] << endl;
    cout << "  Pitch = " << obs.eu_ang[1] << endl;
    cout << "  Yaw   = " << obs.eu_ang[2] << endl;
    
    // 控制指令 (4维，测试用)
    cout << "[控制指令 command]" << endl;
    cout << "  vx = " << obs.command[0] << ", vy = " << obs.command[1] 
         << ", yaw_rate = " << obs.command[2] << ", reserved = " << obs.command[3] << endl;
    
    // 关节位置 (10维)
    cout << "[关节位置 q] (rad)" << endl;
    cout << "  左腿: [" << obs.q[0] << ", " << obs.q[1] << ", " 
         << obs.q[2] << ", " << obs.q[3] << ", " << obs.q[4] << "]" << endl;
    cout << "  右腿: [" << obs.q[5] << ", " << obs.q[6] << ", " 
         << obs.q[7] << ", " << obs.q[8] << ", " << obs.q[9] << "]" << endl;
    
    // 关节速度 (10维)
    cout << "[关节速度 dq] (rad/s)" << endl;
    cout << "  左腿: [" << obs.dq[0] << ", " << obs.dq[1] << ", " 
         << obs.dq[2] << ", " << obs.dq[3] << ", " << obs.dq[4] << "]" << endl;
    cout << "  右腿: [" << obs.dq[5] << ", " << obs.dq[6] << ", " 
         << obs.dq[7] << ", " << obs.dq[8] << ", " << obs.dq[9] << "]" << endl;
    
    // 上次动作 (10维)
    cout << "[上次动作 last_action] (rad)" << endl;
    cout << "  左腿: [" << obs.init_pos[0] << ", " << obs.init_pos[1] << ", " 
         << obs.init_pos[2] << ", " << obs.init_pos[3] << ", " << obs.init_pos[4] << "]" << endl;
    cout << "  右腿: [" << obs.init_pos[5] << ", " << obs.init_pos[6] << ", " 
         << obs.init_pos[7] << ", " << obs.init_pos[8] << ", " << obs.init_pos[9] << "]" << endl;
    
    // 加速度 (3维)
    cout << "[加速度 acc] (g)" << endl;
    cout << "  [" << obs.acc[0] << ", " << obs.acc[1] << ", " << obs.acc[2] << "]" << endl;
    
    // 力矩 (10维)
    cout << "[关节力矩 tau] (Nm)" << endl;
    cout << "  左腿: [" << obs.tau[0] << ", " << obs.tau[1] << ", " 
         << obs.tau[2] << ", " << obs.tau[3] << ", " << obs.tau[4] << "]" << endl;
    cout << "  右腿: [" << obs.tau[5] << ", " << obs.tau[6] << ", " 
         << obs.tau[7] << ", " << obs.tau[8] << ", " << obs.tau[9] << "]" << endl;
    
    cout << "======================================" << endl;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    cout << "========================================" << endl;
    cout << "  Jetson完整数据流测试" << endl;
    cout << "  功能: 发送正弦action，接收观测反馈" << endl;
    cout << "========================================" << endl;
    
    // 创建UDP socket
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0) {
        cerr << "创建socket失败!" << endl;
        return 1;
    }
    
    // 设置socket接收超时
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms超时
    if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        cerr << "设置socket超时失败!" << endl;
    }
    
    // 解析命令行参数
    string ODROID_IP = "192.168.5.159";
    int SERV_PORT = 10000;
    
    if (argc >= 2) {
        ODROID_IP = argv[1];
    }
    if (argc >= 3) {
        SERV_PORT = atoi(argv[2]);
    }
    
    // 绑定本地端口（监听ODroid的Request）
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(SERV_PORT);
    
    if (bind(sock_fd, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        cerr << "Bind端口失败! 请检查端口" << SERV_PORT << "是否被占用" << endl;
        perror("bind");
        close(sock_fd);
        return 1;
    }
    
    cout << "本地监听: 0.0.0.0:" << SERV_PORT << endl;
    cout << "目标ODroid IP: " << ODROID_IP << endl;
    cout << "Request消息大小: " << sizeof(_msg_request) << " bytes" << endl;
    cout << "Response消息大小: " << sizeof(_msg_response) << " bytes" << endl;
    cout << "========================================" << endl;
    cout << "正弦参数: 幅值=PI, 周期=4秒" << endl;
    cout << "========================================" << endl;
    cout << "开始完整数据流测试，按Ctrl+C退出" << endl;
    cout << "等待ODroid连接..." << endl;
    cout << "========================================" << endl;
    
    _msg_request msg_request;
    _msg_response msg_response;
    char send_buf[500] = {0};
    char recv_buf[500] = {0};
    
    // 正弦参数（与test_motor_control.cpp保持一致）
    const float amplitude = M_PI;        // 幅值为PI
    const float period_s = 4.0f;         // 周期4秒
    const float omega_sine = 2.0f * M_PI / period_s;  // 角频率
    
    int send_count = 0;
    int recv_count = 0;
    int loop_count = 0;
    
    struct sockaddr_in odroid_addr;
    socklen_t odroid_addr_len = sizeof(odroid_addr);
    bool has_client = false;
    
    uint64_t start_time_us = 0;
    struct timeval start_tv;
    gettimeofday(&start_tv, NULL);
    start_time_us = start_tv.tv_sec * 1000000ULL + start_tv.tv_usec;
    
    uint64_t last_print_time_us = start_time_us;
    
    while (g_running) {
        // ===== 1. 先接收ODroid的观测量 (Request消息) =====
        int recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 
                               0, (struct sockaddr *)&odroid_addr, 
                               &odroid_addr_len);
        
        if(recv_num > 0) {
            memcpy(&msg_request, recv_buf, sizeof(msg_request));
            recv_count++;
            
            // 首次收到数据，记录客户端地址
            if(!has_client) {
                has_client = true;
                char client_ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &odroid_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
                cout << "\n收到ODroid连接: " << client_ip << ":" 
                     << ntohs(odroid_addr.sin_port) << endl;
                cout << "开始数据交互..." << endl;
            }
        }
        
        // ===== 2. 获取当前时间，计算正弦位置 =====
        struct timeval now_tv;
        gettimeofday(&now_tv, NULL);
        uint64_t now_us = now_tv.tv_sec * 1000000ULL + now_tv.tv_usec;
        float elapsed_s = (now_us - start_time_us) / 1000000.0f;
        float target_position = amplitude * sin(omega_sine * elapsed_s);
        
        // 每500ms打印一次详细观测信息（包含当前目标位置）
        if (now_us - last_print_time_us >= 500000) {
            if(recv_count > 0) {
                print_observation(msg_request, recv_count);
            }
            cout << "[当前时间] t=" << fixed << setprecision(3) << elapsed_s 
                 << "s, 目标位置=" << target_position << " rad (" 
                 << (target_position * 180.0 / M_PI) << " deg)" << endl;
            last_print_time_us = now_us;
        }
        
        // ===== 3. 生成并发送正弦Action (Response消息) =====
        // 10个电机都使用相同的正弦位置（与test_motor_control.cpp一致）
        for (int i = 0; i < 10; i++) {
            msg_response.q_exp[i] = target_position;
            msg_response.dq_exp[i] = 0.0f;
            msg_response.tau_exp[i] = 0.0f;
        }
        
        if (has_client) {
            memcpy(send_buf, &msg_response, sizeof(msg_response));
            int send_num = sendto(sock_fd, send_buf, sizeof(msg_response), 
                                 0, (struct sockaddr *)&odroid_addr, odroid_addr_len);
            
            if(send_num > 0) {
                send_count++;
            }
        }
        
        loop_count++;
        
        // 每100个循环打印一个点（表示程序运行中）
        if(loop_count % 100 == 0) {
            cout << "." << flush;
        }
        
        usleep(2000);  // 2ms, 500Hz
    }
    
    cout << "\n\n========================================" << endl;
    cout << "测试结束" << endl;
    cout << "总循环: " << loop_count << endl;
    cout << "发送Action包: " << send_count << endl;
    cout << "接收Obs包: " << recv_count << endl;
    if(send_count > 0) {
        cout << "丢包率: " << fixed << setprecision(2) 
             << (1.0 - (float)recv_count / send_count) * 100 << "%" << endl;
    }
    cout << "========================================" << endl;
    
    close(sock_fd);
    return 0;
}
