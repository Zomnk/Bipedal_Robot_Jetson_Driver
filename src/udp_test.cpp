/**
 * @file udp_test.cpp
 * @brief Jetson端UDP通信测试程序（不涉及推理）
 * @author Zomnk
 * @date 2026-02-03
 * 
 * @note 用于测试Jetson与ODroid之间的UDP通信是否正常
 *       不加载模型，不进行推理，只验证消息收发
 */

#include <iostream>
#include <iomanip>
#include <cstring>
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

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    cout << "========================================" << endl;
    cout << "  Jetson UDP通信测试程序" << endl;
    cout << "  测试目标: 验证UDP消息收发" << endl;
    cout << "========================================" << endl;
    
    // 创建UDP socket
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0) {
        cerr << "创建socket失败!" << endl;
        return 1;
    }
    
    // 设置socket接收超时（避免阻塞）
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms超时
    if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        cerr << "设置socket超时失败!" << endl;
    }
    
    // 解析命令行参数
    string ODROID_IP = "192.168.5.159";  // ODroid IP
    int SERV_PORT = 10000;
    
    // 支持命令行参数修改IP和端口
    if (argc >= 2) {
        ODROID_IP = argv[1];
    }
    if (argc >= 3) {
        SERV_PORT = atoi(argv[2]);
    }
    
    // ===== 关键修复：Jetson端必须先bind本地端口 =====
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;  // 监听所有网卡
    local_addr.sin_port = htons(SERV_PORT);   // 绑定10000端口
    
    if (bind(sock_fd, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        cerr << "Bind端口失败! 请检查端口" << SERV_PORT << "是否被占用" << endl;
        perror("bind");
        close(sock_fd);
        return 1;
    }
    
    cout << "本地监听: 0.0.0.0:" << SERV_PORT << endl;
    cout << "目标IP: " << ODROID_IP << endl;
    cout << "端口: " << SERV_PORT << endl;
    cout << "Request消息大小: " << sizeof(_msg_request) << " bytes" << endl;
    cout << "Response消息大小: " << sizeof(_msg_response) << " bytes" << endl;
    cout << "========================================" << endl;
    cout << "开始UDP通信测试，按Ctrl+C退出" << endl;
    cout << "等待ODroid发送数据..." << endl;
    cout << "========================================" << endl;
    
    _msg_request msg_request;
    _msg_response msg_response;
    char send_buf[500] = {0};
    char recv_buf[500] = {0};
    
    // 初始化响应消息（使用固定测试值）
    for(int i = 0; i < 10; i++) {
        msg_response.q_exp[i] = (float)(i + 1) * 0.1f;  // 0.1, 0.2, ..., 1.0
        msg_response.dq_exp[i] = 0.0f;
        msg_response.tau_exp[i] = 0.0f;
    }
    
    int send_count = 0;
    int recv_count = 0;
    int loop_count = 0;
    
    // ODroid的地址（首次从recvfrom获取，之后复用）
    struct sockaddr_in odroid_addr;
    socklen_t odroid_addr_len = sizeof(odroid_addr);
    bool has_client = false;
    
    while (g_running) {
        // 先接收Request消息（等待ODroid发送）
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
                cout << "收到ODroid连接: " << client_ip << ":" 
                     << ntohs(odroid_addr.sin_port) << endl;
            }
            
            // 立即回复Response消息
            memcpy(send_buf, &msg_response, sizeof(msg_response));
            int send_num = sendto(sock_fd, send_buf, sizeof(msg_response), 
                                 0, (struct sockaddr *)&odroid_addr, odroid_addr_len);
            
            if(send_num > 0) {
                send_count++;
            } else if(!g_running) {
                break;
            }
            
            // 每250次接收（0.5秒）打印一次详细信息
            if(recv_count % 250 == 0) {
                cout << "\n[统计 #" << recv_count << "]" << endl;
                cout << "  发送: " << send_count << " 包" << endl;
                cout << "  接收: " << recv_count << " 包" << endl;
                cout << "  丢包率: " << fixed << setprecision(2) 
                     << (1.0 - (float)recv_count / send_count) * 100 << "%" << endl;
                
                cout << "\n[接收到的Request]" << endl;
                cout << "  trigger: " << msg_request.trigger << endl;
                cout << "  command: [" << msg_request.command[0] << ", "
                     << msg_request.command[1] << ", "
                     << msg_request.command[2] << ", "
                     << msg_request.command[3] << "]" << endl;
                cout << "  姿态(eu_ang): [" << msg_request.eu_ang[0] << ", "
                     << msg_request.eu_ang[1] << ", "
                     << msg_request.eu_ang[2] << "]" << endl;
                cout << "  角速度(omega): [" << msg_request.omega[0] << ", "
                     << msg_request.omega[1] << ", "
                     << msg_request.omega[2] << "]" << endl;
                cout << "  加速度(acc): [" << msg_request.acc[0] << ", "
                     << msg_request.acc[1] << ", "
                     << msg_request.acc[2] << "]" << endl;
                cout << "  关节位置(q[0-4]): [" << msg_request.q[0] << ", "
                     << msg_request.q[1] << ", "
                     << msg_request.q[2] << ", "
                     << msg_request.q[3] << ", "
                     << msg_request.q[4] << "]" << endl;
                cout << "  关节速度(dq[0-4]): [" << msg_request.dq[0] << ", "
                     << msg_request.dq[1] << ", "
                     << msg_request.dq[2] << ", "
                     << msg_request.dq[3] << ", "
                     << msg_request.dq[4] << "]" << endl;
                
                cout << "\n[发送的Response]" << endl;
                cout << "  q_exp[0-4]: [" << msg_response.q_exp[0] << ", "
                     << msg_response.q_exp[1] << ", "
                     << msg_response.q_exp[2] << ", "
                     << msg_response.q_exp[3] << ", "
                     << msg_response.q_exp[4] << "]" << endl;
                cout << "  q_exp[5-9]: [" << msg_response.q_exp[5] << ", "
                     << msg_response.q_exp[6] << ", "
                     << msg_response.q_exp[7] << ", "
                     << msg_response.q_exp[8] << ", "
                     << msg_response.q_exp[9] << "]" << endl;
                cout << "========================================" << endl;
            }
        } else if(errno == EAGAIN || errno == EWOULDBLOCK) {
            // 超时，继续等待
        } else if(!g_running) {
            break;
        }
        
        loop_count++;
    }
    
    cout << "\n\n========================================" << endl;
    cout << "测试结束" << endl;
    cout << "总循环: " << loop_count << endl;
    cout << "发送包: " << send_count << endl;
    cout << "接收包: " << recv_count << endl;
    if(send_count > 0) {
        cout << "丢包率: " << fixed << setprecision(2) 
             << (1.0 - (float)recv_count / send_count) * 100 << "%" << endl;
    }
    cout << "========================================" << endl;
    
    close(sock_fd);
    return 0;
}
