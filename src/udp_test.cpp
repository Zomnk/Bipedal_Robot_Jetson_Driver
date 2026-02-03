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
    
    // 配置目标地址（ODroid）
    struct sockaddr_in addr_serv;
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    
    string UDP_IP = "192.168.5.159";  // ODroid IP
    int SERV_PORT = 10000;
    
    // 支持命令行参数修改IP和端口
    if (argc >= 2) {
        UDP_IP = argv[1];
    }
    if (argc >= 3) {
        SERV_PORT = atoi(argv[2]);
    }
    
    addr_serv.sin_addr.s_addr = inet_addr(UDP_IP.c_str());
    addr_serv.sin_port = htons(SERV_PORT);
    int len = sizeof(addr_serv);
    
    cout << "目标IP: " << UDP_IP << endl;
    cout << "端口: " << SERV_PORT << endl;
    cout << "Request消息大小: " << sizeof(_msg_request) << " bytes" << endl;
    cout << "Response消息大小: " << sizeof(_msg_response) << " bytes" << endl;
    cout << "========================================" << endl;
    cout << "开始UDP通信测试，按Ctrl+C退出" << endl;
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
    
    while (g_running) {
        // 发送Response消息
        memcpy(send_buf, &msg_response, sizeof(msg_response));
        int send_num = sendto(sock_fd, send_buf, sizeof(msg_response), 
                             MSG_WAITALL, (struct sockaddr *)&addr_serv, len);
        
        if(send_num < 0) {
            perror("发送失败");
            break;
        }
        send_count++;
        
        // 接收Request消息
        int recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 
                               MSG_WAITALL, (struct sockaddr *)&addr_serv, 
                               (socklen_t *)&len);
        
        if(recv_num > 0) {
            memcpy(&msg_request, recv_buf, sizeof(msg_request));
            recv_count++;
            
            // 每250次循环（0.5秒）打印一次详细信息
            if(recv_count % 250 == 0) {
                cout << "\n[统计 #" << loop_count << "]" << endl;
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
        }
        
        loop_count++;
        
        // 每100次循环简单打印一次
        if(loop_count % 100 == 0) {
            cout << "." << flush;
        }
        
        usleep(2000);  // 2ms, 500Hz
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
