/**
 * @file calibrate_robot.cpp
 * @brief 双足机器人初始姿态标定工具
 * @author Zomnk
 * @date 2026-02-03
 * 
 * @note 功能说明：
 *       1. 交互式标定10个关节的初始站立位置
 *       2. 实时显示当前关节角度
 *       3. 保存标定结果到robot.yaml文件
 *       4. 支持单独标定某个关节或全部标定
 * 
 * @note 使用方法：
 *       ./calibrate_robot              # 标定所有关节
 *       ./calibrate_robot --joint 0    # 只标定指定关节
 */

#define _USE_MATH_DEFINES  // 必须在 cmath 之前定义以启用 M_PI
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>
#include <string>
#include <vector>

// 如果系统未定义 M_PI，手动定义
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

// 关节名称映射
const char* JOINT_NAMES[10] = {
    "左腿Yaw (L_YAW)",
    "左腿Roll (L_ROLL)",
    "左腿Pitch (L_PITCH)",
    "左腿Knee (L_KNEE)",
    "左腿Ankle (L_ANKLE)",
    "右腿Yaw (R_YAW)",
    "右腿Roll (R_ROLL)",
    "右腿Pitch (R_PITCH)",
    "右腿Knee (R_KNEE)",
    "右腿Ankle (R_ANKLE)"
};

// 全局变量
volatile bool g_running = true;
_msg_request latest_feedback;
bool feedback_received = false;
bool terminal_modified = false;  // 标记终端是否被修改

void signal_handler(int sig) {
    cout << "\n\n收到信号 " << sig << ", 退出标定..." << endl;
    g_running = false;
    
    // 恢复终端设置
    if (terminal_modified) {
        system("stty icanon echo");
        terminal_modified = false;
    }
}

/**
 * @brief 保存标定结果到YAML文件
 * @param init_pos 10个关节的初始位置
 * @param filename YAML文件路径
 */
bool save_to_yaml(const float init_pos[10], const string& filename) {
    ofstream yaml_file(filename);
    if (!yaml_file.is_open()) {
        cerr << "错误: 无法创建文件 " << filename << endl;
        return false;
    }
    
    yaml_file << "# 双足机器人初始姿态标定数据" << endl;
    yaml_file << "# 生成时间: " << __DATE__ << " " << __TIME__ << endl;
    yaml_file << "# 单位: 弧度 (rad)" << endl;
    yaml_file << endl;
    yaml_file << "robot_config:" << endl;
    yaml_file << "  init_pose:" << endl;
    
    // 左腿
    yaml_file << "    left_leg:" << endl;
    yaml_file << "      yaw:   " << fixed << setprecision(6) << init_pos[0] << "  # rad" << endl;
    yaml_file << "      roll:  " << init_pos[1] << "  # rad" << endl;
    yaml_file << "      pitch: " << init_pos[2] << "  # rad" << endl;
    yaml_file << "      knee:  " << init_pos[3] << "  # rad" << endl;
    yaml_file << "      ankle: " << init_pos[4] << "  # rad" << endl;
    
    // 右腿
    yaml_file << "    right_leg:" << endl;
    yaml_file << "      yaw:   " << init_pos[5] << "  # rad" << endl;
    yaml_file << "      roll:  " << init_pos[6] << "  # rad" << endl;
    yaml_file << "      pitch: " << init_pos[7] << "  # rad" << endl;
    yaml_file << "      knee:  " << init_pos[8] << "  # rad" << endl;
    yaml_file << "      ankle: " << init_pos[9] << "  # rad" << endl;
    
    // 备注说明
    yaml_file << endl;
    yaml_file << "  # 说明:" << endl;
    yaml_file << "  # - 此文件由 calibrate_robot 工具生成" << endl;
    yaml_file << "  # - 请勿手动修改，除非了解参数含义" << endl;
    yaml_file << "  # - 重新标定会覆盖此文件" << endl;
    
    yaml_file.close();
    return true;
}

/**
 * @brief 从ODroid接收反馈数据的线程函数
 */
void receive_feedback_loop(int sock_fd, struct sockaddr_in& odroid_addr, socklen_t& addr_len) {
    char recv_buf[500] = {0};
    
    while (g_running) {
        int recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 
                               0, (struct sockaddr *)&odroid_addr, &addr_len);
        
        if (recv_num > 0) {
            memcpy(&latest_feedback, recv_buf, sizeof(latest_feedback));
            feedback_received = true;
        }
        
        usleep(2000);  // 2ms
    }
}

/**
 * @brief 标定单个关节
 * @param joint_id 关节ID (0-9)
 * @param sock_fd UDP socket
 * @param odroid_addr ODroid地址
 * @param addr_len 地址长度
 * @return 标定的关节位置
 */
float calibrate_joint(int joint_id, int sock_fd, 
                     struct sockaddr_in& odroid_addr, socklen_t addr_len) {
    cout << "\n========================================" << endl;
    cout << "正在标定: " << JOINT_NAMES[joint_id] << " [ID=" << joint_id << "]" << endl;
    cout << "========================================" << endl;
    cout << "操作说明:" << endl;
    cout << "  1. 手动调整机器人到期望的初始站立姿态" << endl;
    cout << "  2. 观察下方显示的当前关节角度" << endl;
    cout << "  3. 确认姿态合适后，按 Enter 键保存" << endl;
    cout << "  4. 按 's' 跳过此关节（使用默认值0.0）" << endl;
    cout << "========================================" << endl;
    cout << "提示: 标定期间该关节扭矩已卸载，可手动调整" << endl;
    
    _msg_response msg_response;
    memset(&msg_response, 0, sizeof(msg_response));
    char send_buf[500] = {0};
    
    float current_angle = 0.0f;
    int update_count = 0;
    
    cout << "\n实时角度监测中... (按Enter确认, 按's'跳过)" << endl;
    cout << "-------------------------------------------" << endl;
    
    // 非阻塞输入设置
    system("stty -icanon -echo");  // 关闭行缓冲和回显
    terminal_modified = true;
    
    while (g_running) {
        // ===== 1. 先接收ODroid的反馈数据 =====
        char recv_buf[500] = {0};
        int recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 
                               0, (struct sockaddr *)&odroid_addr, &addr_len);
        
        if (recv_num > 0) {
            memcpy(&latest_feedback, recv_buf, sizeof(latest_feedback));
            feedback_received = true;
        }
        
        // ===== 2. 如果收到反馈，更新并发送控制指令 =====
        if (feedback_received) {
            current_angle = latest_feedback.q[joint_id];
            
            // 关键：将所有关节的目标位置设置为当前实际位置
            // 使用 dq_exp[0] = -999.0 作为标定模式标志（该字段为预留字段，安全）
            for (int i = 0; i < 10; i++) {
                msg_response.q_exp[i] = latest_feedback.q[i];
                msg_response.dq_exp[i] = 0.0f;
                msg_response.tau_exp[i] = 0.0f;
            }
            msg_response.dq_exp[0] = -999.0f;  // 标定模式标志（预留字段）
            
            // 更新发送缓冲
            memcpy(send_buf, &msg_response, sizeof(msg_response));
            
            // 发送控制指令（目标=当前位置，实现扭矩卸载）
            sendto(sock_fd, send_buf, sizeof(msg_response), 
                   0, (struct sockaddr *)&odroid_addr, addr_len);
            
            // 更新显示
            if (update_count % 10 == 0) {  // 每20ms更新一次显示
                cout << "\r当前角度: " << fixed << setprecision(4) << setw(8) 
                     << current_angle << " rad (" << setw(7) << setprecision(2)
                     << (current_angle * 180.0 / M_PI) << " deg)   " << flush;
            }
            update_count++;
        } else {
            // 如果没收到反馈，发送零指令保持连接
            memcpy(send_buf, &msg_response, sizeof(msg_response));
            sendto(sock_fd, send_buf, sizeof(msg_response), 
                   0, (struct sockaddr *)&odroid_addr, addr_len);
        }
        
        // 检查键盘输入
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = 1000;  // 1ms超时
        
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0) {
            char ch = getchar();
            if (ch == '\n') {
                // Enter键 - 确认标定
                break;
            } else if (ch == 's' || ch == 'S') {
                // 跳过此关节
                cout << "\n跳过标定，使用默认值 0.0 rad" << endl;
                current_angle = 0.0f;
                break;
            }
        }
        
        usleep(2000);  // 2ms
    }
    
    // 恢复终端设置
    system("stty icanon echo");
    terminal_modified = false;
    
    // 如果是因为信号退出，直接返回当前值
    if (!g_running) {
        cout << "\n标定被中断" << endl;
        return current_angle;
    }
    
    cout << "\n✓ 标定完成: " << current_angle << " rad" << endl;
    return current_angle;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    cout << "========================================" << endl;
    cout << "   双足机器人初始姿态标定工具" << endl;
    cout << "========================================" << endl;
    
    // 解析命令行参数
    int target_joint = -1;  // -1表示标定所有关节
    string yaml_filename = "../robot.yaml";  // 默认保存路径
    
    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--joint" && i + 1 < argc) {
            target_joint = atoi(argv[++i]);
            if (target_joint < 0 || target_joint > 9) {
                cerr << "错误: 关节ID必须在0-9之间" << endl;
                return 1;
            }
        } else if (arg == "--output" && i + 1 < argc) {
            yaml_filename = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            cout << "使用方法:" << endl;
            cout << "  " << argv[0] << " [选项]" << endl;
            cout << "\n选项:" << endl;
            cout << "  --joint <ID>      只标定指定关节 (0-9)" << endl;
            cout << "  --output <FILE>   指定输出文件 (默认: ../robot.yaml)" << endl;
            cout << "  --help, -h        显示此帮助信息" << endl;
            cout << "\n关节ID映射:" << endl;
            for (int i = 0; i < 10; i++) {
                cout << "  " << i << " - " << JOINT_NAMES[i] << endl;
            }
            return 0;
        }
    }
    
    // 创建UDP socket
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        cerr << "创建socket失败!" << endl;
        return 1;
    }
    
    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    // 配置ODroid地址
    string ODROID_IP = "192.168.5.159";
    int SERV_PORT = 10000;
    
    struct sockaddr_in local_addr, odroid_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    memset(&odroid_addr, 0, sizeof(odroid_addr));
    
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(SERV_PORT);
    
    odroid_addr.sin_family = AF_INET;
    odroid_addr.sin_addr.s_addr = inet_addr(ODROID_IP.c_str());
    odroid_addr.sin_port = htons(SERV_PORT);
    
    // 绑定本地端口
    if (bind(sock_fd, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        cerr << "Bind端口失败!" << endl;
        close(sock_fd);
        return 1;
    }
    
    cout << "通信配置: " << ODROID_IP << ":" << SERV_PORT << endl;
    cout << "输出文件: " << yaml_filename << endl;
    
    if (target_joint >= 0) {
        cout << "标定模式: 单关节 [" << JOINT_NAMES[target_joint] << "]" << endl;
    } else {
        cout << "标定模式: 全部关节 (10个)" << endl;
    }
    
    cout << "========================================" << endl;
    cout << "\n等待与ODroid建立连接..." << endl;
    
    // 等待首次反馈
    socklen_t addr_len = sizeof(odroid_addr);
    while (!feedback_received && g_running) {
        _msg_response dummy_msg;
        memset(&dummy_msg, 0, sizeof(dummy_msg));
        sendto(sock_fd, &dummy_msg, sizeof(dummy_msg), 
               0, (struct sockaddr *)&odroid_addr, addr_len);
        
        char recv_buf[500];
        int recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 
                               0, (struct sockaddr *)&odroid_addr, &addr_len);
        if (recv_num > 0) {
            memcpy(&latest_feedback, recv_buf, sizeof(latest_feedback));
            feedback_received = true;
            cout << "✓ 已连接到ODroid，开始标定..." << endl;
        }
        usleep(100000);  // 100ms
    }
    
    if (!g_running) {
        close(sock_fd);
        return 0;
    }
    
    // 标定数组
    float init_pos[10] = {0};
    
    // 读取现有配置（如果存在）
    ifstream existing_yaml(yaml_filename);
    if (existing_yaml.is_open()) {
        cout << "\n检测到现有配置文件，将作为默认值..." << endl;
        // 简单解析（仅读取数值）
        string line;
        int idx = 0;
        while (getline(existing_yaml, line) && idx < 10) {
            size_t pos = line.find(':');
            if (pos != string::npos) {
                string value_str = line.substr(pos + 1);
                size_t comment_pos = value_str.find('#');
                if (comment_pos != string::npos) {
                    value_str = value_str.substr(0, comment_pos);
                }
                try {
                    float value = stof(value_str);
                    init_pos[idx++] = value;
                } catch (...) {}
            }
        }
        existing_yaml.close();
    }
    
    // 执行标定
    if (target_joint >= 0) {
        // 单关节标定
        init_pos[target_joint] = calibrate_joint(target_joint, sock_fd, odroid_addr, addr_len);
    } else {
        // 全部关节标定
        for (int i = 0; i < 10; i++) {
            if (!g_running) break;
            init_pos[i] = calibrate_joint(i, sock_fd, odroid_addr, addr_len);
            
            if (i < 9 && g_running) {
                cout << "\n按Enter继续标定下一个关节..." << endl;
                cin.ignore();  // 清除输入缓冲
                getchar();
            }
        }
    }
    
    if (g_running) {
        // 保存结果
        cout << "\n========================================" << endl;
        cout << "标定结果汇总:" << endl;
        cout << "========================================" << endl;
        
        for (int i = 0; i < 10; i++) {
            cout << "[" << i << "] " << setw(25) << left << JOINT_NAMES[i] 
                 << ": " << fixed << setprecision(4) << setw(8) << init_pos[i] 
                 << " rad (" << setprecision(2) << (init_pos[i] * 180.0 / M_PI) 
                 << " deg)" << endl;
        }
        
        cout << "========================================" << endl;
        cout << "\n保存到文件: " << yaml_filename << " ... ";
        
        if (save_to_yaml(init_pos, yaml_filename)) {
            cout << "✓ 成功!" << endl;
            cout << "\n标定完成！配置文件已生成。" << endl;
            cout << "现在可以使用 jetson_full_test 或其他程序读取此配置。" << endl;
        } else {
            cout << "✗ 失败!" << endl;
        }
    }
    
    // 最后确保终端设置已恢复
    if (terminal_modified) {
        system("stty icanon echo");
        terminal_modified = false;
    }
    
    close(sock_fd);
    return 0;
}
