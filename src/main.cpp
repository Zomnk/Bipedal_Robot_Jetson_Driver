/**
 * @file main.cpp
 * @brief Jetson RL推理程序主入口 - 改进版本
 * @author Refactored
 * @date 2026-02-05
 *
 * @details 本文件是Jetson RL部署程序的主入口。
 *          主要功能：
 *          1. 解析命令行参数
 *          2. 加载标定配置文件
 *          3. 初始化UDP通信
 *          4. 加载PyTorch模型
 *          5. 运行500Hz控制循环
 *
 * @note 使用方法:
 *       ./udp_publisher_tinker [--ip IP] [--port PORT] [--config FILE]
 */

#include "udp_publish_tinker.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include <csignal>
#include <cmath>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

using namespace std;

/*
 * ============================================================
 * 全局变量
 * ============================================================
 */

/// 运行标志，用于控制主循环
volatile bool g_running = true;

/// 全局RL对象
RL_Tinymal_UDP tinymal_rl;

/// 消息结构体
_msg_request msg_request;
_msg_response msg_response;

/**
 * @brief 信号处理函数
 *
 * @details 处理SIGINT(Ctrl+C)和SIGTERM信号，
 *          设置g_running为false以优雅退出主循环。
 *
 * @param sig 信号编号
 */
void signalHandler(int sig) {
    cout << "\n收到信号 " << sig << ", 正在关闭..." << endl;
    g_running = false;
}

/**
 * @brief 打印使用说明
 *
 * @param prog 程序名称（argv[0]）
 */
void printUsage(const char* prog) {
    cout << "用法: " << prog << " [选项]" << endl;
    cout << "  --ip <IP>:    ODroid IP地址 (默认: 192.168.5.159)" << endl;
    cout << "  --port <N>:   UDP端口 (默认: 10000)" << endl;
    cout << "  --config <F>: 标定文件路径 (默认: ../robot.yaml)" << endl;
}

/**
 * @brief 从YAML文件加载标定数据
 *
 * @details 解析robot.yaml文件，提取10个关节的初始位置。
 *
 * @param filename YAML文件路径
 * @param init_pos 输出的初始位置数组（10个float）
 * @return 成功读取10个值返回true，否则返回false
 */
bool loadCalibration(const string& filename, float init_pos[10]) {
    ifstream f(filename);
    if (!f.is_open()) {
        cout << "警告: 无法打开标定文件 " << filename << endl;
        return false;
    }

    string line;
    int idx = 0;

    // 逐行解析YAML文件
    while (getline(f, line) && idx < 10) {
        // 查找冒号分隔符
        size_t pos = line.find(':');
        if (pos == string::npos) continue;

        // 提取冒号后的值
        string val = line.substr(pos + 1);

        // 移除注释部分
        size_t comment = val.find('#');
        if (comment != string::npos) val = val.substr(0, comment);

        // 跳过前导空白
        size_t start = val.find_first_not_of(" \t");
        if (start == string::npos) continue;

        // 尝试解析为浮点数
        try {
            init_pos[idx++] = stof(val.substr(start));
        } catch (...) {
            // 解析失败，跳过此行
        }
    }

    f.close();

    // 必须成功读取10个值
    if (idx == 10) {
        cout << "已加载标定文件: " << filename << endl;
        return true;
    } else {
        cout << "警告: 标定文件中只找到 " << idx << " 个值，使用默认值" << endl;
        return false;
    }
}

/**
 * @brief 缓慢移动机器人到初始姿态
 *
 * @details 使用线性插值将机器人从当前位置平滑移动到标定的初始姿态。
 *          这是一个安全措施，避免机器人突然跳到目标位置。
 *
 * @param sock_fd UDP socket文件描述符
 * @param addr_serv 目标地址结构体
 * @param init_pos 目标初始姿态（10个关节）
 * @param steps 插值步数（默认500步，约1秒）
 */
void moveToInitPose(int sock_fd, struct sockaddr_in& addr_serv, const float init_pos[10], int steps = 500) {
    cout << "正在移动到标定的初始姿态..." << endl;

    memset(&msg_response, 0, sizeof(msg_response));

    // 当前位置（从反馈获取）
    float current_pos[10] = {0};
    bool got_feedback = false;

    // ========== 步骤1: 获取当前位置 ==========
    // 尝试最多50次（约500ms）获取当前关节位置
    for (int i = 0; i < 50 && !got_feedback; i++) {
        // 发送零位置指令
        char send_buf[500] = {0};
        memcpy(send_buf, &msg_response, sizeof(msg_response));
        sendto(sock_fd, send_buf, sizeof(msg_response), MSG_WAITALL,
               (struct sockaddr *)&addr_serv, sizeof(addr_serv));

        // 接收反馈
        char recv_buf[500] = {0};
        int recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_WAITALL,
                               (struct sockaddr *)&addr_serv, (socklen_t *)&sizeof(addr_serv));
        if (recv_num > 0) {
            memcpy(&msg_request, recv_buf, sizeof(msg_request));
            // 保存当前关节位置
            for (int j = 0; j < 10; j++) {
                current_pos[j] = msg_request.q[j];
            }
            got_feedback = true;
        }
        usleep(10000);  // 10ms
    }

    if (!got_feedback) {
        cout << "未收到反馈，使用零位作为起点" << endl;
    }

    // ========== 步骤2: 线性插值移动 ==========
    for (int step = 0; step <= steps && g_running; step++) {
        // 计算插值系数 (0.0 -> 1.0)
        float alpha = (float)step / steps;

        // 计算插值位置
        for (int i = 0; i < 10; i++) {
            msg_response.q_exp[i] = current_pos[i] + alpha * (init_pos[i] - current_pos[i]);
        }

        // 发送位置指令
        char send_buf[500] = {0};
        memcpy(send_buf, &msg_response, sizeof(msg_response));
        sendto(sock_fd, send_buf, sizeof(msg_response), MSG_WAITALL,
               (struct sockaddr *)&addr_serv, sizeof(addr_serv));

        // 接收反馈
        char recv_buf[500] = {0};
        recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_WAITALL,
                (struct sockaddr *)&addr_serv, (socklen_t *)&sizeof(addr_serv));

        usleep(2000);  // 2ms，500Hz
    }

    cout << "已到达初始姿态" << endl;
}

/**
 * @brief 数值限幅函数
 * @param input 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
float limit(float input, float min_val, float max_val) {
    if (input > max_val)
        return max_val;
    if (input < min_val)
        return min_val;
    return input;
}

/**
 * @brief 主函数
 *
 * @details 程序执行流程：
 *          1. 解析命令行参数
 *          2. 注册信号处理函数
 *          3. 加载标定配置
 *          4. 初始化UDP通信
 *          5. 加载PyTorch模型
 *          6. 移动到初始姿态
 *          7. 进入500Hz控制循环
 *          8. 收到退出信号后清理退出
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 正常退出返回0，错误返回1
 */
int main(int argc, char** argv) {
    // ========== 解析命令行参数 ==========
    string target_ip = "192.168.5.159";        // ODroid IP地址
    int port = 10000;                          // UDP端口
    string config_file = "../robot.yaml";      // 标定文件路径

    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) {
            target_ip = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
    }

    // ========== 注册信号处理函数 ==========
    signal(SIGINT, signalHandler);   // Ctrl+C
    signal(SIGTERM, signalHandler);  // kill命令

    // ========== 打印启动信息 ==========
    cout << "========================================" << endl;
    cout << "  Jetson RL 推理程序 (LibTorch)" << endl;
    cout << "  目标: " << target_ip << ":" << port << endl;
    cout << "========================================" << endl;

    // ========== 加载标定配置 ==========
    float calibrated_init_pos[10] = {0};
    // 使用默认值
    for (int i = 0; i < 10; i++) {
        calibrated_init_pos[i] = tinymal_rl.init_pos[i];
    }
    // 尝试从文件加载
    loadCalibration(config_file, calibrated_init_pos);

    // ========== 创建UDP socket ==========
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        cerr << "创建socket失败" << endl;
        return 1;
    }

    // ========== 配置目标地址（ODroid） ==========
    struct sockaddr_in addr_serv;
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr(target_ip.c_str());
    addr_serv.sin_port = htons(port);
    int len = sizeof(addr_serv);

    // ========== 初始化RL策略（加载模型） ==========
    cout << "正在初始化RL策略..." << endl;
    tinymal_rl.init_policy();

    // 初始化Response消息
    memset(&msg_response, 0, sizeof(msg_response));
    for (int i = 0; i < 10; i++) {
        msg_response.q_exp[i] = calibrated_init_pos[i];
    }

    cout << "========================================" << endl;
    cout << "消息大小: Request=" << sizeof(msg_request) << " bytes, "
         << "Response=" << sizeof(msg_response) << " bytes" << endl;
    cout << "========================================" << endl;

    // ========== 移动到初始姿态 ==========
    moveToInitPose(sock_fd, addr_serv, calibrated_init_pos);

    // ========== 开始控制循环 ==========
    cout << "========================================" << endl;
    cout << "开始控制循环 (500Hz)..." << endl;
    cout << "按 Ctrl+C 退出" << endl;
    cout << "========================================" << endl;

    int loop_count = 0;   // 循环计数
    int infer_count = 0;  // 推理计数
    char send_buf[500] = {0}, recv_buf[500] = {0};

    // ========== 主控制循环 ==========
    while (g_running) {
        // 发送上一次的响应
        memcpy(send_buf, &msg_response, sizeof(msg_response));
        int send_num = sendto(sock_fd, send_buf, sizeof(msg_response), MSG_WAITALL,
                             (struct sockaddr *)&addr_serv, len);

        if (send_num < 0) {
            perror("发送失败");
            break;
        }

        // 接收新的请求
        int recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_WAITALL,
                               (struct sockaddr *)&addr_serv, (socklen_t *)&len);

        if (recv_num > 0) {
            memcpy(&msg_request, recv_buf, sizeof(msg_request));

            // 调用推理处理
            tinymal_rl.handleMessage(msg_request);

            // ========== 检查推理是否成功 ==========
            if (tinymal_rl.action_refresh) {
                tinymal_rl.action_refresh = 0;

                // 检查输出中是否有NaN
                bool has_nan = false;
                for (int i = 0; i < 10; i++) {
                    if (isnan(tinymal_rl.action[i])) {
                        has_nan = true;
                        break;
                    }
                }

                // ========== 推理成功且无NaN时的处理 ==========
                if (!has_nan) {
                    // 使用推理结果
                    for (int i = 0; i < 10; i++) {
                        msg_response.q_exp[i] = limit(tinymal_rl.action[i], -15, 15);
                    }
                    infer_count++;

                    // 每0.5秒打印一次状态（250次 × 2ms = 500ms）
                    if (infer_count % 250 == 0) {
                        // 获取观测向量
                        float obs[39];
                        tinymal_rl.getLastObservation(obs);

                        cout << "[推理 #" << infer_count << "] 观测向量 (39维):" << endl;
                        cout << "  角速度(0-2):     ";
                        for (int i = 0; i < 3; i++) {
                            cout.precision(4);
                            cout << fixed << obs[i] << " ";
                        }
                        cout << endl;

                        cout << "  欧拉角(3-5):     ";
                        for (int i = 3; i < 6; i++) {
                            cout.precision(4);
                            cout << fixed << obs[i] << " ";
                        }
                        cout << endl;

                        cout << "  控制指令(6-8):   ";
                        for (int i = 6; i < 9; i++) {
                            cout.precision(4);
                            cout << fixed << obs[i] << " ";
                        }
                        cout << endl;

                        cout << "  关节位置偏差(9-18): ";
                        for (int i = 9; i < 19; i++) {
                            cout.precision(4);
                            cout << fixed << obs[i] << " ";
                        }
                        cout << endl;

                        cout << "  关节速度(19-28):  ";
                        for (int i = 19; i < 29; i++) {
                            cout.precision(4);
                            cout << fixed << obs[i] << " ";
                        }
                        cout << endl;

                        cout << "  上次动作(29-38):  ";
                        for (int i = 29; i < 39; i++) {
                            cout.precision(4);
                            cout << fixed << obs[i] << " ";
                        }
                        cout << endl;

                        cout << "  推理动作 (10维):  ";
                        for (int i = 0; i < 10; i++) {
                            cout.precision(4);
                            cout << fixed << msg_response.q_exp[i] << " ";
                        }
                        cout << endl << endl;
                    }
                } else {
                    // 推理输出有NaN，终止推理并回到初始姿态
                    cout << "\n[错误] 推理输出包含NaN，终止推理" << endl;
                    cout << "正在回到初始姿态..." << endl;
                    moveToInitPose(sock_fd, addr_serv, calibrated_init_pos);

                    // 重新初始化response
                    memset(&msg_response, 0, sizeof(msg_response));
                    for (int i = 0; i < 10; i++) {
                        msg_response.q_exp[i] = calibrated_init_pos[i];
                    }

                    cout << "已回到初始姿态，重新开始控制循环" << endl;
                    cout << "========================================" << endl;
                }
            }
        }

        loop_count++;
        usleep(2000);  // 2ms，500Hz
    }

    // ========== 关闭socket ==========
    close(sock_fd);

    // ========== 打印退出信息 ==========
    cout << "\n========================================" << endl;
    cout << "程序已退出" << endl;
    cout << "总循环次数: " << loop_count << endl;
    cout << "总推理次数: " << infer_count << endl;
    cout << "========================================" << endl;

    return 0;
}
