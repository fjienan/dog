#include "ares_protocol.hpp"
#include <iostream>
#include <vector>
#include <chrono> // for sleep
#include <thread> // for sleep
#include <limits> // Required for numeric_limits
#include <ios>    // Required for streamsize

// --- 回调函数示例 ---

// 处理同步帧的回调 (假设这是上位机接收)
void sync_handler(uint16_t data_id, const uint8_t* data, size_t len) {
    std::cout << "[Callback] Sync received: DataID=" << data_id << ", Len=" << len << ", Data=";
    for(size_t i = 0; i < len && i < 8; ++i) { // 最多打印前8字节
         printf("%02X ", data[i]);
    }
    if (len > 8) std::cout << "...";
    std::cout << std::endl;
}

// 处理执行帧的回调 (假设这是下位机接收并处理)
uint32_t exec_handler(uint16_t func_id, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint8_t request_id) {
    std::cout << "[Callback] Exec received: FuncID=" << func_id
              << ", Arg1=" << arg1 << ", Arg2=" << arg2 << ", Arg3=" << arg3
              << ", ReqID=" << (int)request_id << std::endl;
    // 模拟处理并返回结果
    return arg1 + arg2 + arg3;
}

// 移除旧的 send_func
// void send_func(const uint8_t* buf, size_t len) { ... }

int main() {
    ares::Protocol proto;

    // 注册回调 (根据你的应用是上位机还是下位机选择注册)
    // 如果是上位机，主要注册 sync_cb_ 和处理返回帧的回调 (需要添加)
    proto.register_sync_callback(sync_handler);
    // 如果是下位机，主要注册 exec_cb_
    // proto.register_exec_callback(exec_handler);

    // 连接设备
    std::cout << "Connecting to device..." << std::endl;
    if (!proto.connect()) {
        std::cerr << "Failed to connect." << std::endl;
        return 1;
    }
    std::cout << "Device connected: " << proto.is_connected() << std::endl;
    std::cout << "Continuously reading data... Press Enter to disconnect and exit." << std::endl;


    // --- 示例：上位机发送执行命令 (可以按需保留或移除) ---
    // std::cout << "Sending Exec command (FuncID=0x1)..." << std::endl;
    // if (!proto.send_exec(0x1, 100, 200, 300, 0x01)) {
    //      std::cerr << "Failed to send Exec command." << std::endl;
    // }
    // std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等待一下

    // --- 示例：下位机发送同步数据 (可以按需保留或移除) ---
    // uint8_t sync_data[] = {0xDE, 0xAD, 0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,0xDE, 0xAD, 0xBE, 0xEF,};
    // std::cout << "Sending Sync data (DataID=0x2048)..." << std::endl;
    // if (!proto.send_sync(0x2048, sync_data, sizeof(sync_data))) {
    //     std::cerr << "Failed to send Sync data." << std::endl;
    // }


    // 持续运行，等待用户输入退出
    // 清除输入缓冲区的任何剩余字符
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    // 等待用户按 Enter 键
    std::cin.get(); // <--- 这里会一直阻塞，直到按下 Enter

    // 断开连接
    std::cout << "Disconnecting..." << std::endl;
    proto.disconnect();

    std::cout << "Program finished." << std::endl;
    return 0;
}