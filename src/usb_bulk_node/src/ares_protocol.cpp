#include "ares_protocol.hpp"
#include <cstring>
#include <iostream> // 用于调试输出
#include <arpa/inet.h> // 包含网络字节序转换函数 htons, ntohs, htonl, ntohl
#include <vector> // 需要包含 vector
// {{ edit_1: 确保包含时间戳所需的头文件 }}
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <stdexcept> // 如果需要抛出异常

namespace ares {

// {{ edit_2: 重新添加 get_current_timestamp 函数实现 }}
// 辅助函数：获取当前时间戳
std::string Protocol::get_current_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    // 使用北京时间 (UTC+8)
    // 注意：std::put_time 和 std::localtime 不是线程安全的，但在一个线程内调用通常没问题
    // 如果在多线程环境中使用，需要加锁或使用线程安全的替代方案
    struct tm tm_buf; // POSIX 兼容的缓冲区
    time_t tt = now_c;
    // 使用 localtime_r (POSIX 线程安全版本) 或 localtime_s (Windows 线程安全版本)
    // 这里为了跨平台简单性，暂时使用非线程安全的 localtime，但在多线程日志记录中应注意
    #ifdef _WIN32
        struct tm ptm_s;
        localtime_s(&ptm_s, &tt);
        struct tm* ptm = &ptm_s;
    #else
        struct tm* ptm = localtime_r(&tt, &tm_buf); // POSIX
    #endif

    if (ptm) {
         ss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
         ss << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
    } else {
        ss << "TimestampError";
    }
    return ss.str();
}


Protocol::Protocol() = default; // 构造函数

Protocol::~Protocol() { // 析构函数确保断开连接
    disconnect();
}

bool Protocol::connect() {
    if (dev_handle_) { // 已经连接
        return true;
    }

    if(stop_) {
        return false;
    }

    int r = libusb_init(&usb_ctx_);
    if (r < 0) {
        std::cerr << "libusb_init Error: " << libusb_error_name(r) << std::endl;
        return false;
    }

    // libusb_set_option(usb_ctx_, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG); // 可选：开启调试日志

    dev_handle_ = libusb_open_device_with_vid_pid(usb_ctx_, VID, PID);
    if (!dev_handle_) {
        std::cerr << "Error finding/opening device VID=" << std::hex << VID << " PID=" << PID << std::dec << std::endl;
        libusb_exit(usb_ctx_);
        usb_ctx_ = nullptr;
        return false;
    }

    // 尝试为设备解绑内核驱动（如果需要）
    libusb_detach_kernel_driver(dev_handle_, 0); // 忽略错误，可能不需要

    r = libusb_claim_interface(dev_handle_, 0); // 假设使用接口 0
    if (r < 0) {
        std::cerr << "libusb_claim_interface Error: " << libusb_error_name(r) << std::endl;
        libusb_close(dev_handle_);
        dev_handle_ = nullptr;
        libusb_exit(usb_ctx_);
        usb_ctx_ = nullptr;
        return false;
    }

    std::cout << "USB Device connected successfully." << std::endl;
    running_ = true;
    read_thread_ = std::thread(&Protocol::usb_read_loop, this); // 启动读取线程
    heartbeat_thread_ = std::thread(&Protocol::heartbeat_loop, this); // 新增：启动心跳线程

    return true;
}

void Protocol::disconnect() {
    if (!dev_handle_ && !running_) { // 检查 running_ 避免重复执行
        return;
    }

    running_ = false; // 通知所有线程停止
    stop_ = true;

    // 等待读取线程结束
    if (read_thread_.joinable()) {
        // 可能需要取消正在进行的 bulk transfer 以便线程快速退出
        // libusb_cancel_transfer(...); // 如果有进行中的 transfer
        read_thread_.join();
    }
    // 新增：等待心跳线程结束
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }

    // 只有在设备句柄有效时才释放接口和关闭设备
    if (dev_handle_) {
        libusb_release_interface(dev_handle_, 0);
        libusb_close(dev_handle_);
        dev_handle_ = nullptr;
    }
    // 只有在 USB 上下文有效时才退出
    if (usb_ctx_) {
        libusb_exit(usb_ctx_);
        usb_ctx_ = nullptr;
    }
    std::cout << "USB Device disconnected." << std::endl;
}

bool Protocol::is_connected() const {
    return dev_handle_ != nullptr && running_;
}

void Protocol::register_sync_callback(SyncCallback cb) {
    sync_cb_ = std::move(cb);
}

void Protocol::register_exec_callback(ExecCallback cb) {
    exec_cb_ = std::move(cb);
}

// {{ edit_1: 实现新的回调注册函数 }}
void Protocol::register_exec_reply_callback(ExecReplyCallback cb) {
    exec_reply_cb_ = std::move(cb);
}

void Protocol::register_error_callback(ErrorCallback cb) {
    error_cb_ = std::move(cb);
}


// 移除 set_send_function 实现

// 内部 USB 写函数
bool Protocol::usb_write(const uint8_t* data, size_t len) {
    if (!is_connected()) return false;

    int actual_length = 0;
    int r;
    { // 锁住 USB 写操作，防止多线程冲突
        std::lock_guard<std::mutex> lock(usb_mutex_);
        if (!dev_handle_) return false; // 再次检查，防止 disconnect 发生在判断之后
        r = libusb_bulk_transfer(dev_handle_, EP_OUT, const_cast<uint8_t*>(data), len, &actual_length, USB_TIMEOUT_MS);
    } // 锁在此处释放

    if (r == 0 && actual_length == static_cast<int>(len)) {
        return true;
    } else {
        std::cerr << "USB Write Error: " << libusb_error_name(r) << ", transferred: " << actual_length << "/" << len << std::endl;
        // 可以考虑在这里处理断线等错误，例如调用 disconnect()
        // if (r == LIBUSB_ERROR_NO_DEVICE || r == LIBUSB_ERROR_IO) {
        //     disconnect(); // 示例：发生严重错误时断开连接
        // }
        return false;
    }
}

// 内部 USB 读线程循环
void Protocol::usb_read_loop() {
    uint8_t buffer[USB_FS_MPS]; // 使用最大包大小作为缓冲区
read_begin:
    while (running_) {
        int actual_length = 0;
        int r = libusb_bulk_transfer(dev_handle_, EP_IN, buffer, sizeof(buffer), &actual_length, 100); // 短超时，避免阻塞 disconnect
        // std::cout << get_current_timestamp() << " USB read loop: r=" << r << ", actual_length=" << actual_length << std::endl;

        if (r == 0 && actual_length > 0) {
            on_receive_internal(buffer, actual_length);
        } else if (r != LIBUSB_ERROR_TIMEOUT && r != LIBUSB_TRANSFER_CANCELLED) {
            // 忽略超时和取消错误，其他错误则打印并可能停止
            std::cerr << "USB Read Error: " << libusb_error_name(r) << std::endl;
            if (r == LIBUSB_ERROR_NO_DEVICE || r == LIBUSB_ERROR_IO || r == LIBUSB_ERROR_PIPE) {
                 std::cerr << "USB connection lost. Stopping read loop." << std::endl;
                 running_ = false; // 停止循环
                 // 可能需要通知主线程或尝试自动重连
            }
        }
        // 超时或无数据时继续循环
    }
    while(!running_ && !stop_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if(!stop_) {
        goto read_begin;
    }
    std::cout << "USB read loop finished." << std::endl;
}


// 修改 on_receive 为内部方法 on_receive_internal
// --- 内部数据处理 ---
void Protocol::on_receive_internal(const uint8_t* buf, size_t len) {
    if (len < 2) { // 至少需要包含帧头
        // 现在可以调用 get_current_timestamp 了
        std::cerr << get_current_timestamp() << " Error: Received runt frame. Len=" << len << std::endl;
        return;
    }

    // 网络字节序转主机字节序获取帧头
    uint16_t head = ntohs(*(reinterpret_cast<const uint16_t*>(buf)));

    // {{ edit_1: 移除或注释掉通用的接收打印，避免刷屏 }}
    // std::cout << get_current_timestamp() << " Received frame: Head=0x" << std::hex << head << ", Len=" << std::dec << len << std::endl;

    switch (head) {
        case SYNC_FRAME_HEAD: { // 0x5A5A
            if (len >= offsetof(SyncFrame, data)) { // 至少包含头部和 data_id
                const SyncFrame* frame = reinterpret_cast<const SyncFrame*>(buf);
                uint16_t data_id = (frame->data_id);
                size_t data_len = len - offsetof(SyncFrame, data);

                // Sync 帧不检查 request_id，直接调用回调
                if (sync_cb_) {
                    try {
                        sync_cb_(data_id, frame->data, data_len);
                        // std::cout << get_current_timestamp() << " Sync data received: DataID=" << data_id << ", Len=" << data_len << std::endl;
                    } catch (const std::exception& e) {
                        std::cerr << get_current_timestamp() << " Error in sync_cb_: " << e.what() << std::endl; // OK
                    } catch (...) {
                        std::cerr << get_current_timestamp() << " Unknown error in sync_cb_" << std::endl; // OK
                    }
                }
            } else {
                std::cerr << get_current_timestamp() << " Error: Received truncated Sync frame. Len=" << len << std::endl; // OK
            }
            break;
        }
        case EXEC_FRAME_HEAD: { // 0xCAFE (设备端接收)
             if (len == sizeof(ExecFrame)) {
                const ExecFrame* frame = reinterpret_cast<const ExecFrame*>(buf);
                uint16_t func_id = ntohs(frame->func_id);

                // {{ edit_2: 检查 request_id 是否为心跳 ID，如果是则忽略 }}
                if (frame->request_id != HEARTBEAT_REQUEST_ID) {
                    if (exec_cb_) {
                        try {
                            uint32_t result = exec_cb_(func_id, frame->arg1, frame->arg2, frame->arg3, frame->request_id);
                            // 可以在这里添加发送 ExecReplyFrame 的逻辑
                            // send_exec_reply(func_id, result, frame->request_id);
                        } catch (const std::exception& e) {
                             std::cerr << get_current_timestamp() << " Error in exec_cb_: " << e.what() << std::endl; // OK
                        } catch (...) {
                             std::cerr << get_current_timestamp() << " Unknown error in exec_cb_" << std::endl; // OK
                        }
                    } else {
                        // std::cout << get_current_timestamp() << " Warning: Received Exec frame but no callback registered." << std::endl;
                        // 没有回调，可以考虑发送一个 "未知函数" 或 "未实现" 的错误帧
                    }
                } else {
                    // 收到的是心跳请求 (ExecFrame with ReqID=0xFF)，内部可能已处理或忽略
                    // std::cout << get_current_timestamp() << " Info: Ignored Exec frame with heartbeat request ID (0xFF)." << std::endl;
                }
            } else {
                std::cerr << get_current_timestamp() << " Error: Received Exec frame with incorrect size. Len=" << len << std::endl; // OK
            }
            break;
        }
        case EXEC_REPLY_HEAD: { // 0xC0DE (主机端接收)
             if (len == sizeof(ExecReplyFrame)) {
                 const ExecReplyFrame* frame = reinterpret_cast<const ExecReplyFrame*>(buf);
                 if (frame->request_id != HEARTBEAT_REQUEST_ID) {
                     // {{ edit_2: 调用 ExecReply 回调 }}
                     if (exec_reply_cb_) {
                         try {
                             exec_reply_cb_(ntohs(frame->func_id), frame->value, frame->request_id);
                         } catch (const std::exception& e) {
                             std::cerr << get_current_timestamp() << " Error in exec_reply_cb_: " << e.what() << std::endl;
                         } catch (...) {
                             std::cerr << get_current_timestamp() << " Unknown error in exec_reply_cb_" << std::endl;
                         }
                     } else {
                        // 可选：打印未处理的回复
                        // std::cout << get_current_timestamp() << " Received Exec Reply but no callback registered. ReqID=" << (int)frame->request_id << std::endl;
                     }
                 } else {
                     // 忽略心跳相关的回复
                 }
             } else {
                 std::cerr << get_current_timestamp() << " Error: Received Exec Reply frame with incorrect size. Len=" << len << std::endl;
             }
             break;
        }
        case ERROR_FRAME_HEAD: { // 0xCADE
            if (len == sizeof(ErrorFrame)) {
                const ErrorFrame* frame = reinterpret_cast<const ErrorFrame*>(buf);
                if (frame->request_id != HEARTBEAT_REQUEST_ID) {
                    // {{ edit_3: 调用 Error 回调 }}
                    if (error_cb_) {
                         try {
                             error_cb_(frame->request_id, ntohs(frame->error_code));
                         } catch (const std::exception& e) {
                             std::cerr << get_current_timestamp() << " Error in error_cb_: " << e.what() << std::endl;
                         } catch (...) {
                             std::cerr << get_current_timestamp() << " Unknown error in error_cb_" << std::endl;
                         }
                    } else {
                        // 可选：打印未处理的错误
                        // std::cerr << get_current_timestamp() << " Received Error Frame but no callback registered. ReqID=" << (int)frame->request_id << ", Code=0x" << std::hex << ntohs(frame->error_code) << std::dec << std::endl;
                    }
                } else {
                    // 忽略心跳相关的错误
                }
            } else {
                 std::cerr << get_current_timestamp() << " Error: Received Error frame with incorrect size. Len=" << len << std::endl;
            }
            break;
        }
        default:
            std::cerr << get_current_timestamp() << " Error: Received frame with unknown head: 0x" << std::hex << head << std::dec << std::endl; // OK
            break;
    }
}

// 修改 send_exec 使用内部 USB 写并调整字节序转换
bool Protocol::send_exec(uint16_t func_id, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint8_t request_id) {
    ExecFrame frame_to_send; // 存储要发送的帧
    frame_to_send.head = htons(EXEC_FRAME_HEAD); // 转换 head
    frame_to_send.func_id = htons(func_id);     // 转换 func_id
    frame_to_send.arg1 = arg1; // arg1 不再转换
    frame_to_send.arg2 = arg2; // arg2 不再转换
    frame_to_send.arg3 = arg3; // arg3 不再转换
    frame_to_send.request_id = request_id;
    frame_to_send.reserved = 0;
    return usb_write(reinterpret_cast<uint8_t*>(&frame_to_send), sizeof(frame_to_send));
}

// 修改 send_sync 方法 (下位机使用) 并添加字节序转换 (此函数逻辑不变，因为只转换 head 和 data_id)
bool Protocol::send_sync(uint16_t data_id, const uint8_t* data, size_t len) {
    constexpr size_t sync_header_size = offsetof(ares::SyncFrame, data);
    if (len > (USB_FS_MPS - sync_header_size)) { // 检查是否超过最大包大小减去头部
        std::cerr << "Error: Sync data too large (" << len << " > " << (USB_FS_MPS - sync_header_size) << ")" << std::endl;
        return false;
    }
    // 使用一个足够大的缓冲区来构造帧
    std::vector<uint8_t> buffer(sync_header_size + len);

    // 填充头部并转换字节序
    uint16_t net_head = htons(SYNC_FRAME_HEAD);
    uint16_t net_data_id = htons(data_id);
    memcpy(buffer.data() + offsetof(SyncFrame, head), &net_head, sizeof(uint16_t));
    memcpy(buffer.data() + offsetof(SyncFrame, data_id), &net_data_id, sizeof(uint16_t));

    // 复制数据负载
    memcpy(buffer.data() + sync_header_size, data, len);

    // 发送实际构造的帧大小
    return usb_write(buffer.data(), buffer.size());
}


// ... (移除 get_sync_data_buffer 实现) ...

// 新增：心跳线程循环函数实现
void Protocol::heartbeat_loop() {
heartbeat_begin:
    while (running_) {
        // 构造心跳错误帧
        ErrorFrame heartbeat_frame;
        heartbeat_frame.head = htons(ERROR_FRAME_HEAD); // 转换 head
        heartbeat_frame.request_id = HEARTBEAT_REQUEST_ID; // 单字节无需转换
        heartbeat_frame.reserved = 0;
        heartbeat_frame.error_code = htons(HEARTBEAT_ERROR_CODE); // 转换 error_code

        // 发送心跳帧 (usb_write 内部有锁保护)
        if (is_connected()) { // 检查连接状态再发送
             if (!usb_write(reinterpret_cast<uint8_t*>(&heartbeat_frame), sizeof(heartbeat_frame))) {
                 std::cerr << "Failed to send heartbeat frame." << std::endl;
                 // 可以选择在这里处理发送失败，例如尝试重连或停止心跳
             }
        }


        // 等待指定间隔
        std::this_thread::sleep_for(HEARTBEAT_INTERVAL);

        if(stop_) {
            std::cerr << "Heartbeat loop stopped." << std::endl;
            return;
        }
    }
    while(!running_) {
reconnect_fail:

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(stop_) {
            return;
        }

        dev_handle_ = libusb_open_device_with_vid_pid(usb_ctx_, VID, PID);
        if (!dev_handle_) {
            goto reconnect_fail;
        }
    
        // 尝试为设备解绑内核驱动（如果需要）
        libusb_detach_kernel_driver(dev_handle_, 0); // 忽略错误，可能不需要
    
        int r = libusb_claim_interface(dev_handle_, 0); // 假设使用接口 0
        if (r < 0) {
            goto reconnect_fail;
        }
    
        std::cout << "USB Device connected successfully." << std::endl;
        running_ = true;
        break;
    }
    if(running_) {
        goto heartbeat_begin;
    }
}


} // namespace ares