#pragma once
#include <cstdint>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include "crc.hpp"

namespace ares_comm {

// 协议异常类型
class ProtocolException : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// 帧类型枚举
enum class FrameType {
    ExecuteFrame,    // 执行帧 0xCAFE
    ResponseFrame,   // 返回帧 0xC0DE 
    DataFrame,       // 数据帧 0x5A5A
    ErrorFrame       // 错误帧 0xCADE
};

// 发送帧数据结构
struct ExecuteFrame {
    uint16_t func_id;
    float arg1;
    float arg2;
    float arg3;
    uint8_t request_id;
};

// 返回帧数据结构
struct ResponseFrame {
    uint16_t func_id;
    uint32_t value;
    uint8_t request_id;
    bool success;
};

// 数据帧数据结构 CRC 后面添加一个byte对齐
struct DataFrame {
    uint16_t data_id;
    std::vector<uint8_t> data;
};

// 错误帧数据结构
struct ErrorFrame {
    uint8_t request_id;
    uint16_t error_code;
};

class ProtocolParser {
public:
    // 序列化执行帧（网络字节序）
    static std::vector<uint8_t> serialize_execute(const ExecuteFrame& frame);

    // 通用帧解析接口
    static FrameType parse_frame(const std::vector<uint8_t>& data);

    // 完整解析执行帧
    static ExecuteFrame parse_execute(const std::vector<uint8_t>& data);

    // 解析返回帧
    static ResponseFrame parse_response(const std::vector<uint8_t>& data);

    // 解析数据帧
    static DataFrame parse_data(const std::vector<uint8_t>& data);

    // 解析错误帧
    static ErrorFrame parse_error(const std::vector<uint8_t>& data);
    
    // 字节序转换辅助函数
    // 大端序 (网络字节序) 读取函数
    static uint16_t read_be16(const uint8_t* p);
    static uint32_t read_be32(const uint8_t* p);
    
    // 小端序读取函数
    static uint16_t read_le16(const uint8_t* p);
    static uint32_t read_le32(const uint8_t* p);

private:


    // 帧验证系列函数
    static void validate_execute_frame(const std::vector<uint8_t>& data);
    static void validate_response_frame(const std::vector<uint8_t>& data);
    static void validate_data_frame(const std::vector<uint8_t>& data);
    static void validate_error_frame(const std::vector<uint8_t>& data);

    static void validate_generic_frame(
        const std::vector<uint8_t>& data,
        size_t expected_length,
        uint16_t expected_tail,
        size_t crc_start_offset,
        size_t crc_length
    );
};

} // namespace ares_comm