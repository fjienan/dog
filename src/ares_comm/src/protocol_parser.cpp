#include "ares_comm/protocol_parser.hpp"
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>

namespace ares_comm {

// 字节序转换辅助函数实现
uint16_t ProtocolParser::read_be16(const uint8_t* p) {
    return (static_cast<uint16_t>(p[0]) << 8) | p[1];
}

uint32_t ProtocolParser::read_be32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8)  |
           p[3];
}

// 添加小端序读取函数实现
uint16_t ProtocolParser::read_le16(const uint8_t* p) {
    return (static_cast<uint16_t>(p[1]) << 8) | p[0];
}

uint32_t ProtocolParser::read_le32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[3]) << 24) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[1]) << 8)  |
           p[0];
}

// 序列化执行帧实现
std::vector<uint8_t> ProtocolParser::serialize_execute(const ExecuteFrame& frame) {
    std::vector<uint8_t> buffer(20); // 固定20字节长度
    
    // 头部
    buffer[0] = 0xCA;
    buffer[1] = 0xFE;
    
    // 功能ID - 仍使用大端序
    buffer[2] = static_cast<uint8_t>(frame.func_id >> 8);
    buffer[3] = static_cast<uint8_t>(frame.func_id & 0xFF);
    
    // // 参数区 - 修改为小端序
    // auto write_le32 = [&](size_t pos, uint32_t value) {
    //     buffer[pos+0] = static_cast<uint8_t>(value & 0xFF);
    //     buffer[pos+1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    //     buffer[pos+2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    //     buffer[pos+3] = static_cast<uint8_t>((value >> 24) & 0xFF);
    // };
    // write_le32(4, frame.arg1);
    // write_le32(8, frame.arg2);
    // write_le32(12, frame.arg3);
    
    // 使用memcpy
    memcpy(&buffer[4], &frame.arg1, sizeof(frame.arg1));
    memcpy(&buffer[8], &frame.arg2, sizeof(frame.arg2));
    memcpy(&buffer[12], &frame.arg3, sizeof(frame.arg3));

    // 请求ID
    buffer[16] = frame.request_id;
    
    // CRC计算（覆盖2-16字节）
    const uint8_t crc = CRC8::compute(buffer.data() + 2, 15);
    buffer[17] = crc;
    
    // 尾部
    buffer[18] = 0xBA;
    buffer[19] = 0xBE;
    
    return buffer;
}

std::string to_hex(uint8_t value) {
    char buf[3];
    snprintf(buf, sizeof(buf), "%02X", value);
    return buf;
}

std::string to_hex(uint16_t value) {
    char buf[5];
    snprintf(buf, sizeof(buf), "%04X", value);
    return buf;
}

// 通用帧帧验证实现
void ProtocolParser::validate_generic_frame(
    const std::vector<uint8_t>& data,
    size_t expected_length,
    uint16_t expected_tail,
    size_t crc_start_offset,
    size_t crc_length
) {
    // 长度验证
    if (data.size() != expected_length) {
        throw ProtocolException("Invalid frame length: Expected " + 
                              std::to_string(expected_length) + 
                              ", Got " + std::to_string(data.size()));
    }

    // CRC验证 functionID开始
    const uint8_t expected_crc = CRC8::compute(
        data.data() + crc_start_offset, 
        crc_length
    );
    if (data[expected_length - 3] != expected_crc) {
        throw ProtocolException("CRC mismatch: Expected 0x" + 
                              to_hex(expected_crc) + 
                              ", Got 0x" + to_hex(data[expected_length-3]));
    }

    // 尾部验证
    const uint16_t actual_tail = read_be16(data.data() + (expected_length - 2));  // 把帧变成数组 指针移动18个数
    if (actual_tail != expected_tail) {
        throw ProtocolException("Invalid tail: Expected 0x" + 
                              to_hex(expected_tail) + 
                              ", Got 0x" + to_hex(actual_tail));
    }
}

// 帧类型识别实现
FrameType ProtocolParser::parse_frame(const std::vector<uint8_t>& data) {
    if (data.size() < 4) {
        throw ProtocolException("Frame too short for header validation");
    }

    // 检查帧头标识
    const uint16_t header = read_be16(data.data());
    switch (header) {
        case 0xCAFE: return FrameType::ExecuteFrame;
        case 0xC0DE: return FrameType::ResponseFrame;
        case 0x5A5A: return FrameType::DataFrame;
        case 0xCADE: return FrameType::ErrorFrame;
        default:
            throw ProtocolException("Unknown frame header: 0x" + to_hex(header));
    }
}

// 发送帧解析实现
ExecuteFrame ProtocolParser::parse_execute(const std::vector<uint8_t>& data) {
    validate_execute_frame(data);

    ExecuteFrame frame;
    frame.func_id = read_be16(&data[2]);  // 功能ID仍使用大端序
    // 修改为使用小端序读取参数
    frame.arg1 = read_le32(&data[4]);
    frame.arg2 = read_le32(&data[8]);
    frame.arg3 = read_le32(&data[12]);
    frame.request_id = data[16];
    
    return frame;
}

// 接收帧解析实现
ResponseFrame ProtocolParser::parse_response(const std::vector<uint8_t>& data) {
    validate_response_frame(data);

    ResponseFrame frame;
    frame.func_id = read_be16(&data[2]);  // 功能ID仍使用大端序
    // 修改为使用小端序读取value
    frame.value = read_le32(&data[4]);
    frame.request_id = data[8];
    frame.success = (data[9] == 0x01);
    
    return frame;
}

// 数据帧解析实现
DataFrame ProtocolParser::parse_data(const std::vector<uint8_t>& data) {
    validate_data_frame(data);

    DataFrame frame;
    frame.data_id = read_be16(&data[2]);
    
    // 查找帧尾位置
    size_t tail_pos = 0;
    for (size_t i = 4; i < data.size() - 1; i++) {
        if (data[i] == 0xAA && data[i+1] == 0x55) {
            tail_pos = i;
            break;
        }
    }
    
    if (tail_pos == 0) {
        // 这不应该发生，因为validate_data_frame已经检查了帧尾
        throw ProtocolException("Data frame tail not found");
    }
    
    // 数据部分是从数据ID之后到CRC之前
    // 由于CRC校验暂时禁用，我们假设CRC位于帧尾前一个字节
    size_t data_length = tail_pos - 4; // 头(2) + 数据ID(2)到帧尾前的长度
    
    // 确保长度有效
    if (data_length > 0) {
        frame.data.resize(data_length);
        std::copy(data.begin() + 4, data.begin() + 4 + data_length, frame.data.begin());
    } else {
        // 空数据帧
        frame.data.clear();
    }
    
    return frame;
}

// 错误帧解析实现
ErrorFrame ProtocolParser::parse_error(const std::vector<uint8_t>& data) {
    // validate_error_frame(data);

    ErrorFrame frame;
    frame.request_id = data[2];
    // 修改为使用小端序读取错误码
    frame.error_code = read_le16(&data[3]);
    
    return frame;
}

// 发送帧验证实现
void ProtocolParser::validate_execute_frame(const std::vector<uint8_t>& data) {
    validate_generic_frame(
        data,
        20,         // 预期长度
        0xBABE,     // 预期尾部
        2,          // CRC计算起始偏移
        15          // CRC计算长度（2-16字节）
    );
}

// 接收帧验证实现
void ProtocolParser::validate_response_frame(const std::vector<uint8_t>& data) {
    validate_generic_frame(
        data,
        12,         // 预期长度
        0xBEEF,     // 预期尾部
        2,          // CRC计算起始偏移
        7           // CRC计算长度（2-8字节）
    );
}

// 数据帧验证实现
void ProtocolParser::validate_data_frame(const std::vector<uint8_t>& data) {
    // 最小data size
    if (data.size() < 7) {   
        throw ProtocolException("Data frame too short");
    }
    
    // 查找帧尾
    bool found_tail = false;
    size_t expected_tail_pos = 0;
    
    for (size_t i = 4; i < data.size() - 1; i++) {
        if (data[i] == 0xAA && data[i+1] == 0x55) {
            found_tail = true;
            expected_tail_pos = i;
            break;
        }
    }
    
    if (!found_tail) {
        throw ProtocolException("Cannot find data frame tail 0xAA55");
    }
    
    // 验证帧尾
    const uint16_t tail = read_be16(&data[expected_tail_pos]);
    if (tail != 0xAA55) {
        throw ProtocolException("Invalid tail: Expected 0xAA55, Got 0x" + 
                             to_hex(tail));
    }
    
    // 执行CRC校验，但不抛出异常，只记录日志
    static bool crc_warning_shown = false;  // 静态变量，确保警告只显示一次
    
    // CRC位于帧尾之前一个字节
    const uint8_t actual_crc = data[expected_tail_pos - 1];
    
    // 计算期望的CRC（从Data ID开始到CRC之前的所有数据）
    const uint8_t expected_crc = CRC8::compute(
        data.data() + 2,  // Data ID开始
        expected_tail_pos - 3  // 到CRC之前的长度
    );
    
    // 如果CRC不匹配并且尚未显示警告，则输出调试信息
    if (actual_crc != expected_crc && !crc_warning_shown) {
        // 打印十六进制格式的数据帧和CRC计算区域
        std::string hex_data = "";
        for (size_t i = 0; i < data.size(); i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), " %02X", data[i]);
            hex_data += buf;
            
            // 在CRC字节前后添加标记
            if (i == expected_tail_pos - 1) {
                hex_data += "[CRC]";
            }
        }
        
        std::cout << "数据帧CRC校验失败: 实际=" << std::hex << static_cast<int>(actual_crc)
                  << ", 期望=" << static_cast<int>(expected_crc) << std::endl;
        std::cout << "数据帧内容:" << hex_data << std::endl;
        std::cout << "CRC计算范围: 索引2-" << (expected_tail_pos - 3) << std::endl;
        
        crc_warning_shown = true;  // 设置标志，避免重复显示
    }
    
    // 即使CRC不匹配，也继续处理帧
}

// // 错误帧验证实现  没有CRC
// void ProtocolParser::validate_error_frame(const std::vector<uint8_t>& data) {
//     validate_generic_frame(
//         data,
//         6,          // 预期长度
//         0xDEAD,     // 预期尾部
//         // 2,          // CRC计算起始偏移
//         // 3           // CRC计算长度（2-4字节）
//     );
// }

} // namespace ares_comm



// 解析帧