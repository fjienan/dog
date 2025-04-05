#pragma once
#include <cstdint>
#include <vector>
#include <cstddef>

namespace ares_comm {

/**
 * @brief CRC-8 校验模块（标准CCITT算法）
 * 
 * 多项式: x^8 + x^2 + x + 1 (0x07)
 * 初始值: 0x00
 * 无输入/输出反转
 */
class CRC8 {
public:
    static uint8_t compute(const uint8_t* data, size_t len);
};

} 