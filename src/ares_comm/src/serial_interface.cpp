#include "ares_comm/serial_interface.hpp"  // 正确包含自定义头文件
#include "ares_comm/protocol_parser.hpp"  // 添加协议解析器头文件
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <stdexcept>
#include <string>

namespace ares_comm {

/* ================= 接口工厂方法 ================= */
std::unique_ptr<SerialInterface> SerialInterface::create(const std::string& port, uint32_t baudrate) {
    auto ser = std::make_unique<LinuxSerial>();
    if (!port.empty()) {
        ser->open(port, baudrate);
    }
    return ser;
}

/* LinuxSerial 构造函数 */
LinuxSerial::LinuxSerial() : fd_(-1) {}  // 初始化文件描述符为无效值

/* LinuxSerial 析构函数 */
LinuxSerial::~LinuxSerial() {
    if (is_open()) {
        close();  // 确保资源释放
    }
}

/* 打开串口 */
void LinuxSerial::open(const std::string& port, uint32_t baudrate) {
    if (is_open()) {
        throw SerialException("Port " + port + " already open");
    }

    // 以非阻塞模式打开设备
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        throw SerialException("Failed to open " + port + ": " + strerror(errno));
    }

    // 获取当前串口配置
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close();
        throw SerialException("tcgetattr failed: " + std::string(strerror(errno)));
    }

    // 基础串口配置
    tty.c_cflag &= ~PARENB;   // 禁用奇偶校验
    tty.c_cflag &= ~CSTOPB;   // 1位停止位
    tty.c_cflag |= CS8;       // 8位数据位
    tty.c_cflag &= ~CRTSCTS;  // 禁用硬件流控
    tty.c_cflag |= CREAD | CLOCAL; // 启用接收，忽略控制线

    // 输入模式配置
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // 输出模式配置
    tty.c_oflag &= ~OPOST;  // 原始输出模式
    tty.c_oflag &= ~ONLCR;

    // 本地模式配置
    tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);

    // 设置波特率
    speed_t speed = B115200;
    switch (baudrate) {
        case 9600:    speed = B9600; break;
        case 19200:   speed = B19200; break;
        case 38400:   speed = B38400; break;
        case 57600:   speed = B57600; break;
        case 115200:  speed = B115200; break;
        case 230400:  speed = B230400; break;
        default:
            close();
            throw SerialException("Unsupported baudrate: " + std::to_string(baudrate));
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 应用配置
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close();
        throw SerialException("tcsetattr failed: " + std::string(strerror(errno)));
    }

    // 清空输入输出缓冲区
    tcflush(fd_, TCIOFLUSH);
}

/* 关闭串口 */
void LinuxSerial::close() {
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;  // 标记为已关闭
    }
}

/* 检查端口状态 */
bool LinuxSerial::is_open() const {
    return fd_ != -1;
}

std::vector<uint8_t> LinuxSerial::read_frame() {
    constexpr size_t HEADER_SIZE = 2;
    constexpr size_t MAX_FRAME_SIZE = 256;
    std::vector<uint8_t> buffer(MAX_FRAME_SIZE);
    size_t total_read = 0;

    // 寻找有效的帧头
    bool found_header = false;
    uint16_t header = 0;
    
    while (!found_header) {
        // 确保缓冲区有足够空间
        if (total_read >= buffer.size() - 1) {
            // 删除前面的数据，保留最后一个字节（可能是帧头的一部分）
            if (total_read > 0) {
                memmove(buffer.data(), buffer.data() + total_read - 1, 1);
                total_read = 1;
            } else {
                total_read = 0;
            }
        }
        
        // 读取更多数据
        size_t n = read(buffer.data() + total_read, buffer.size() - total_read);
        if (n == 0) throw SerialException("Read timeout");
        
        // 在新读取的数据中查找有效帧头
        for (size_t i = total_read; i < total_read + n - 1; i++) {
            header = (buffer[i] << 8) | buffer[i+1];
            if (header == 0xCAFE || header == 0xC0DE || header == 0x5A5A || header == 0xCADE) {
                // 找到有效帧头，将帧头移到缓冲区开始位置
                if (i > 0) {
                    memmove(buffer.data(), buffer.data() + i, (total_read + n) - i);
                    total_read = (total_read + n) - i;
                } else {
                    total_read += n;
                }
                found_header = true;
                break;
            }
        }
        
        if (!found_header) {
            total_read += n;
        }
    }
    
    // 此时buffer[0]和buffer[1]应该是有效的帧头
    // 根据帧头确定帧长度
    size_t frame_length = 0;
    size_t min_frame_length = 0;
    uint16_t expected_tail = 0;
    
    header = (buffer[0] << 8) | buffer[1];
    switch (header) {
        case 0xCAFE:  // 执行帧
            frame_length = 20;
            min_frame_length = 20;
            expected_tail = 0xBABE;
            break;
        case 0xC0DE:  // 返回帧
            frame_length = 12;
            min_frame_length = 12;
            expected_tail = 0xBEEF;
            break;
        case 0x5A5A:  // 数据帧 (可变长度)
            min_frame_length = 7;  // 头(2) + 数据ID(2) + CRC(1) + 尾(2)
            expected_tail = 0xAA55;
            break;
        case 0xCADE:  // 心跳包
            frame_length = 8;
            min_frame_length = 8;
            expected_tail = 0xDEAD;
            break;
        default:
            throw SerialException("Unknown frame header");
    }
    
    // 数据帧需要特殊处理，因为它是可变长度的
    if (header == 0x5A5A) {
        // 读取更多数据直到找到帧尾 0xAA55
        while (true) {
            // 确保已读取至少最小帧长度
            if (total_read < min_frame_length) {
                size_t to_read = min_frame_length - total_read;
                size_t n = read(buffer.data() + total_read, to_read);
                if (n == 0) throw SerialException("Read timeout");
                total_read += n;
                continue;
            }
            
            // 在已读取的数据中查找帧尾
            for (size_t i = 0; i <= total_read - 2; i++) {
                if (i >= 4 && buffer[i] == 0xAA && buffer[i+1] == 0x55) {
                    frame_length = i + 2;  // 包括帧尾的总长度
                    break;
                }
            }
            
            if (frame_length > 0) {
                break;  // 找到完整帧
            }
            
            // 未找到帧尾，继续读取更多数据
            size_t n = read(buffer.data() + total_read, buffer.size() - total_read);
            if (n == 0) throw SerialException("Read timeout");
            total_read += n;
            
            // 防止缓冲区溢出
            if (total_read >= buffer.size() - 1) {
                throw SerialException("Frame too large or no valid tail found");
            }
        }
    } else {
        // 对于固定长度的帧，读取剩余数据
        while (total_read < frame_length) {
            size_t n = read(buffer.data() + total_read, frame_length - total_read);
            if (n == 0) throw SerialException("Read timeout");
            total_read += n;
        }
        
        // 验证帧尾
        uint16_t actual_tail = (buffer[frame_length-2] << 8) | buffer[frame_length-1];
        if (actual_tail != expected_tail) {
            throw SerialException("Invalid frame tail");
        }
    }
    
    // 创建一个包含实际帧数据的vector
    std::vector<uint8_t> frame(buffer.begin(), buffer.begin() + frame_length);
    
    return frame;
}

/* 读取数据 */
size_t LinuxSerial::read(uint8_t* buffer, size_t size, uint32_t timeout_ms) {
    if (!is_open()) {
        throw SerialException("Port not open");
    }

    // 设置select超时
    timeval timeout{};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    // 等待数据到达
    int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ret < 0) {
        throw SerialException("select error: " + std::string(strerror(errno)));
    } else if (ret == 0) {
        return 0; // 超时
    }

    // 读取数据
    ssize_t n = ::read(fd_, buffer, size);
    if (n < 0) {
        throw SerialException("read error: " + std::string(strerror(errno)));
    }
    return static_cast<size_t>(n);
}   

/* 写入数据 */
void LinuxSerial::write(const std::vector<uint8_t>& data) {
    if (!is_open()) {
        throw SerialException("Port not open");
    }

    ssize_t n = ::write(fd_, data.data(), data.size());
    if (n < 0) {
        throw SerialException("write error: " + std::string(strerror(errno)));
    } else if (static_cast<size_t>(n) != data.size()) {
        throw SerialException("write incomplete: " + std::to_string(n) + "/" + std::to_string(data.size()));
    }
}

// // 更新工厂方法
// std::unique_ptr<SerialInterface> SerialInterface::create(const std::string& port, uint32_t baudrate) {
//     auto ser = std::make_unique<LinuxSerial>();
//     if (!port.empty()) {
//         ser->open(port, baudrate);
//     }
//     return ser;
// }
} // namespace ares_comm