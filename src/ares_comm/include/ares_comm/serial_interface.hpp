#pragma once
#include <vector>
#include <memory>
#include <stdexcept>

namespace ares_comm {

class SerialException : public std::runtime_error {
public:
    explicit SerialException(const std::string& msg) 
        : std::runtime_error(msg) {}
};

class SerialInterface {
public:
    virtual ~SerialInterface() = default;
    
    // 基本操作接口
    virtual void open(const std::string& port, uint32_t baudrate) = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;
    virtual size_t read(uint8_t* buffer, size_t size, uint32_t timeout_ms = 100) = 0;
    virtual void write(const std::vector<uint8_t>& data) = 0;
    
    // 协议相关接口
    virtual std::vector<uint8_t> read_frame() = 0;

    // 工厂方法
    static std::unique_ptr<SerialInterface> create(
        const std::string& port = "", 
        uint32_t baudrate = 115200
    );
};

/* Linux 实现声明 */
class LinuxSerial : public SerialInterface {
public:
    LinuxSerial();
    ~LinuxSerial() override;
    
    void open(const std::string& port, uint32_t baudrate) override;
    void close() override;
    bool is_open() const override;
    size_t read(uint8_t* buffer, size_t size, uint32_t timeout_ms = 100) override;
    void write(const std::vector<uint8_t>& data) override;
    std::vector<uint8_t> read_frame() override;

private:
    int fd_ = -1;
};

} // namespace ares_comm