# 对应协议中的错误帧(0xCADE开头)
uint8 request_id   # 1字节请求标识符 (可能是执行帧或数据帧的ID)
uint16 error_code  # 2字节错误码 (使用位掩码组合)