# 对应协议中的同步数据帧(0x5A5A开头)
uint16 data_id   # 2字节数据标识符 (对应协议中的Data ID)
uint8[] data     # 可变长度数据载荷 (最长22字节)