import sys
sys.path.append('/home/ares/.local/lib/python3.10/site-packages')  # 根据实际路径调整
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
import csv
import os
import re

# 定义.db3文件夹路径（无需包含.db3扩展名）
bag_path = 'pose_data'  # 替换为你的.db3文件夹路径
output_dir = 'csv_output'
os.makedirs(output_dir, exist_ok=True)

# 初始化ROS2消息类型解析器
typestore = get_typestore(Stores.ROS2_FOXY)

with Reader(bag_path) as reader:
    for connection in reader.connections:
        topic = connection.topic
        safe_topic = re.sub(r'[^a-zA-Z0-9_\-]', '_', topic)
        csv_path = os.path.join(output_dir, f'{safe_topic}.csv')
        
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # 按消息类型定义表头（以Odometry为例）
            writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            
            for _, timestamp, rawdata in reader.messages(connections=[connection]):
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                try:
                    # 提取位置和姿态数据
                    pos = msg.pose.pose.position
                    ori = msg.pose.pose.orientation
                    writer.writerow([
                        timestamp,
                        pos.x, pos.y, pos.z,
                        ori.x, ori.y, ori.z, ori.w
                    ])
                except AttributeError:
                    print(f"Topic {topic} 包含不支持的消息类型")