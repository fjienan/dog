import pandas as pd
import numpy as np

def convert_csv_to_tum(input_csv, output_tum):
    # 读取CSV文件
    df = pd.read_csv(input_csv)
    
    # 转换时间戳：纳秒转秒（若需要）
    if df['timestamp'].max() > 1e18:  # 判断是否为纳秒级时间戳
        df['timestamp'] = df['timestamp'] / 1e9
    
    # 按TUM格式要求排列列：时间戳 x y z qx qy qz qw
    tum_df = df[['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']]
    
    # 保存为TUM格式
    tum_df.to_csv(output_tum, sep=' ', header=False, index=False, float_format='%.6f')

# 示例：转换单个文件
convert_csv_to_tum('csv_output/your_topic.csv', 'output_tum.txt')