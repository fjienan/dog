import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

class InteractivePlotter:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        plt.subplots_adjust(bottom=0.2)
        self.colors = ['red', 'green', 'blue', 'purple', 'orange']
        self.current_color = 0
        self.xdata = []
        self.ydata = []
        self.circles = []
        
        # 初始化交互功能
        self.setup_interaction()
        self.update_help_text()
        self.update_plot()
        plt.show()

    def setup_interaction(self):
        """绑定交互事件"""
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        plt.ion()  # 启用交互模式

    def update_help_text(self):
        """显示帮助信息"""
        help_text = (
            "交互操作指南:\n"
            "左键单击: 添加点\n"
            "右键拖动: 移动点\n"
            "Delete键: 删除最近的点\n"
            "C键: 切换颜色\n"
            "R键: 重置画布"
        )
        self.ax.text(0.5, -0.15, help_text,
                    ha='center', va='top', transform=self.ax.transAxes,
                    fontsize=9, color='gray')

    def update_plot(self):
        """更新绘图"""
        self.ax.clear()
        self.ax.set_title(f"当前颜色: {self.colors[self.current_color]}", color=self.colors[self.current_color])
        self.ax.grid(True)
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(0, 10)
        
        # 绘制所有点
        self.circles = []
        for x, y in zip(self.xdata, self.ydata):
            circle = plt.Circle((x, y), 0.2, fc=self.colors[self.current_color], alpha=0.5)
            self.ax.add_patch(circle)
            self.circles.append(circle)
        
        self.update_help_text()
        self.fig.canvas.draw()

    # 事件处理函数
    def on_click(self, event):
        """鼠标点击事件"""
        if event.button == 1 and event.inaxes:  # 左键
            self.xdata.append(event.xdata)
            self.ydata.append(event.ydata)
            self.update_plot()

    def on_motion(self, event):
        """鼠标移动事件"""
        if event.button == 3 and event.inaxes:  # 右键拖拽
            closest_idx = self.find_closest_point(event.xdata, event.ydata)
            if closest_idx is not None:
                self.xdata[closest_idx] = event.xdata
                self.ydata[closest_idx] = event.ydata
                self.update_plot()

    def on_key(self, event):
        """键盘事件"""
        if event.key == 'delete':
            if self.xdata:
                self.xdata.pop()
                self.ydata.pop()
                self.update_plot()
        elif event.key.lower() == 'c':
            self.current_color = (self.current_color + 1) % len(self.colors)
            self.update_plot()
        elif event.key.lower() == 'r':
            self.xdata = []
            self.ydata = []
            self.update_plot()

    def find_closest_point(self, x, y):
        """找到最近的点"""
        if not self.xdata:
            return None
        distances = [(x - xi)**2 + (y - yi)**2 for xi, yi in zip(self.xdata, self.ydata)]
        return np.argmin(distances)

if __name__ == "__main__":
    print("启动交互式绘图程序...")
    print("操作提示:")
    print("- 左键单击添加点")
    print("- 右键拖动移动点")
    print("- Delete键删除最后一点")
    print("- C键切换颜色")
    print("- R键重置画布")
    InteractivePlotter()