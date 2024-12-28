import csv
import numpy as np
import matplotlib.pyplot as plt 
import os

def read_trajectory(n_states, n_controls, results_path):
    reader = csv.reader(open(results_path))   
    states = []
    controls = []
    for i, row in enumerate(reader):   
        s = [float(num) for num in row[:n_states]]
        states.append(s)

        if row[-1] != ' ':  # check for terminal state
            a = [float(num) for num in row[-n_controls:]]
            controls.append(a)

    states = np.array(states)
    controls = np.array(controls)
    return states, controls

def read_compress_roadPath(compress_road_path):
    compress_left_x = []
    compress_left_y = []
    compress_right_x = []
    compress_right_y = []

    compress_reader = csv.reader(open(compress_road_path))   
    for row in compress_reader:
        if len(row) >= 6:
            compress_left_x.append(float(row[2]))
            compress_left_y.append(float(row[3]))
            compress_right_x.append(float(row[4]))
            compress_right_y.append(float(row[5]))

    return compress_left_x, compress_left_y, compress_right_x, compress_right_y

def read_initial_road(initial_road_file):
    center_lane_x = []
    center_lane_y = []
    left_lane_x = []
    left_lane_y = []
    right_lane_x = []
    right_lane_y = []

    with open(initial_road_file, 'r') as file:
        lines = file.readlines()
        for line in lines:
            data = line.split()
            if len(data) == 6:
                center_lane_x.append(float(data[0]))
                center_lane_y.append(float(data[1]))
                left_lane_x.append(float(data[2]))
                left_lane_y.append(float(data[3]))
                right_lane_x.append(float(data[4]))
                right_lane_y.append(float(data[5]))
    return center_lane_x, center_lane_y, left_lane_x, left_lane_y, right_lane_x, right_lane_y

class Arrow:
    def __init__(self, ax, State, L):
        x_start = State[0]
        y_start = State[1]
        x_end = State[0] + L * np.cos(State[2])
        y_end = State[1] + L * np.sin(State[2])
        ax.arrow(x_start, y_start, x_end - x_start, y_end - y_start, 
                 head_width=0.3, head_length=1, fc='red', ec='red')

def main(index):
    n_states = 4
    n_controls = 1
    
    initial_road_file = f'road_data/data{index}.txt'
    center_x, center_y, left_x, left_y, right_x, right_y = read_initial_road(initial_road_file)
    
    # 创建一个包含两个子图的 figure
    fig, axs = plt.subplots(2, 2, figsize=(21, 12))  # 2行2列的子图
    
    # 第一个子图 - 道路和轨迹
    ax1 = axs[0, 0]
    ax1.plot(left_x, left_y, marker='', linestyle='-', color='green', label='Left Lane')
    ax1.plot(right_x, right_y, marker='', linestyle='-', color='green', label='Right Lane')
    ax1.plot(center_x, center_y, marker='', linestyle='--', color='red', label='Center Lane')
   
    compress_path = f'process_road_data/road_compress{index}.csv'
    compress_lx, compress_ly, compress_rx, compress_ry = read_compress_roadPath(compress_path)
    ax1.plot(compress_lx, compress_ly, marker='', linestyle='-', color='purple', label='Compress Left Lane')
    ax1.plot(compress_rx, compress_ry, marker='', linestyle='-', color='purple', label='Compress Right Lane')

    Start_state = [2.0, 2.0, 0.0, 0.0]
    Terminal_state = [55.0, 0.0, -0.1, 0]
    
    # 将箭头绘制到第一个子图 ax1 上
    Arrow(ax1, Start_state, 5)
    Arrow(ax1, Terminal_state, 5)
    
    ax1.plot(Start_state[0], Start_state[1], marker='o', color='blue')
    ax1.plot(Terminal_state[0], Terminal_state[1], marker='x', color='red')

    results_path = f'planning_data/ipddp_result{index}.csv'
    x, u = read_trajectory(n_states, n_controls, results_path)
    ax1.plot(x[:, 0], x[:, 1], 'r', label='path_ipddp')
    ax1.set_title('ipddp motion planning')
    ax1.legend()

    # 第二个子图 - 控制输入 u1
    ax2 = axs[0, 1]
    ax2.plot(u[:, 0], 'r--', label='u1')
    ax2.set_title('Control Input dKappa')
    ax2.legend()
    
    # 第三个子图 - 车辆航向
    ax3 = axs[1, 0]
    ax3.plot(x[:, 2], 'b', label='heading')
    ax3.set_title('Car Heading')
    ax3.legend()
    
    # 第四个子图 - 车辆Kappa
    ax4 = axs[1, 1]
    ax4.plot(x[:, 3], 'g', label='Kappa')
    ax4.set_title('Car Kappa')
    ax4.legend()

    # 保存图像
    save_path = f'./output_figure/planning_result{index}.png'
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    plt.savefig(save_path, dpi=300)
    print(f"Figure saved as '{save_path}'")

    # 清理当前的图像，确保不会影响下一个图像
    plt.close(fig)

if __name__ == "__main__":
    for index in range(1, 101):
        main(index)
