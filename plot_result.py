import csv
import numpy as np
import matplotlib
import pdb
import matplotlib.pyplot as plt 
#matplotlib.use('TkAgg')

def read_trajectory(n_states, n_controls, results_path):
    # TODO clean this up
    reader = csv.reader(open(results_path))   
    pdb.set_trace()    
    states = []
    controls = []
    for i, row in enumerate(reader):   
        s = [float(num) for num in row[:n_states]]
        states.append(s)

        if row[-1] != ' ': # check for terminal state
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


def read_initial_road():
    center_lane_x = []
    center_lane_y = []
    left_lane_x = []
    left_lane_y = []
    right_lane_x = []
    right_lane_y = []

    with open('road_data/data1.txt', 'r') as file:
        lines = file.readlines()
        for line in lines:
            # 将每行数据拆分为六个部分，并将它们转换为浮点数
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
    def __init__(self, State, L):
        x_start = State[0]
        y_start = State[1]
        x_end = State[0] + L * np.cos(State[2])
        y_end = State[1] + L * np.sin(State[2])
        plt.arrow(x_start, y_start, x_end - x_start, y_end - y_start, 
                  head_width=0.3, head_length=1, fc='red', ec='red')


if __name__ == "__main__":
    # TODO take n_states, n_controls, result_path from argparser

    # Currently set up for acrobot
    n_states = 4
    n_controls = 1

    center_x, center_y, left_x, left_y, right_x, right_y = read_initial_road()
    plt.figure(1)
    plt.plot(left_x, left_y, marker='', linestyle='-', color='green', label='Left Lane')
    plt.plot(right_x, right_y, marker='', linestyle='-', color='green', label='Right Lane')
    plt.plot(center_x, center_y, marker='', linestyle='--', color='red', label='Center Lane')
   
    compress_path = 'compress_road_path_data/road_path_compress1.csv'
    compress_lx, compress_ly, compress_rx, compress_ry = read_compress_roadPath(compress_path)
    plt.plot(compress_lx, compress_ly, marker='', linestyle='-', color='purple', label='Compress Left Lane')
    plt.plot(compress_rx, compress_ry, marker='', linestyle='-', color='purple', label='Compress Right Lane')

    Start_state = [2.0, 2.0, 0.0, 0.0]
    Terminal_state = [55.0, 0.0, -0.1, 0]
    Arrow(Start_state, 5)
    Arrow(Terminal_state, 5)
    plt.plot(Start_state[0], Start_state[1], marker='o', color='blue')
    plt.plot(Terminal_state[0], Terminal_state[1], marker='x', color='red')

    results_path = 'road_path_planning_data/ipddp_result1.csv'
    x, u = read_trajectory(n_states, n_controls, results_path)
    plt.plot(11,11)
    plt.plot(x[:, 0], x[:, 1],'r', label='path_ipddp')
    plt.title('ipddp motion planning')
    plt.legend()
    plt.show()

    # plt.figure(2)
    # plt.plot(u[:, 0], 'r--', label='u1')
    # plt.legend()
    # plt.show()

   