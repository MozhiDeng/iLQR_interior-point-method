import os

# 定义一个函数来读取文件并计算超时次数
def count_time_out(file_path):
    time_out_count = 0
    sum_count = 0
    # 确保文件路径存在
    if os.path.exists(file_path):
        # 打开文件进行读取
        with open(file_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                # 检查每一行是否包含 "time out"
                if 'time out' in line:
                    time_out_count += 1
                elif 'max_breach_constrains' in line:
                    sum_count += 1

    return time_out_count, sum_count

def main():
    file_path = "/home/car/Project/iLQR_interior-point-method/output.txt"
    time_out_count, sum_count = count_time_out(file_path)
    print(f"interior iLQR success rate:{100*(1-time_out_count/sum_count)}%")

if __name__ == "__main__":
    main()