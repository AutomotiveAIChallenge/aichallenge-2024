import csv
import matplotlib.pyplot as plt
import os

# CSVファイルのパス
short_center_csv = '../csv/origin/short_center.csv'
long_center_csv = '../csv/origin/long_center.csv'
short_left_csv = '../csv/origin/short_left.csv'
long_left_csv = '../csv/origin/long_left.csv'
short_right_csv = '../csv/origin/short_right.csv'
long_right_csv = '../csv/origin/long_right.csv'

# CSVファイルからデータを読み込む関数
def read_csv(csv_filename):
    x = []
    y = []
    if os.path.exists(csv_filename):
        with open(csv_filename, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                x.append(float(row['local_x']))
                y.append(float(row['local_y']))
    return x, y

# プロットを作成
plt.figure(figsize=(10, 10))

# 各ラインのデータを読み込み、プロット
# センターライン（短い方）
short_center_x, short_center_y = read_csv(short_center_csv)
plt.plot(short_center_x, short_center_y, label='Short Centerline', color='blue', linestyle='-', marker='o')

# センターライン（長い方）
long_center_x, long_center_y = read_csv(long_center_csv)
plt.plot(long_center_x, long_center_y, label='Long Centerline', color='cyan', linestyle='-', marker='o')

# 左サイドライン（短い方）
short_left_x, short_left_y = read_csv(short_left_csv)
plt.plot(short_left_x, short_left_y, label='Short Left Side', color='red', linestyle='-', marker='o')

# 左サイドライン（長い方）
long_left_x, long_left_y = read_csv(long_left_csv)
plt.plot(long_left_x, long_left_y, label='Long Left Side', color='orange', linestyle='-', marker='o')

# 右サイドライン（短い方）
short_right_x, short_right_y = read_csv(short_right_csv)
plt.plot(short_right_x, short_right_y, label='Short Right Side', color='green', linestyle='-', marker='o')

# 右サイドライン（長い方）
long_right_x, long_right_y = read_csv(long_right_csv)
plt.plot(long_right_x, long_right_y, label='Long Right Side', color='lime', linestyle='-', marker='o')

# グラフの設定
plt.title('Lanelet Lines')
plt.xlabel('Local X')
plt.ylabel('Local Y')
plt.legend()
plt.grid(True)

# プロットを表示
plt.show()
