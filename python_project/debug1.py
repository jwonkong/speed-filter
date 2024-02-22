import pandas as pd
import matplotlib.pyplot as plt
import csv
import time

# 차의 경로와 T-S그래프에 대한 확인

def plot_vehicle_and_object_trajectory(vehicle_csv_path, object_csv_path, combined_txt_paths):
    plt.ion()  # 대화형 모드 활성화

    while True:
        # vehicle_trajectory.csv 파일에서 데이터 읽기
        x_data, y_data = [], []
        try:
            with open(vehicle_csv_path, 'r') as f:
                reader = csv.reader(f)
                next(reader)  # 헤더 제거
                for row in reader:
                    x_data.append(-float(row[1]))  # X 데이터
                    y_data.append(float(row[0]))  # Y 데이터
        except (FileNotFoundError, StopIteration):
            pass

        # object_point.csv 파일에서 데이터 읽기
        obj_x, obj_y = [], []
        try:
            with open(object_csv_path, 'r') as f:
                reader = csv.reader(f)
                next(reader)  # 헤더 제거
                for row in reader:
                    obj_x.append(-float(row[1]))  # X 데이터
                    obj_y.append(float(row[0]))  # Y 데이터
        except (FileNotFoundError, StopIteration):
            pass
                

        # 첫 번째 그래프: vehicle_trajectory
        plt.subplot(2, 1, 1)
        if x_data and y_data:
            plt.scatter(x_data, y_data, color='blue', label='Vehicle Trajectory', s=5)
            if obj_x is not None and obj_y is not None:
                plt.scatter(obj_x, obj_y, color='red', label='Object', s=200)  # 빨간색 점 크기 조정
            # plt.xlabel('X')
            plt.ylabel('Longitudinal')
            plt.title('Vehicle Trajectory and Object Point')
            plt.legend()
            plt.grid(True)
            plt.xlim(-40, 40)  # x 축 범위 고정
            plt.ylim(0, 100)  # y 축 범위 고정

        # 두 번째 그래프: Combined from TXT
        plt.subplot(2, 1, 1)
        plt.subplot(2, 1, 1)
        colors = ['blue', 'red']
        for i, path in enumerate(combined_txt_paths):
            try:
                df = pd.read_csv(path, header=None, sep=',')
                if not df.empty:  
                    df.columns = ['x', 'y']
                    if i == 0:
                        plt.scatter(df['x'], df['y'], color=colors[0], label=f'Speed profile', s=5)
                        plt.plot(df['x'].to_numpy(), df['y'].to_numpy(), color=colors[i])
                    else:
                        plt.scatter(df['x'], df['y'], color=colors[i], label=f'Obstacle', s=30)
            except Exception as e:
                print(f"Error reading {path}: {e}")

        plt.title('T-S Diagram')
        plt.xlabel('T')
        plt.ylabel('S')
        plt.ylim(0, 120)
        plt.xlim(0, 8)
        plt.legend(loc='upper right')  
        
        # 텍스트 추가
        plt.text(0.5, -10, 'Additional Text', fontsize=12, ha='center')
        
        plt.draw()
        plt.pause(0.001)
        plt.clf()



# CSV 파일 경로 지정
vehicle_csv_path = '/home/kong/catkin_ws/vehicle_trajectory.csv'
object_csv_path = '/home/kong/catkin_ws/object_point.csv'

combined_txt_paths = [
    '/home/kong/catkin_ws/path.txt',
    '/home/kong/catkin_ws/object.txt',
]

# 그래프 업데이트 함수 호출
plot_vehicle_and_object_trajectory(vehicle_csv_path, object_csv_path, combined_txt_paths)