import pandas as pd
import matplotlib.pyplot as plt
import csv
import time
import os

# T-S그래프에 대한 확인

def plot_vehicle_and_object_trajectory(vehicle_csv_path, object_csv_path, combined_txt_paths, additional_txt_paths):
    plt.ion()  # 대화형 모드 활성화

    while True:
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
        
        # 추가 텍스트 파일에서 값을 읽어와 그래프에 추가
        for txt_path in additional_txt_paths:
            try:
                if os.path.exists(txt_path):  # 파일이 존재하는지 확인
                    with open(txt_path, 'r') as file:
                        lines = file.readlines()
                        if len(lines) > 1:  # 파일이 1줄 이상인 경우에만 두 번째 줄의 데이터를 사용
                            y_value = float(lines[1].split(',')[1])  # 두 번째 줄의 두 번째 값(y값) 추출
                            plt.subplot(2, 1, 2)  # 서브플롯 (2, 1, 2)으로 이동
                            plt.axis('off')
                            plt.text(0.5, 0.6, f'INPUT A : 2.0', fontsize=24, ha='center')  # 원하는 위치에 텍스트 추가
                            if y_value != '':  # 파일 내용이 비어 있는지 확인
                                plt.text(0.5, 0.3, f'TARGET A : {y_value:3.1f}', fontsize=24, ha='center')  # 원하는 위치에 텍스트 추가
                            else:
                                plt.text(0.5, 0.3, f'TARGET A : -5.0', fontsize=24, ha='center')  # 파일 내용이 비어 있을 경우 기본값 출력
                        
                        else:
                            plt.subplot(2, 1, 2)  # 서브플롯 (2, 1, 2)으로 이동
                            plt.axis('off')
                            plt.text(0.5, 0.6, f'INPUT A : 2.0', fontsize=24, ha='center')  # 원하는 위치에 텍스트 추가
                            plt.text(0.5, 0.3, f'TARGET A : -5.0', fontsize=24, ha='center')  # 원하는 위치에 텍스트 추가
            except Exception as e:
                print(f"Error reading {txt_path}: {e}")
        
        plt.draw()
        plt.pause(0.01)
        plt.clf()


# CSV 파일 경로 지정
vehicle_csv_path = '/home/kong/catkin_ws/vehicle_trajectory.csv'
object_csv_path = '/home/kong/catkin_ws/object_point.csv'

combined_txt_paths = [
    '/home/kong/catkin_ws/path.txt',
    '/home/kong/catkin_ws/object.txt',
]

additional_txt_paths = [
    '/home/kong/catkin_ws/cmd_acceleration.txt',
]

# 그래프 업데이트 함수 호출
plot_vehicle_and_object_trajectory(vehicle_csv_path, object_csv_path, combined_txt_paths, additional_txt_paths)
