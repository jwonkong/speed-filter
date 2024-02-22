import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_combined_from_txt(file_paths):
    plt.ion()  # 대화형 모드 활성화
    colors = ['blue', 'red']
    
    while True:
        # plt.clf()  # 그래프 영역 클리어
        fig = plt.figure(figsize=(10, 6))
        
        for i, path in enumerate(file_paths):
            try:
                df = pd.read_csv(path, header=None, sep=',')
                if df.empty:  # 데이터프레임이 비어있으면 이번 루프를 스킵
                    continue
                df.columns = ['x', 'y']
                
                # 첫 번째 파일(path.txt)일 경우 선으로 이어서 그리기
                if i == 0:
                    plt.scatter(df['x'], df['y'], color=colors[0], label=f'File 1', s=5)
                    plt.plot(df['x'].to_numpy(), df['y'].to_numpy(), color=colors[i])
                    
                else:
                    plt.scatter(df['x'], df['y'], color=colors[i], label=f'File {i+1}', s=5)
            except Exception as e:
                print(f"Error reading {path}: {e}")

        plt.title('T-S Diagram')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.ylim(0, 200)  # y축 범위 설정
        plt.draw()
        plt.pause(0.4)  # 0.5초 동안 일시 정지
        plt.close(fig)  # 현재 그래프 창 닫기

file_paths = [
    # '/home/kong/cpp_project/path.txt'.
    # '/home/kong/cpp_project/object.txt',
    
    '/home/kong/catkin_ws/path.txt',
    '/home/kong/catkin_ws/object.txt',
]

plot_combined_from_txt(file_paths)
