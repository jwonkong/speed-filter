import csv
import matplotlib.pyplot as plt

# CSV 파일 경로를 따로 지정해줍니다.
csv_file = '/home/kong/cpp_project/vehicle_trajectory.csv'

# 데이터를 저장할 리스트 초기화
x_data = []
y_data = []

# CSV 파일 열기 및 데이터 읽기
with open(csv_file, 'r') as file:
    csv_reader = csv.reader(file)
    next(csv_reader)  # 첫 번째 행은 헤더이므로 스킵
    for row in csv_reader:
        x_data.append(-float(row[1]))
        y_data.append(float(row[0]))

# 그래프 그리기 (scatter plot으로 변경)
plt.figure(figsize=(10, 6))
plt.scatter(x_data, y_data, label='Vehicle Trajectory', color='b', marker='o', s=5)  # s는 점 크기
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('vehicle path')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()