 
import numpy as np

def follow_the_gap(raw_distances):
    n = len(raw_distances)
    
    left_part = raw_distances[-int(n * 0.25):]
    right_part = raw_distances[:int(n * 0.25)]
    sum_radius = np.concatenate([left_part , right_part])

    # 인덱스 당 각도 차이
    angle_min = -1.5708  
    angle_max = 1.5708   
    angle_difference = (angle_max - angle_min) / (len(sum_radius) - 1)

    # 데이터 전처리
    lidar_distances = np.asarray(sum_radius).copy()
    max_distances = 3000.0
    threshold = 100 

    # error
    lidar_distances = np.where(lidar_distances <= 0, max_distances, lidar_distances)
    lidar_distances[lidar_distances > max_distances] = max_distances
    
    # 노이즈(너무 가까운 것) 처리 -> 무시하기 위해 max로 보냄
    lidar_distances[lidar_distances < threshold] = max_distances

    # 위험 거리 제한
    danger_threshold = 1500.0 
    danger_indices = np.where(lidar_distances < danger_threshold)[0]

    
    # 4. Gap 찾기
    car_width = 800  
    best_gap = (0, 0)
    a = -1
#시작은 인덱스가 아무리 작아봐야 0이지만 끝은 인덱스를 모르기 때문에 한계를 정해줘야
    for s,e in zip(start, end):
        if s - e >= 0 : continue

        start_distance = lidar_distances[s]
        end_distance = lidar_distances[min(e, len(lidar_distances)-1)]
        theta = (e -s)*angle_difference
        gap = np.sqrt(max(0, start_distance**2+end_distance**2 - 2*start_distance*end_distance*np.cos(theta)))
        if gap > car_width:
            center =  (e+s) // 2
            b = lidar_distance[center] - (abs(center - len(lidar_distances//2)*5)
            if b > a:
                  a = b
                  best_gap = (s,e)
            
                                        

    # 조향각 제한 (약 24도)
    steering_angle_l = np.clip(steering_angle_l, -0.42, 0.42)

    return steering_angle_l


