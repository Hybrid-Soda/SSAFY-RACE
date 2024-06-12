from math import *
from DrivingInterface.drive_controller import DrivingController


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #
        self.is_debug = False

        self.track_type = 99

        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0

        # api or keyboard
        self.enable_api_control = True # True(Controlled by code) /False(Controlled by keyboard)
        super().set_enable_api_control(self.enable_api_control)

        #
        # Editing area ends
        # ==========================================================#
        super().__init__()
    
    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #

        if self.is_debug:
            print("=========================================================")
            print("[MyCar] to middle: {}".format(sensing_info.to_middle))

            print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))

            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))

            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################
        ## 기본 주행 ##
        # Throttle 및 Brake 설정
        car_controls.throttle = 1
        car_controls.brake = 0

        # 변수 초기화
        to_degree = 180 / pi
        to_radian = pi / 180
        speed = sensing_info.speed
        middle = sensing_info.to_middle
        RL = 1 if middle >= 0 else -1  # 오른쪽(양수) / 왼쪽(음수)

        # 웨이포인트까지의 거리 및 각도 계산
        length = [abs(middle)] + sensing_info.distance_to_way_points
        angles = [0] + sensing_info.track_forward_angles
        angle_differences = []
        angle_F = 90

        # 웨이포인트까지의 각도 및 방향 계산
        for i in range(20):
            angle_T = 180 - angle_F - (angles[i + 1] - angles[i]) * RL
            angle_F = asin(max(min(length[i] * sin(angle_T * to_radian) / length[i + 1], 1), -1)) * to_degree
            angle_differences.append(180 - angle_T - angle_F)

        # 웨이포인트 좌표 계산
        waypoints = []
        for j in range(20):
            temp = sum(angle_differences[: j + 1]) * to_radian
            waypoints.append([length[j + 1] * sin(temp), -length[j + 1] * RL * cos(temp)])

        # 속도에 따른 목표 웨이포인트 결정
        if speed < 100: target_waypoint = 3
        elif speed < 140: target_waypoint = 4
        else: target_waypoint = 4; car_controls.brake = 0.5

        # 목표 웨이포인트까지의 각도 계산
        target_theta = atan(waypoints[target_waypoint][1] / waypoints[target_waypoint][0]) * to_degree - sensing_info.moving_angle

        # 커브 각도에 따른 스티어링 설정
        if abs(angles[target_waypoint + 2]) < 49 or abs(angles[target_waypoint + 1]) < 47:
            if speed < 120:
                car_controls.steering = target_theta / 85
            else:
                car_controls.steering = target_theta / (speed * 0.95)
        else:
            radius = max(abs(waypoints[target_waypoint][0]), abs(waypoints[target_waypoint][1]))
            alpha = asin(sqrt(waypoints[target_waypoint][0] ** 2 + waypoints[target_waypoint][1] ** 2) / (2 * radius)) * 2
            beta = alpha * speed * 0.1 / radius
            beta = beta if target_theta >= 0 else -beta
            car_controls.steering = (beta - sensing_info.moving_angle * to_radian) * 1.15


        ## 예외 처리 ##
        ## 급커브
        if abs(angles[target_waypoint + 5]) > 50:
            car_controls.throttle = 0.5
            car_controls.brake = 0.5

        ## 장애물 탐지
        obstacles = []
        if sensing_info.track_forward_obstacles:
            for obstacle in sensing_info.track_forward_obstacles:
                if obstacle['dist'] > 50: continue
                obstacles.append([obstacle['dist'], obstacle['to_middle'] - sensing_info.to_middle])
        if obstacles and 1 < sensing_info.track_forward_obstacles[0]['dist'] < 60 and speed > 80:
                car_controls.brake = min(0.007 * speed, 1)
        obstacles.sort(key=lambda x: (x[1], x[0]))

        ## 장애물 회피
        l_path = r_path = 0
        for obstacle in obstacles:
            if abs(l_path - obstacle[1]) > 2.3:
                break
            l_path -= 3.1 - obstacle[1]
        
        for obstacle in obstacles:
            if abs(r_path - obstacle[1]) > 2.3:
                break
            r_path += 3.1 + obstacle[1]
        
        if abs(l_path) < abs(r_path):
            path = l_path
        else:
            path = r_path

        car_controls.steering += path * 8 / (abs(speed) + 1)

        ## 충돌 감지
        if sensing_info.lap_progress > 0.5 and not self.is_accident and abs(speed) < 1:
            self.accident_count += 1
        if self.accident_count > 6:
            self.is_accident = True

        ## 충돌 후처리
        if self.is_accident:
            self.recovery_count += 1
            car_controls.steering = 0
            car_controls.throttle = -1

        ## 정상 주행 전환
        if self.recovery_count > 15:
            car_controls.throttle = 1
            car_controls.steering = 0
            self.is_accident = False
            self.recovery_count = 0
            self.accident_count = 0
        
        ## 역주행
        if not sensing_info.moving_forward and not self.is_accident:
            if sensing_info.moving_angle > 0:
                car_controls.steering = 1
            else:
                car_controls.steering = -1
            car_controls.throttle = 1

        # Moving straight forward
        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # Editing area ends
        # ==========================================================#
        return car_controls


    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name


if __name__ == '__main__':
    print("[MyCar] Start Bot! (PYTHON)")

    client = DrivingClient()
    return_code = client.run()

    print("[MyCar] End Bot! (PYTHON)")

    exit(return_code)
