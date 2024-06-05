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

        self.is_accident = False # 사고가 났는가
        self.recovery_count = 0 # 복구 횟수
        self.accident_count = 0 # 사고 횟수

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
        ## 핸들을 돌리는데 필요한 변수 : 전방의 커브 각도, 차량 속도 (+ 중앙에서 벗어난 정도) 

        ## 도로의 실제 폭의 1/2 로 계산됨
        half_load_width = self.half_road_limit - 1.25 # 도로폭 : 10m, 차량 전폭 : 2m

        ## 현재 속도와 전방 커브 각도에 따른 커브 각도 계산
        def get_ref_angle(speed):
            # track_forward_angles : 차량 전방의 20개 구간에 대한 각도 (10m씩 총 200m)
            angle_num = min(int(speed / 45), len(sensing_info.track_forward_angles) - 1)
            ref_angle = sensing_info.track_forward_angles[angle_num]
            return ref_angle
        
        ref_angle = get_ref_angle(sensing_info.speed)

        ## 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
        middle_add = (sensing_info.to_middle / 80) * -1

        ## 전방의 커브 각도에 따라 throttle 값을 조절하여 속도를 제어함
        throttle_factor = 0.6 / (abs(ref_angle) + 0.1) # 커브 각이 클수록 엑셀을 살살 밟음, 커브 각이 클수록 엑셀을 세게 밟음
        if throttle_factor > 0.11: throttle_factor = 0.11  ## throttle 값을 최대 0.81 로 설정
        set_throttle = 0.8 + throttle_factor
        if sensing_info.speed < 60: set_throttle = 1.0  ## 속도가 60Km/h 이하인 경우 1.0 로 설정

        ## 차량의 Speed 에 따라서 핸들을 돌리는 값을 조정함
        steer_factor = sensing_info.speed * 1.5 # 70km 이하 (50km이면 steer_factor는 75)
        if sensing_info.speed > 70: steer_factor = sensing_info.speed * 0.85 # 70~100km (80km이면 steer_factor는 68)
        if sensing_info.speed > 100: steer_factor = sensing_info.speed * 0.7 # 100km 이상 (120km이면 steer_factor는 84)

        ## (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
        set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001)
        # 커브가 왼쪽(-)으로 꺾여있고, 주행 각도가 오른쪽(+)이면 왼쪽으로 핸들 돌리기(-)
        # 커브가 오른쪽(+)으로 꺾여있고, 주행 각도가 왼쪽(-)이면 오른쪽으로 핸들 돌리기(+)
        # (참고할 전방의 커브 - 내 차량의 주행 각도)는 일반적인 경우 -60도에서 +60까지 일것으로 예상
        
        ## 차선 중앙정렬 값을 추가로 고려함
        set_steering += middle_add # middle_add : -0.075 ~ +0.075
        ###########################################################################

        # Moving straight forward
        car_controls.steering = set_steering
        car_controls.throttle = set_throttle
        car_controls.brake = set_brake
        
        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}"\
                  .format(car_controls.steering, car_controls.throttle, car_controls.brake))

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
