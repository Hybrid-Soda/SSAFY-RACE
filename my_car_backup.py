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
        
        ## 긴급 및 예외 상황 처리(초기화) ########################################################################################
        full_throttle = True # 풀악셀은 밟아둠
        emergency_brake = False # 비상 브레이크는 일단 떼둠

        ## 전방 커브의 각도가 큰 경우 속도를 제어함
        ## 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
        road_range = int(sensing_info.speed / 30) # road_range : ~29km(0), 30~59km(0-1), 60~89km(0-2), 90~119km(0-3), 120~149km(0-4), 150km~(0-5)
        for i in range(0, road_range): # 바로 앞 구간부터 순차적으로 고려
            fwd_angle = abs(sensing_info.track_forward_angles[i]) 
            if fwd_angle > 45:  ## 커브가 45도 이상인 경우 brake, throttle 을 제어
                full_throttle = False # 풀 악셀을 뗌
            if fwd_angle > 80:  ## 커브가 80도 이상인 경우 steering 까지 추가로 제어
                emergency_brake = True # 비상 브레이크 밟음
                break # 다른 구간은 더이상 고려하지 않음

        ## brake, throttle 제어 # 결국 마지막에 남는 set_throttle값과 set_brake값이 맨 밑에 가서 적용됨
        set_brake = 0.0 # 밟지 않음으로 초기화
        if full_throttle == False: # 풀악셀을 밟지 않을때
            if sensing_info.speed > 100: # 100km 넘어가면
                set_brake = 0.0 # 속도 증가 (풀 악셀)
            if sensing_info.speed > 120: # 120km 넘어가면
                set_throttle = 0.8 # 속도 약간 증가 (노말 악셀, 노말 브레이크)
                set_brake = 0.3 
            if sensing_info.speed > 140: # 140km 넘어가면
                set_throttle = 0.5 # 속도 감소 (하프 악셀, 풀 브레이크)
                set_brake = 0.9

        ## steering 까지 추가로 제어
        if emergency_brake: # 비상 브레이크 밟을때
            if set_steering > 0: # 오른쪽으로 커브돌때
                set_steering += 0.3 # 커브 0.3 추가
            else: # 왼쪽으로 커브돌때
                set_steering -= 0.3 # 커브 -0.3 추가


        ## 장애물 피하기 
        ## 1. 장애물 인식
        if sensing_info.track_forward_obstacles: # 장애물이 인식될때
            first_obstacles = sensing_info.track_forward_obstacles[0]
            first_dist = first_obstacles['dist']
        ## 2. 피할 조건 설정
            if first_dist < 30: # 30m 이내로 장애물이 들어오면
                first_to_middle = first_obstacles['to_middle']
                car_to_middle = sensing_info.to_middle
                diff_to_middle = first_to_middle - car_to_middle # 장애물과 차의 위치차이
                print('diff_to_middle', diff_to_middle)
                steer_coeff = 50
                ## 3. 스티어링 조절
                if abs(diff_to_middle) < (2 + 0.2): # 차이가 2.2m보다 작으면 # 0.2m는 여유
                    need_steering = ((2 + 0.2) - abs(diff_to_middle)) / (2 + 0.2) # 스티어링이 필요한 정도 # 차이가 0일때 1
                    if car_to_middle > 0: # 차가 트랙보다 오른쪽에 있으면
                        # 오른쪽 트랙에 여유가 있는지 계산
                        if 5 - max(car_to_middle, first_to_middle) > (2 + 0.2): 
                            # 오른쪽으로 이동
                            set_steering = +need_steering * steer_coeff / steer_factor # steer_factor에 따라 조절 # steer_factor가 0일때?
                        else:
                            # 왼쪽으로 이동
                            set_steering = -need_steering * steer_coeff / steer_factor # 속도가 커지면 steer_factor 커짐, 커브각(set_steering) 줄어듬  
                    if car_to_middle < 0: # 차가 트랙보다 왼쪽에 있으면
                        # 왼쪽 트랙에 여유가 있는지 계산
                        if 5 + min(car_to_middle, first_to_middle) > (2 + 0.2): 
                            # 왼쪽으로 이동
                            set_steering = -need_steering * steer_coeff / steer_factor
                        else:
                            # 오른쪽으로 이동
                            set_steering = +need_steering * steer_coeff / steer_factor
                    # 차량이 정가운데 있는경우 
                    else:
                        # 장애물이 왼쪽에 있는경우 핸들은 오른쪽으로 
                        if first_to_middle < 0:
                            set_steering = +need_steering * steer_coeff / steer_factor
                        # 장애물이 오른쪽에 있는경우 핸들은 왼쪽으로
                        if first_to_middle > 0:
                            set_steering = -need_steering * steer_coeff / steer_factor
                print('set_steering', set_steering)
                if set_steering > 1: # set_steering이 1보다 커진 경우 보정해줌
                    set_steering = 1

        ## 충돌 상황 감지 후 회피 하기 (1~5 단계)
        ## 1. 30Km/h 이상의 속도로 달리는 경우 정상 적인 상황으로 간주
        if sensing_info.speed > 30.0:
            self.is_accident = False # 사고 안났다고 인식
            self.recovery_count = 0 # 복구 카운트 초기화
            self.accident_count = 0 # 사고 카운트 초기화

        ## 2. 레이싱 시작 후 Speed 1km/h 이하가 된 경우 상황 체크 (사고 직후 장애물이나 벽에서 살짝 밀려난 경우)
        if sensing_info.lap_progress > 0.5 and self.is_accident == False and (sensing_info.speed < 1.0 and sensing_info.speed > -1.0): # 구간의 절반도 가지 않았는데, 사고는 나지 않았고, 속도가 -1km ~ 1km인 경우
            self.accident_count += 1 # 사고 카운트

        ## 3. Speed 1km/h 이하인 상태가 지속된 경우 충돌로 인해 멈준 것으로 간주
        if self.accident_count > 6: # 사고카운트가 6이 넘어가면 (0.1초마다 적용되었으니 0.6초가 지난 경우)
            self.is_accident = True # 사고났음을 알림

        ## 4. 충돌로 멈춘 경우 후진 시작
        if self.is_accident == True: # 사고난경우
            set_steering = 0.02 # 오른쪽으로 핸들 살짝 꺾은 후
            set_brake = 0.0 # 브레이크는 떼고
            set_throttle = -1 # 풀악셀로 후진
            self.recovery_count += 1 # 사고를 복구한것으로 카운트

        ## 5. 어느 정도 후진이 되었을 때 충돌을 회피 했다고 간주 후 정상 주행 상태로 돌림
        if self.recovery_count > 20: # 복구 카운트가 20이 넘은 경우 (0.1초마다 적용되었으니 2초가 지난 경우)
            self.is_accident = False # 사고를 복구했다고 표시
            self.recovery_count = 0 # 복구 카운트 초기화
            self.accident_count = 0 # 사고 카운트 초기화
            set_steering = 0 # 핸들 초기화
            set_brake = 0 # 브레이크 초기화
            set_throttle = 0 # 악셀 초기화
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
