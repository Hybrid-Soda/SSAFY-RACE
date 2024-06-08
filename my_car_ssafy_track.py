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
        # 제어 초기화
        set_throttle = 0
        set_steering = 0
        set_brake = 0

        ## 커브 각도 계산
        def get_ref_angle(speed):
            angle_num = min(int(speed / 45), len(sensing_info.track_forward_angles) - 1)
            ref_angle = sensing_info.track_forward_angles[angle_num]
            return ref_angle
        
        ref_angle = get_ref_angle(sensing_info.speed)

        ## 차선 중앙 정렬 (감쇠진동)
        middle_add = (sensing_info.to_middle / 200) * -1
        if abs(sensing_info.to_middle) >= self.half_road_limit: # 도로 이탈시 더 빠르게 중앙으로 복귀
            middle_add = (sensing_info.to_middle / 40) * -1
        elif abs(sensing_info.to_middle) >= self.half_road_limit-1:
            middle_add = (sensing_info.to_middle / 50) * -1
        elif abs(sensing_info.to_middle) >= self.half_road_limit-1.8:
            middle_add = (sensing_info.to_middle / 60) * -1
        elif abs(sensing_info.to_middle) >= self.half_road_limit-2.5:
            middle_add = (sensing_info.to_middle / 70) * -1

        ## 커브 각도에 따른 속도 제어
        throttle_factor = 0.6 / (abs(ref_angle) + 0.1)
        if throttle_factor > 0.11: throttle_factor = 0.11
        set_throttle = 0.8 + throttle_factor
        if sensing_info.speed < 60: set_throttle = 1.0

        ## 핸들링 제어
        speed = sensing_info.speed
        if 130 < speed:
            steer_factor = speed * 0.7 + 0.001
        elif 100 < speed:
            steer_factor = speed * 0.9 + 0.001
        else:
            steer_factor = speed * 1.5 + 0.001

        ## (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
        set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001) + middle_add


        ## 예외 처리 ##

        ## 1. 코너링
        if speed < 100:
            look_ahead = 1
        elif speed < 120:
            look_ahead = 2
        elif speed < 130:
            look_ahead = 3
        elif speed < 140:
            look_ahead = 4
        else:
            look_ahead = 5
            set_brake = 0.0013 * speed

        out_point = sensing_info.track_forward_angles[look_ahead + 6]
        curve_point = sensing_info.track_forward_angles[look_ahead]
        now_point = sensing_info.track_forward_angles[0]

        # 아웃 코스
        if abs(out_point) > 80:
            if out_point > 0 and sensing_info.to_middle < 0.5:
                set_steering -= 0.06
            elif out_point < 0 and -0.5 < sensing_info.to_middle:
                set_steering += 0.06
        # 커브
        if 60 < abs(curve_point) <= 80:
            if set_steering > 0:
                set_steering += 0.01 * abs(curve_point)
            else:
                set_steering -= 0.01 * abs(curve_point)
            print('최적화')

        # 드리프트
        elif speed > 100 and abs(curve_point) > 80:
            if set_steering > 0:
                set_steering = 1.0
            else:
                set_steering = -1.0
            set_brake = 0.9
            if speed > 100 and abs(now_point) > 20:
                set_brake = 0.0
                print('drift')

        # def adjust_speed_and_brake(set_throttle, set_brake, ref_angle, speed, track_forward_obstacles):
        #     MAX_SPEED = 90
        #     MIN_ANGLE = 40

        #     if speed > MAX_SPEED:
        #         speed_ratio = (speed - MAX_SPEED) / MAX_SPEED
        #         angle_ratio = max(0, ref_angle - MIN_ANGLE) / 90  # ref_angle이 MIN_ANGLE 이상일 때만 반영

        #         # set_throttle은 속도와 각도에 따라 이차함수적으로 감소
        #         set_throttle = max(0, 1 - (speed_ratio**2) - (angle_ratio**2))

        #         # set_brake는 속도와 각도에 따라 이차함수적으로 증가
        #         set_brake = min(1, (speed_ratio**2) + (angle_ratio**2) / 2)

        #         # ref_angle이 일정 기준을 넘으면 추가적으로 throttle과 brake 조정
        #         if ref_angle >= MIN_ANGLE:
        #             set_throttle= max(0, 1 - (ref_angle / speed) * 2.5)
        #             set_brake = min(1, (ref_angle / 250) + speed_ratio)

        #     # 장애물이 있는지 여부에 따라 속도 조정
        #     if track_forward_obstacles:
        #         set_throttle = 0.8  # 장애물이 있으면 throttle을 감소시킴
        #     else:
        #         set_throttle= 1.8  # 장애물이 없으면 throttle을 증가시킴

        #     # throttle과 brake 값의 범위 제한
        #     set_throttle = max(0, min(1, set_throttle))
        #     set_brake = max(0, min(1, set_brake))

        #     return set_throttle, set_brake
        # if sensing_info.speed >= 50:
        #    set_throttle, set_brake = adjust_speed_and_brake(set_throttle,set_brake,ref_angle,sensing_info.speed,sensing_info.track_forward_obstacles)


        def calculate_escape_steering(car_to_middle, first_to_middle, steer_coeff, steer_factor, diff_to_middle):
            # 장애물 피하기 스티어링 값을 계산하는 로직
           
            need_steering = ((2 + 0.2) - abs(diff_to_middle)) / (2 + 0.2)
            
            if car_to_middle > 0.5:
                if 5 - max(car_to_middle, first_to_middle) > (2 + 0.2) and car_to_middle-1 > first_to_middle:              
                    return +need_steering * steer_coeff / steer_factor
                else:
                    return -need_steering * steer_coeff / steer_factor
            elif car_to_middle < - 0.5:
                if 5 + min(car_to_middle, first_to_middle) > (2 + 0.2) and car_to_middle+1 < first_to_middle:    
                    return -need_steering * steer_coeff / steer_factor
                else:
                    return +need_steering * steer_coeff / steer_factor
            else:
                if first_to_middle < 0:
                    print("장애물이 왼쪽에 있는경우 핸들은 오른쪽으로")
                    return +need_steering * steer_coeff / steer_factor

                else:
                    print("장애물이 오른쪽에 있는경우 핸들은 왼쪽으로")
                    return -need_steering * steer_coeff / steer_factor 
        
        # 장애물 피하기
        # 1. 장애물 인식
        if sensing_info.track_forward_obstacles:  # 장애물이 인식될 때
            first_obstacles = sensing_info.track_forward_obstacles[0]
            first_dist = first_obstacles['dist']

            # 2. 피할 조건 설정
            if first_dist < 80: 
                if first_dist < 40:
                    print("장애물 발견")
                    set_throttle *= 0.85
                    car_to_middle = sensing_info.to_middle
                    first_to_middle = first_obstacles['to_middle']
                    diff_to_middle = first_to_middle - car_to_middle
                    steer_coeff = 50
                    
                    if abs(diff_to_middle) < 3:
                        set_steering = calculate_escape_steering(car_to_middle, first_to_middle, steer_coeff, steer_factor, diff_to_middle)
                        # set_steering = restore_direction(car_to_middle, steer_factor)

                print('set_steering', set_steering)
                if set_steering > 1:
                    set_steering = 1


        ## 충돌 상황 감지 및 회피
        ## 1. 30Km/h 이상의 속도로 달리는 경우 정상 적인 상황으로 간주
        if sensing_info.speed > 30.0:
            self.is_accident = False
            self.recovery_count = 0
            self.accident_count = 0

        ## 2. 레이싱 시작 후 Speed 1km/h 이하가 된 경우 상황 체크
        if sensing_info.lap_progress > 0.5 and self.is_accident == False and (sensing_info.speed < 1.0 and sensing_info.speed > -1.0):
            self.accident_count += 1

        ## 3. Speed 1km/h 이하인 상태가 지속된 경우 충돌로 인해 멈준 것으로 간주
        if self.accident_count > 6:
            set_brake = 0.5
            self.is_accident = True

        ## 4. 충돌로 멈춘 경우 후진 시작
        if self.is_accident == True:
            if sensing_info.to_middle < 0:
                set_steering = 0.02
            else:
                set_steering = -0.02
            set_brake = 0.0
            set_throttle = -1
            self.recovery_count += 1

        ## 5. 어느 정도 후진이 되었을 때 충돌을 회피 했다고 간주 후 정상 주행 상태
        if self.recovery_count > 15:
            self.is_accident = False
            self.recovery_count = 0
            self.accident_count = 0
            set_steering = 0
            set_brake = 0
            set_throttle = 1

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
