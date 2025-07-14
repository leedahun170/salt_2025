#!/usr/bin/env python
# -*- coding:utf-8 -*-
# limo_application/scripts/lane_detection/control.py
# WeGo LIMO Pro를 이용한 주행 코드

import rospy
import time
import threading
from std_msgs.msg import Int32, String, Bool, Float32
from geometry_msgs.msg import Twist
from object_msgs.msg import ObjectArray
from limo_base.msg import LimoStatus
from dynamic_reconfigure.server import Server
from limo_application.cfg import controlConfig

import math


class LimoController:
    '''
        차선 인식, 횡단보도 인식, LiDAR 기반 장애물 인식, YOLO 기반 신호등 및 표지판 인식
        위 기능을 통합한 전체 주행 코드
        Private Params --> control_topic_name
        < Subscriber >
        limo_status (LimoStatus) --> LIMO의 Motion Model 확인용
        /limo/lane_x (Int32) --> 인식된 차선 (카메라 좌표계 x)
        /limo/crosswalk_y (Int32) --> 인식된 횡단보도 (카메라 좌표계 y)
        /limo/traffic_light (String) --> YOLO 기반 인식 결과
        /limo/lidar_warning (String) --> LiDAR 기반 장애물 검출 결과
        < Publisher >
        /cmd_vel (Twist) --> Default 출력, LIMO 제어를 위한 Topic
    '''
    def __init__(self):
        rospy.init_node('limo_control', anonymous=True)
        self.LIMO_WHEELBASE = 0.2
        self.distance_to_ref = 0
        self.crosswalk_detected = False

	self.redline_detected = False

	self.is_force_disabled = False

	self.redline_detected = False

	self.is_red_disabled = False

	self.state = 0

	#bscho modified
	self.wall_velocity = 0
	self.wall_angular = 0

        self.yolo_object = "green"
        self.e_stop = "Safe"
        self.is_pedestrian_stop_available = True
        self.pedestrian_stop_time = 5.0
        self.pedestrian_stop_last_time = rospy.Time.now().to_sec()
        self.yolo_object_last_time = rospy.Time.now().to_sec()
        self.bbox_size = [0, 0]
        self.limo_mode = "ackermann"
        srv = Server(controlConfig, self.reconfigure_callback)
        rospy.Subscriber("limo_status", LimoStatus, self.limo_status_callback)
        rospy.Subscriber("/limo/lane_x", Int32, self.lane_x_callback)
        rospy.Subscriber("/limo/crosswalk_y", Int32, self.crosswalk_y_callback)
        rospy.Subscriber("/limo/yolo_object", ObjectArray, self.yolo_object_callback)
        rospy.Subscriber("/limo/lidar_warning", String, self.lidar_warning_callback)
	rospy.Subscriber("/limo/redline_detection", Bool, self.redline_callback)
	
	#bsho modified
        rospy.Subscriber("/limo/wall_speed", Float32, self.wall_speed_callback)
	rospy.Subscriber("/limo/wall_angle", Float32, self.wall_angle_callback)

        self.drive_pub = rospy.Publisher(rospy.get_param("~control_topic_name", "/cmd_vel"), Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.03), self.drive_callback)

    # return float
    def calcTimeFromDetection(self, _last_detected_time):
        '''
            마지막 검출 시간부터 흐른 시간 확인하는 함수
        '''
        return rospy.Time.now().to_sec() - _last_detected_time

    # ==============================================
    #               Callback Functions
    # ==============================================

    # bscho modiifed
    def wall_speed_callback(self, _data):
	self.wall_speed = _data.data

    def wall_angle_callback(self, _data):
	self.wall_angle = _data.data





    def limo_status_callback(self, _data):
        '''
            LIMO의 상태가 Ackermann인지, Differential인지 확인하여, 모드 변경
            최종 출력의 angular.z 값이 달라지므로, 이와 같은 처리가 필요
        '''
        if _data.motion_mode == 1:
            if self.limo_mode == "ackermann":
                pass
            else:
                self.limo_mode = "ackermann"
                rospy.loginfo("Mode Changed --> Ackermann")
        else:
            if self.limo_mode == "diff":
                pass
            else:
                self.limo_mode = "diff"
                rospy.loginfo("Mode Changed --> Differential Drive")

    def lidar_warning_callback(self, _data):
        '''
            장애물 유무 저장
        '''
        self.e_stop = _data.data

    def yolo_object_callback(self, _data):
        '''
            신호등 또는 표지판 상태 저장
            중간에 일부 끊어질 수 있으므로, 마지막으로 인식된 시간도 함께 저장
        '''
        if len(_data.Objects) == 0:
            pass
        else:
            self.yolo_object = _data.Objects[0].class_name
            self.yolo_object_last_time = rospy.Time.now().to_sec()
            self.bbox_size = [_data.Objects[0].xmin_ymin_xmax_ymax[2] - _data.Objects[0].xmin_ymin_xmax_ymax[0], _data.Objects[0].xmin_ymin_xmax_ymax[3] - _data.Objects[0].xmin_ymin_xmax_ymax[1]]  # 왼쪽 상단 x y, 오른쪽 하단 x, y

    def lane_x_callback(self, _data):
        '''
            실제 기준 좌표와 검출된 차선과의 거리 저장
        '''
        if _data.data == -1:
            self.distance_to_ref = 0
        else:
            self.distance_to_ref = self.REF_X - _data.data

    def redline_callback(self, _data):
	self.redline_detected = _data.data

    def crosswalk_y_callback(self, _data):
        '''
            횡단보도 검출 여부 및 거리 저장
        '''
        if 0 < _data.data <= 50:
            self.crosswalk_detected = True
            self.crosswalk_distance = _data.data
	    rospy.loginfo("HB distance = {}".format(self.crosswalk_distance))

        else:
            self.crosswalk_detected = False
            self.crosswalk_distance = _data.data
            rospy.loginfo("HB distance = {}".format(self.crosswalk_distance))

	

    def reconfigure_callback(self, _config, _level):
        '''
            Dynamic_Reconfigure를 활용
            차량 제어 속도 (BASE_SPEED)
            횡방향 제어 Gain (LATERAL_GAIN)
            차선 기준 좌표 (카메라 픽셀 좌표계 기준) (REF_X)
        '''
        self.BASE_SPEED = _config.base_speed
        self.LATERAL_GAIN = float(_config.lateral_gain * 0.001)
        self.REF_X = _config.reference_lane_x
        self.PEDE_STOP_WIDTH = _config.pedestrian_width_min
        return _config

    def force_crosswalk_disable(self):
        """
        5초 동안 crosswalk_detected 상태를 강제로 False로 유지.
        """
        self.is_force_disabled = True  # 강제 비활성화 상태 플래그 설정
	threading.Timer(10, self.restore_crosswalk_detection).start() 
        

    def restore_crosswalk_detection(self):
        """
        5초 후 crosswalk_detected 감지 상태 복구.
        """
        self.is_force_disabled = False  # 강제 비활성화 플래그 해제
        rospy.loginfo("Crosswalk detection restored.")

    def force_redline_disable(self):
        """
        10초 동안 redline_detected 상태를 강제로 False로 유지.
        """
        self.is_red_disabled = True  # 강제 비활성화 상태 플래그 설정
	threading.Timer(10, self.restore_redline_detection).start()

    def restore_redline_detection(self):
        """
        5초 후 redline_detected 감지 상태 복구.
        """
        self.is_red_disabled = False  # 강제 비활성화 플래그 해제
        rospy.loginfo("redline detection restored.")

    def drive_callback(self, _event):
        '''
            입력된 데이터를 종합하여,
            속도 및 조향을 조절하여 최종 cmd_vel에 Publish
        '''
        if self.yolo_object != "green" and self.calcTimeFromDetection(self.yolo_object_last_time) > 3.0:
            self.yolo_object = "green"
            self.bbox_size = [0, 0]

        if self.calcTimeFromDetection(self.pedestrian_stop_last_time) > 20.0:
            self.is_pedestrian_stop_available = True

        drive_data = Twist()
        drive_data.angular.z = self.distance_to_ref * self.LATERAL_GAIN
        rospy.loginfo("OFF_CENTER, Lateral_Gain = {}, {}".format(self.distance_to_ref, self.LATERAL_GAIN))
        rospy.loginfo("Bbox Size = {}, Bbox_width_min = {}".format(self.bbox_size, self.PEDE_STOP_WIDTH))
	
        try:
	    if self.redline_detected and not self.is_red_disabled:
		self.state += 1
		rospy.logwarn("redline Detected!!!!!!!")
		self.is_red_disabled = True
	        self.force_redline_disable()

	    if self.state == 0:
                if self.e_stop == "Warning":
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = 0.0
                    rospy.logwarn("Obstacle Detected, Stop!")

                elif self.yolo_object == "yellow" or self.yolo_object == "red":
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = 0.0
                    rospy.logwarn("Traffic light is Red or Yellow, Stop!")

                elif self.crosswalk_detected and not self.is_force_disabled:
                    drive_data.linear.x = 0.0
                    rospy.logwarn("Crosswalk Detected, Stop!")
		    self.drive_pub.publish(drive_data)  # 즉시 멈춤 명령
		
		    time.sleep(5)
		
 		    self.crosswalk_detected = False  # 감지 상태 False로 변경
    		    rospy.logwarn("Crosswalk detection disabled for 5 seconds.")
    		    self.force_crosswalk_disable()

                elif self.yolo_object == "slow":
                    drive_data.linear.x = self.BASE_SPEED / 2
                    rospy.logwarn("Slow Traffic Sign Detected, Slow Down!")

                elif self.yolo_object == "pedestrian" and self.is_pedestrian_stop_available and self.bbox_size[0] > self.PEDE_STOP_WIDTH:
                    drive_data.linear.x = 0.0
                    drive_data.angular.z = 0.0
                    self.is_pedestrian_stop_available = False
                    self.pedestrian_stop_last_time = rospy.Time.now().to_sec()
                    rospy.logwarn("Pedestrian Traffic Sign Detected, Stop {} Seconds!".format(self.pedestrian_stop_time))
                    rospy.sleep(rospy.Duration(self.pedestrian_stop_time))

                else:
                    drive_data.linear.x = self.BASE_SPEED
                    rospy.loginfo("All Clear, Just Drive!")

                if self.limo_mode == "diff":
                    self.drive_pub.publish(drive_data)
                elif self.limo_mode == "ackermann":
                    if drive_data.linear.x == 0:
                        drive_data.angular.z = 0
                    else:
                        drive_data.angular.z = \
                            math.tan(drive_data.angular.z / 2) * drive_data.linear.x / self.LIMO_WHEELBASE
                        # 2를 나눈 것은 Differential과 GAIN비율을 맞추기 위함
                        self.drive_pub.publish(drive_data)

	    elif self.state == 1:
		drive_data.linear.x = self.wall_speed
		drive_data.angular.z = self.wall_angle
		self.drive_pub.publish(drive_data)

	    elif self.state == 2:
                drive_data.linear.x = 0.0
                rospy.logwarn("END!")
		self.drive_pub.publish(drive_data)  # 즉시 멈춤 명령
		
		time.sleep(1000)
	    
        except Exception as e:
            rospy.logwarn(e)


def run():
    new_class = LimoController()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")

