# S/N : XYZARIS0V3P2402N01
# Robot IP : 192.168.1.185
# code_version : 3.1.5.2


#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
#
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
#   1. git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
#   2. cd xArm-Python-SDK
#   3. python setup.py install
"""
import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm import version
from xarm.wrapper import XArmAPI

from threading import Thread, Event
import socket
import json
import os

# 카메라 관련 import
import cv2
import cv2.aruco as aruco
import mediapipe as mp
import torch
from ultralytics import YOLO
import cvzone
from cvzone.HandTrackingModule import HandDetector
from collections import defaultdict

import numpy as np

from hi5_sound import Hi5_Sound

import rclpy
from rclpy.node import Node

class RobotMain(Node):
    """Robot Main Class"""

    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._vars = {}
        self._funcs = {}
        self._robot_init()
        self.state = 'stopped'

        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500

        self.motion_num = 0
        self.capsule_position = 0 
        self.return_capsule_position = 0
        self.detect_sealing = 0
        self.dondurma_detected = False
        self.dondurma_someone = True

        self.invaded = 0

        self.xy = None

        self.a = False

        self.json_message = None
        self.icecream = None
        self.topping = None
        self.topping_list = None

        self.tracking_position =[]
        self.tracking_angle = None
        self.tracking_cnt = 0
        self.tracking_dic = defaultdict(list)

        # self.position_home = [180, -42.1, 7.4, 186.7, 41.5, 0] #angle
 
        self.position_sealing_check = [-136.8, 71.5, 307.6, 69.6, -73.9, -59] #Linear
        
        self.position_icecream_with_topping = [168.7, 175.6, 359.5, 43.9, 88.3, 83.3] #Linear
        self.position_icecream_no_topping = [48.4, -13.8, 36.3, 193.6, 42.0, -9.2] #angle
        self.position_jig_A_serve = [-258.7, -136.4, 208.2, 43.4, 88.7, -72.2] #Linear
        self.position_jig_B_serve = [-166.8, -126.5, 200.9, -45.2, 89.2, -133.6] #Linear
        self.position_jig_C_serve = [-63.1, -138.2, 199.5, -45.5, 88.1, -112.1] #Linear


        #--------Hi5----------
        # self.position_home = [180, -45, 0, 180, 45, 0] #angle
        # self.position_home = [180.00002, -24.999982, 14.999978, 180.00002, 50.000021, 0.0]
        self.position_home = [180.00002, -30, 10, 180.00002, 50.000021, 0.0] #angle

        self.position_icecream = [243.1, 134.7, 300, -59.6, 88.5, 29.5] #linear
        # self.position_finish = [270, -45, 0, 180, 45, 0] #angle
        # 270, -35, 4.5, 180, 50, 0

        # self.position_finish = [270.000001, -24.999982, 14.999978, 180.00002, 50.000021, 0.0] #angle
        self.position_finish = [270.000001, -24.999982, 14.999978, 180.00002, 50.000021, 0.0] #angle
        self.position_jig_A_grab = [-255.2, -133.8, 200, 68.3, 86.1, -47.0] #linear
        self.position_jig_B_grab = [-152.3, -127.0, 200, 4.8, 89.0, -90.7] #linear
        self.position_jig_C_grab = [-76.6, -144.6, 200, 5.7, 88.9, -50.1] #linear
        self.position_topping_A = [-194.485367, 165.352158, 300, 15.641404, 89.68411, 143.873541] #Linear
        self.position_topping_B = [-133.771973, 141.502975, 300, 53.655951, 89.68411, 143.873541] #Linear
        self.position_topping_C = [-63.588264, 163.637115, 300, 90, 89.68411, 143.873541] #Linear
        self.position_capsule_grab = [234.2, 129.8, 464.5, -153.7, 87.3, -68.7] #Linear
        self.position_capsule_place = [234.9, 135.9, 465.9, 133.6, 87.2, -142.1] #Linear
        self.position_before_capsule_place = self.position_capsule_place.copy()
        self.position_before_capsule_place[2] += 25
        self.position_cup_grab = [214.0, -100.2, 145.5, -25.6, -88.5, 97.8] #linear
        self.Dondurma_start_position = [0.0, -500, 285, 54.735632, 89.999981, -35.264406] #linear
        self.Dondurma_end_position = [0.0, -180, 285, 54.735632, 89.999981, -35.264406] #linear


        # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # Register count changed callback
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code,
                                                                                                 self._arm.connected,
                                                                                                 self._arm.state,
                                                                                                 self._arm.error_code,
                                                                                                 ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1],
                                       ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    def position_reverse_sealing_fail(self, linear_jig_position = [-257.3, -138.3, 192.1, 68.3, 86.1, -47.0]):
        reverse_position = linear_jig_position.copy()
        reverse_position[2] = reverse_position[2] - 10
        reverse_position[3] = -reverse_position[3]
        reverse_position[4] = -reverse_position[4]
        reverse_position[5] = reverse_position[5] - 180
        return reverse_position

    def socket_connect(self):

        self.HOST = '192.168.1.192'
        self.PORT = 20002
        self.BUFSIZE = 1024
        self.ADDR = (self.HOST, self.PORT)

        # self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.clientSocket.shutdown(1)
            self.clientSocket.close()
        except:
            pass

        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self
        self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.serverSocket.allow_reuse_address = True
        while True:
            try:
                self.serverSocket.bind(self.ADDR)
                print("bind")

                while True:
                    self.serverSocket.listen(1)
                    print(f'[LISTENING] Server is listening on robot_server')
                    time.sleep(1)
                    try:
                        while True:
                            try:
                                self.clientSocket, addr_info = self.serverSocket.accept()
                                print("socket accepted")
                                break
                            except:
                                time.sleep(1)
                                print('except')
                                # break

                        break

                    except socket.timeout:
                        print("socket timeout")

                    except:
                        pass
                break
            except:
                pass
        # self.clientSocket.settimeout(10.0)
        print("accept")
        print("--client info--")
        # print(self.clientSocket)

        self.connected = True
        self.state = 'ready'

        # ------------------- receive msg start -----------
        while self.connected:
            print('loop start')
            time.sleep(0.5)
            try:
                print('waiting')
                self.clientSocket.settimeout(10.0)
                self.recv_msg = self.clientSocket.recv(1024).decode('utf-8')
                # try:
                #    self.recv_msg = self.clientSocket.recv(1024).decode('utf-8')
                # except Exception as e:
                #    self.pprint('MainException: {}'.format(e))
                print('\n' + self.recv_msg)
                if self.recv_msg == '':
                    print('here')
                    # continue
                    # pass
                    # break
                    raise Exception('empty msg')
                self.recv_msg = self.recv_msg.split('/')

                if self.recv_msg[0] == 'app_ping':
                    # print('app_ping received')
                    send_msg = 'robot_ping'
                    now_temp = arm.temperatures
                    now_cur = arm.currents
                    send_msg = [
                        {
                            'type': 'A', 'joint_name': 'Base', 'temperature': now_temp[0],
                            'current': round(now_cur[0], 3) * 100
                        }, {
                            'type': 'B', 'joint_name': 'Shoulder', 'temperature': now_temp[1],
                            'current': round(now_cur[1], 3) * 100
                        }, {
                            'type': 'C', 'joint_name': 'Elbow', 'temperature': now_temp[2],
                            'current': round(now_cur[2], 3) * 100
                        }, {
                            'type': 'D', 'joint_name': 'Wrist1', 'temperature': now_temp[3],
                            'current': round(now_cur[3], 3) * 100
                        }, {
                            'type': 'E', 'joint_name': 'Wrist2', 'temperature': now_temp[4],
                            'current': round(now_cur[4], 3) * 100
                        }, {
                            'type': 'F', 'joint_name': 'Wrist3', 'temperature': now_temp[5],
                            'current': round(now_cur[5], 3) * 100
                        }
                    ]
                    try:
                        time.sleep(0.5)
                        self.clientSocket.send(f'{send_msg}'.encode('utf-8'))
                        print('robot_ping')

                    except Exception as e:
                        self.pprint('MainException: {}'.format(e))
                        print('ping send fail')
                    # send_msg = arm.temperatures
                    if self.state == 'ready':
                        print('STATE : ready for new msg')
                    else:
                        print('STATE : now moving')
                else:
                    self.recv_msg[0] = self.recv_msg[0].replace("app_ping", "")
                    if self.recv_msg[0] in ['breath', 'greet', 'farewell' 'dance_random', 'dance_a', 'dance_b',
                                            'dance_c',
                                            'sleep', 'comeon']:
                        print(f'got message : {self.recv_msg[0]}')
                        if self.state == 'ready':
                            self.state = self.recv_msg[0]
                    elif self.recv_msg[0] == 'robot_script_stop':
                        code = self._arm.set_state(4)
                        if not self._check_code(code, 'set_state'):
                            return
                        sys.exit()
                        self.codeis_alive = False
                        print('program exit')

                    # 픽업존 아이스크림 뺐는지 여부 확인
                    elif self.recv_msg[0].find('icecream_go') >= 0 or self.recv_msg[0].find(
                            'icecream_stop') >= 0 and self.state == 'icecreaming':
                        print(self.recv_msg[0])
                        if self.recv_msg[0].find('icecream_go') >= 0:
                            self.order_msg['makeReq']['latency'] = 'go'
                        else:
                            self.order_msg['makeReq']['latency'] = 'stop'
                            print('000000000000000000000000000000')

                    # 실링 존재 여부 확인
                    if self.recv_msg[0].find('sealing_pass') >= 0 and self.state == 'icecreaming':
                        self.order_msg['makeReq']['sealing'] = 'go'
                        print('socket_go')
                    elif self.recv_msg[0].find('sealing_reject') >= 0 and self.state == 'icecreaming':
                        self.order_msg['makeReq']['sealing'] = 'stop'
                        print('socket_stop')

                    else:
                        # print('else')
                        try:
                            self.order_msg = json.loads(self.recv_msg[0])
                            if self.order_msg['type'] == 'ICECREAM':
                                if self.state == 'ready':
                                    print('STATE : icecreaming')
                                    print(f'Order message : {self.order_msg}')
                                    self.state = 'icecreaming'
                            # else:
                            #    self.clientSocket.send('ERROR : already moving'.encode('utf-8'))
                            else:
                                self.clientSocket.send('ERROR : wrong msg received'.encode('utf-8'))
                        except:
                            pass
                self.recv_msg[0] = 'zzz'

            except Exception as e:
                self.pprint('MainException: {}'.format(e))
                # if e == 'empty msg' :
                #    pass
                # self.connected = False
                print('connection lost')
                while True:
                    time.sleep(2)
                    try:

                        try:
                            self.serverSocket.shutdown(socket.SHUT_RDWR)
                            self.serverSocket.close()
                        except:
                            pass

                        print('socket_making')
                        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                        self.serverSocket.bind(self.ADDR)
                        print("bind")

                        while True:
                            print('listening')
                            self.serverSocket.listen(1)
                            print(f'reconnecting')
                            try:
                                self.clientSocket, addr_info = self.serverSocket.accept()
                                break

                            except socket.timeout:
                                print('socket.timeout')
                                break

                            except:
                                pass
                        break
                    except Exception as e:
                        self.pprint('MainException: {}'.format(e))
                        print('except')
                        # pass

    def socket1_connect(self):

        self.HOST = '192.168.1.7'
        self.PORT = 9005
        self.BUFSIZE = 1024
        self.ADDR = (self.HOST, self.PORT)

        # self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.clientSocket.shutdown(1)
            self.clientSocket.close()
        except:
            pass

        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self
        self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.serverSocket.allow_reuse_address = True
        while True:
            try:
                self.serverSocket.bind(self.ADDR)
                print("bind")

                while True:
                    self.serverSocket.listen(1)
                    print(f'[LISTENING] Server is listening on kiosk_server')
                    time.sleep(1)
                    try:
                        while True:
                            try:
                                self.clientSocket, addr_info = self.serverSocket.accept()
                                print("socket1 accepted")
                                break
                            except:
                                time.sleep(1)
                                print('except')
                                # break

                        break

                    except socket.timeout:
                        print("socket1 timeout")

                    except:
                        pass
                break
            except:
                pass
        # self.clientSocket.settimeout(10.0)
        print("accept")
        print("--client info--")
        # print(self.clientSocket)

        self.connected = True
        self.state = 'ready'

        # ------------------- receive msg start -----------
        while self.connected:
            print('loop start')
            time.sleep(0.5)
            try:
                print('waiting')
                self.clientSocket.settimeout(10.0)
                self.recv_msg = self.clientSocket.recv(1024).decode('utf-8')

                print('\n' + self.recv_msg)
                if self.recv_msg == '':
                    print('here')
                    # continue
                    # pass
                    # break
                    raise Exception('empty msg')
                
               
                self.json_message = self.recv_msg


                # JSON 메시지
                # json_message = '''
                # {
                #     "OR": [
                #         {"orderId": "1", "icecream": "choco", "topping":"cereal,oreo,chocoball"}
                #     ]
                # }
                # '''

                # # JSON 메시지 파싱
                data = json.loads(self.json_message)

                # for order in data['OR']:
                #     self.icecream = order['icecream']
                #     self.topping = order['topping']
                #     print(f"icecream: {self.icecream}, topping: {self.topping}")

                #     # 토핑의 갯수 계산
                #     self.topping_list = [t.strip() for t in self.topping.split(',')]
                #     # topping_count = len(topping_list)
                    
                #     print(self.topping_list)
                #     self.a = True

                # 필요한 정보 추출
                self.icecream = data["OR"]["icecream"]
                self.topping = data["OR"]["topping"]
                print(f"icecream: {self.icecream}, topping: {self.topping}")

                self.topping_list = [t.strip() for t in self.topping.split(',')]
                print(self.topping_list)
                
                
                self.clientSocket.send('ERROR : already moving'.encode('utf-8'))

                # if self.recv_msg[0] == 'app_ping':
                    # print('app_ping received')
                send_msg = 'robot_ping'
                now_temp = arm.temperatures
                now_cur = arm.currents
                send_msg = [
                    {
                        'type': 'A', 'joint_name': 'Base', 'temperature': now_temp[0],
                        'current': round(now_cur[0], 3) * 100
                    }, {
                        'type': 'B', 'joint_name': 'Shoulder', 'temperature': now_temp[1],
                        'current': round(now_cur[1], 3) * 100
                    }, {
                        'type': 'C', 'joint_name': 'Elbow', 'temperature': now_temp[2],
                        'current': round(now_cur[2], 3) * 100
                    }, {
                        'type': 'D', 'joint_name': 'Wrist1', 'temperature': now_temp[3],
                        'current': round(now_cur[3], 3) * 100
                    }, {
                        'type': 'E', 'joint_name': 'Wrist2', 'temperature': now_temp[4],
                        'current': round(now_cur[4], 3) * 100
                    }, {
                        'type': 'F', 'joint_name': 'Wrist3', 'temperature': now_temp[5],
                        'current': round(now_cur[5], 3) * 100
                    }
                ]

                try:
                    time.sleep(0.5)
                    for _ in range(0,3):
                        self.clientSocket.send(f'{send_msg}'.encode('utf-8'))
                        time.sleep(3)
                    print('robot_ping')

                except Exception as e:
                    self.pprint('MainException: {}'.format(e))
                    print('ping send fail')
                # send_msg = arm.temperatures
                if self.state == 'ready':
                    print('STATE : ready for new msg')
                else:
                    print('STATE : now moving')

                # else:
                #     self.recv_msg[0] = self.recv_msg[0].replace("app_ping", "")
                #     if self.recv_msg[0] in ['breath', 'greet', 'farewell' 'dance_random', 'dance_a', 'dance_b',
                #                             'dance_c',
                #                             'sleep', 'comeon']:
                #         print(f'got message : {self.recv_msg[0]}')
                #         if self.state == 'ready':
                #             self.state = self.recv_msg[0]
                #     elif self.recv_msg[0] == 'robot_script_stop':
                #         code = self._arm.set_state(4)
                #         if not self._check_code(code, 'set_state'):
                #             return
                #         sys.exit()
                #         self.codeis_alive = False
                #         print('program exit')

                #     # 픽업존 아이스크림 뺐는지 여부 확인
                #     elif self.recv_msg[0].find('icecream_go') >= 0 or self.recv_msg[0].find(
                #             'icecream_stop') >= 0 and self.state == 'icecreaming':
                #         print(self.recv_msg[0])
                #         if self.recv_msg[0].find('icecream_go') >= 0:
                #             self.order_msg['makeReq']['latency'] = 'go'
                #         else:
                #             self.order_msg['makeReq']['latency'] = 'stop'
                #             print('000000000000000000000000000000')

                #     # 실링 존재 여부 확인
                #     if self.recv_msg[0].find('sealing_pass') >= 0 and self.state == 'icecreaming':
                #         self.order_msg['makeReq']['sealing'] = 'go'
                #         print('socket_go')
                #     elif self.recv_msg[0].find('sealing_reject') >= 0 and self.state == 'icecreaming':
                #         self.order_msg['makeReq']['sealing'] = 'stop'
                #         print('socket_stop')

                #     else:
                #         # print('else')
                #         try:
                #             self.order_msg = json.loads(self.recv_msg[0])
                #             if self.order_msg['type'] == 'ICECREAM':
                #                 if self.state == 'ready':
                #                     print('STATE : icecreaming')
                #                     print(f'Order message : {self.order_msg}')
                #                     self.state = 'icecreaming'
                #             # else:
                #             #    self.clientSocket.send('ERROR : already moving'.encode('utf-8'))
                #             else:
                #                 self.clientSocket.send('ERROR : wrong msg received'.encode('utf-8'))
                #         except:
                #             pass
                # self.recv_msg[0] = 'zzz'

            except Exception as e:
                self.pprint('MainException: {}'.format(e))
                # if e == 'empty msg' :
                #    pass
                # self.connected = False
                print('connection lost')
                while True:
                    time.sleep(2)
                    try:

                        try:
                            self.serverSocket.shutdown(socket.SHUT_RDWR)
                            self.serverSocket.close()
                        except:
                            pass

                        print('socket_making')
                        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                        self.serverSocket.bind(self.ADDR)
                        print("bind")

                        while True:
                            print('listening')
                            self.serverSocket.listen(1)
                            print(f'reconnecting')
                            try:
                                self.clientSocket, addr_info = self.serverSocket.accept()
                                break

                            except socket.timeout:
                                print('socket.timeout')
                                break

                            except:
                                pass
                        break
                    except Exception as e:
                        self.pprint('MainException: {}'.format(e))
                        print('except')
                        # pass

    # =================================  motion  =======================================
    ''' # def motion_grab_capsule(self):
    # def motion_grab_capsule(self):

    #     code = self._arm.set_cgpio_analog(0, 5)
    #     if not self._check_code(code, 'set_cgpio_analog'):
    #         return
    #     code = self._arm.set_cgpio_analog(1, 5)
    #     if not self._check_code(code, 'set_cgpio_analog'):
    #         return

    #     # Joint Motion
    #     self._angle_speed = 100
    #     self._angle_acc = 100

    #     self._tcp_speed = 100
    #     self._tcp_acc = 1000

    
    #     # code = self._arm.close_lite6_gripper()
    #     # if not self._check_code(code, 'close_lite6_gripper'):
    #     #     return
    #     # time.sleep(1)
    #     # code = self._arm.open_lite6_gripper()
    #     # if not self._check_code(code, 'open_lite6_gripper'):
    #     #     return
    #     # time.sleep(1)
    #
    #     code = self._arm.stop_lite6_gripper()
    #     if not self._check_code(code, 'stop_lite6_gripper'):
    #         return
    #     time.sleep(0.5)

    #     try:
    #         self.clientSocket.send('motion_grab_capsule_start'.encode('utf-8'))
    #     except:
    #         print('socket error')

    #     # code = self._arm.set_servo_angle(angle=[175.4, 28.7, 23.8, 84.5, 94.7, -5.6], speed=self._angle_speed,
    #     #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
    #     # if not self._check_code(code, 'set_servo_angle'):
    #     #    return

    #     if self.order_msg['makeReq']['jigNum'] in ['A']:
    #         # code = self._arm.set_servo_angle(angle=[166.1, 30.2, 25.3, 75.3, 93.9, -5.4], speed=self._angle_speed,
    #         #                                  mvacc=self._angle_acc, wait=True, radius=0.0)
    #         # if not self._check_code(code, 'set_servo_angle'):
    #         #     return
    #         pass
    #     else:

    #         code = self._arm.set_servo_angle(angle=[176, 31.7, 31, 76.7, 91.2, -1.9], speed=self._angle_speed,
    #                                          mvacc=self._angle_acc, wait=True, radius=0.0)
    #         if not self._check_code(code, 'set_servo_angle'):
    #             return
    #         # code = self._arm.set_servo_angle(angle=[166.1, 30.2, 25.3, 75.3, 93.9, -5.4], speed=self._angle_speed,
    #         #                                  mvacc=self._angle_acc, wait=False, radius=20.0)
    #         # if not self._check_code(code, 'set_servo_angle'):
    #         #     return


    #     code = self._arm.open_lite6_gripper()
    #     if not self._check_code(code, 'open_lite6_gripper'):
    #         return
    #     time.sleep(1)

    #     if self.order_msg['makeReq']['jigNum'] == 'A':
    #         code = self._arm.set_servo_angle(angle=[179.5, 33.5, 32.7, 113.0, 93.1, -2.3], speed=self._angle_speed,
    #                                          mvacc=self._angle_acc, wait=False, radius=20.0)
    #         if not self._check_code(code, 'set_servo_angle'):
    #             return
    #         # code = self._arm.set_position(*[-255.4, -139.3, 193.5, -12.7, 87.2, -126.1], speed=self._tcp_speed,
    #         #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
    #         # if not self._check_code(code, 'set_position'):
    #         #    return

    #         code = self._arm.set_position(*self.position_jig_A_grab, speed=self._tcp_speed,
    #                                       mvacc=self._tcp_acc, radius=0.0, wait=True)
    #         if not self._check_code(code, 'set_position'):
    #             return

    #     elif self.order_msg['makeReq']['jigNum'] == 'B':

    #         code = self._arm.set_position(*self.position_jig_B_grab, speed=self._tcp_speed,
    #                                       mvacc=self._tcp_acc, radius=0.0, wait=True)
    #         if not self._check_code(code, 'set_position'):
    #             return

    #     elif self.order_msg['makeReq']['jigNum'] == 'C':
    #         code = self._arm.set_servo_angle(angle=[182.6, 27.8, 27.7, 55.7, 90.4, -6.4], speed=self._angle_speed,
    #                                          mvacc=self._angle_acc, wait=False, radius=20.0)
    #         if not self._check_code(code, 'set_servo_angle'):
    #             return
    #         # code = self._arm.set_position(*[-76.6, -144.6, 194.3, 5.7, 88.9, -50.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
    #         # if not self._check_code(code, 'set_position'):
    #         #    return
    #         code = self._arm.set_position(*self.position_jig_C_grab, speed=self._tcp_speed,
    #                                       mvacc=self._tcp_acc, radius=0.0, wait=True)
    #         if not self._check_code(code, 'set_position'):
    #             return

    #     code = self._arm.close_lite6_gripper()
    #     if not self._check_code(code, 'close_lite6_gripper'):
    #         return

    #     time.sleep(1)
    #     if self.order_msg['makeReq']['jigNum'] == 'C':
    #         code = self._arm.set_position(z=150, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
    #                                       wait=False)
    #         if not self._check_code(code, 'set_position'):
    #             return
    #         self._tcp_speed = 200
    #         self._tcp_acc = 1000
    #         code = self._arm.set_tool_position(*[0.0, 0.0, -90.0, 0.0, 0.0, 0.0], speed=self._tcp_speed,
    #                                            mvacc=self._tcp_acc, wait=False)
    #         if not self._check_code(code, 'set_position'):
    #             return
    #     else:
    #         code = self._arm.set_position(z=100, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
    #                                       wait=False)
    #         if not self._check_code(code, 'set_position'):
    #             return

    #     self._angle_speed = 180
    #     self._angle_acc = 500

    #     if self.order_msg['makeReq']['sealing'] in ['yes']:
    #         code = self._arm.set_servo_angle(angle=[145, -18.6, 10.5, 97.5, 81.4, 145], speed=self._angle_speed,
    #                                          mvacc=self._angle_acc, wait=False, radius=30.0)
    #         if not self._check_code(code, 'set_servo_angle'):
    #             return
    #     else :
    #         code = self._arm.set_servo_angle(angle=[146.1, -10.7, 10.9, 102.7, 92.4, 24.9], speed=self._angle_speed,
    #                                         mvacc=self._angle_acc, wait=True, radius=0.0)
    #         if not self._check_code(code, 'set_servo_angle'):
    #             return
    #     try:
    #         self.clientSocket.send('motion_grab_capsule_finish'.encode('utf-8'))
    #     except:
    #         print('socket error')
    '''
    
    ''' # def motion_check_sealing(self):
    def motion_check_sealing(self):
        print('sealing check')
        self._angle_speed = 200
        self._angle_acc = 200
        self.clientSocket.send('motion_sheck_sealing'.encode('utf-8'))
        code = self._arm.set_position(*self.position_sealing_check, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
    '''

    ''' # def motion_grab_cup(self):
    def motion_grab_cup(self):
        try:
            self.clientSocket.send('motion_grab_cup_start'.encode('utf-8'))
        except:
            print('socket error')

        code = self._arm.set_position(*[233.4, 10.3, 471.1, -172.2, 87.3, -84.5], speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)

        if self.order_msg['makeReq']['cupNum'] in ['A', 'B']:
            code = self._arm.set_servo_angle(angle=[-2.8, -2.5, 45.3, 119.8, -79.2, -18.8], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            # code = self._arm.set_position(*[193.8, -100.2, 146.6, 135.9, -86.0, -55.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*[195.0, -96.5, 200.8, -168.0, -87.1, -110.5], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=10.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            # code = self._arm.set_position(*[195.0, -96.5, 145.8, -168.0, -87.1, -110.5], speed=self._tcp_speed,
            #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            # code = self._arm.set_position(*[195.5, -96.6, 145.6, 179.0, -87.0, -97.1], speed=self._tcp_speed,
            #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            # code = self._arm.set_position(*[214.0, -100.2, 145.0, -25.6, -88.5, 95.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*self.position_cup_grab, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(2)

        code = self._arm.set_position(z=120, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                      wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_servo_angle(angle=[2.9, -31.0, 33.2, 125.4, -30.4, -47.2], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        try:
            self.clientSocket.send('motion_grab_cup_finish'.encode('utf-8'))
        except:
            print('socket error')

        time.sleep(0.5)
    '''

    ''' # def motion_topping(self):
    def motion_topping(self):
        try:
            self.clientSocket.send('motion_topping_start'.encode('utf-8'))
        except:
            print('socket error')

        print('send')

        if self.order_msg['makeReq']['topping'] == '1':
            code = self._arm.set_servo_angle(angle=[36.6, -36.7, 21.1, 85.6, 59.4, 44.5], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            if self.order_msg['makeReq']['jigNum'] == 'C':
                code = self._arm.set_position(*self.position_topping_C, speed=self._tcp_speed,
                                              mvacc=self._tcp_acc, radius=0.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_cgpio_digital(2, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_position(z=20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                              wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_pause_time(int(self.order_msg['makeReq']['toppingAmount']) - 3)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_pause_time(3)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(2, 0, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return

                code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc,
                                              relative=True, wait=False)
                if not self._check_code(code, 'set_position'):
                    return

            elif self.order_msg['makeReq']['jigNum'] in ['B']:
                code = self._arm.set_servo_angle(angle=[55.8, -48.2, 14.8, 86.1, 60.2, 58.7], speed=self._angle_speed,
                                                 mvacc=self._angle_acc, wait=False, radius=20.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                # code = self._arm.set_servo_angle(angle=[87.5, -48.2, 13.5, 125.1, 44.5, 46.2], speed=self._angle_speed,
                #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
                # if not self._check_code(code, 'set_servo_angle'):
                #    return
                code = self._arm.set_servo_angle(angle=self.position_topping_B, speed=self._angle_speed,
                                                 mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_position(z=20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                              wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_pause_time(int(self.order_msg['makeReq']['toppingAmount']) - 4)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_pause_time(4)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc,
                                              relative=True, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_servo_angle(angle=[87.5, -48.2, 13.5, 125.1, 44.5, 46.2], speed=self._angle_speed,
                                                 mvacc=self._angle_acc, wait=False, radius=10.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                code = self._arm.set_position(*[43.6, 137.9, 350.1, -92.8, 87.5, 5.3], speed=self._tcp_speed,
                                              mvacc=self._tcp_acc, radius=10.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return

            elif self.order_msg['makeReq']['jigNum'] == 'A':
                code = self._arm.set_position(*self.position_topping_A, speed=self._tcp_speed,
                                              mvacc=self._tcp_acc, radius=0.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_pause_time(int(self.order_msg['makeReq']['toppingAmount']) - 1)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_pause_time(1)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_servo_angle(angle=[130.0, -33.1, 12.5, 194.3, 51.0, 0.0], speed=self._angle_speed,
                                                 mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                code = self._arm.set_position(*[-38.2, 132.2, 333.9, -112.9, 86.3, -6.6], speed=self._tcp_speed,
                                              mvacc=self._tcp_acc, radius=10.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_position(*[43.6, 137.9, 350.1, -92.8, 87.5, 5.3], speed=self._tcp_speed,
                                              mvacc=self._tcp_acc, radius=10.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
            # code = self._arm.set_position(*[165.1, 162.9, 362.5, -31.7, 86.6, 9.5], speed=self._tcp_speed,
            #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*self.position_icecream_with_topping, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        else:
            # code = self._arm.set_servo_angle(angle=[45.8, -17.9, 33.5, 186.9, 41.8, -7.2], speed=self._angle_speed,
            #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
            # if not self._check_code(code, 'set_servo_angle'):
            #    return
            code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_servo_angle(angle=self.position_icecream_no_topping, speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        try:
            self.clientSocket.send('motion_topping_finish'.encode('utf-8'))
        except:
            print('socket error')

        time.sleep(0.5)
    '''

    ''' # def motion_make_icecream(self):
    def motion_make_icecream(self):
        try:
            self.clientSocket.send('motion_make_icecream_start'.encode('utf-8'))
        except:
            print('socket error')
        if self.order_msg['makeReq']['topping'] == '1':
            time.sleep(5)
        else:
            time.sleep(8)
        try:
            self.clientSocket.send('motion_icecreaming_1'.encode('utf-8'))
        except:
            print('socket error')
        time.sleep(4)
        code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                      wait=True)
        if not self._check_code(code, 'set_position'):
            return
        try:
            self.clientSocket.send('motion_icecreaming_2'.encode('utf-8'))
        except:
            print('socket error')
        time.sleep(4)
        code = self._arm.set_position(z=-10, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                      wait=True)
        if not self._check_code(code, 'set_position'):
            return
        if not self._check_code(code, 'set_pause_time'):
            return
        try:
            self.clientSocket.send('motion_icecreaming_3'.encode('utf-8'))
        except:
            print('socket error')
        code = self._arm.set_position(z=-50, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                      wait=True)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        try:
            self.clientSocket.send('motion_make_icecream_finish'.encode('utf-8'))
        except:
            print('socket error')
        time.sleep(0.5)
    '''

    def motion_serve(self):
        try:
            self.clientSocket.send('motion_serve_start'.encode('utf-8'))
        except:
            print('socket error')
        code = self._arm.set_servo_angle(angle=[18.2, -12.7, 8.3, 90.3, 88.1, 23.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[146.9, -12.7, 8.3, 91.0, 89.3, 22.1], seed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        self._tcp_speed = 100
        self._tcp_acc = 1000

        if self.order_msg['makeReq']['jigNum'] == 'A':
            # code = self._arm.set_position(*[-251.2, -142.1, 213.7, -28.1, 88.8, -146.0], speed=self._tcp_speed,
            #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            # code = self._arm.set_position(*[-250.3, -138.3, 213.7, 68.3, 86.1, -47.0], speed=self._tcp_speed,
            #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*self.position_jig_A_serve, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

            code = self._arm.set_position(z=-18, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                          wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-256.2, -126.6, 210.1, -179.2, 77.2, 66.9], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(0.5)
            code = self._arm.set_position(*[-242.8, -96.3, 210.5, -179.2, 77.2, 66.9], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # code = self._arm.set_tool_position(*[0.0, 0.0, -30, 0.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*[-189.7, -26.0, 193.3, -28.1, 88.8, -146.0], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif self.order_msg['makeReq']['jigNum'] == 'B':

            code = self._arm.set_position(*self.position_jig_B_serve, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(z=-13, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                          wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-165.0, -122.7, 200, -178.7, 80.7, 92.5], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(0.5)
            code = self._arm.set_position(*[-165.9, -81.9, 200, -178.7, 80.7, 92.5], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # code = self._arm.set_tool_position(*[0.0, 0.0, -30, 0.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*[-168.5, -33.2, 192.8, -92.9, 86.8, -179.3], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        elif self.order_msg['makeReq']['jigNum'] == 'C':
            # code = self._arm.set_servo_angle(angle=[171.0, 13.7, 13.5, 73.9, 92.3, -2.9], speed=self._angle_speed,
            #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
            # if not self._check_code(code, 'set_servo_angle'):
            #    return
            code = self._arm.set_servo_angle(angle=[177.6, 0.2, 13.5, 70.0, 94.9, 13.8], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*self.position_jig_C_serve, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(z=-12, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                          wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-75, -132.8, 208, -176.8, 76.1, 123.0], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(0.5)
            code = self._arm.set_position(*[-92.0, -107.5, 208, -176.8, 76.1, 123.0], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # code = self._arm.set_tool_position(*[0.0, 0.0, -30, 0.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*[-98.1, -52.1, 191.4, -68.4, 86.4, -135.0], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        try:
            self.clientSocket.send('motion_serve_finish'.encode('utf-8'))
        except:
            print('socket error')
        time.sleep(0.5)
        code = self._arm.set_servo_angle(angle=[169.6, -8.7, 13.8, 85.8, 93.7, 19.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        self._tcp_speed = 100
        self._tcp_acc = 1000

    ''' # def motion_trash_capsule(self):
    def motion_trash_capsule(self):
        try:
            self.clientSocket.send('motion_trash_start'.encode('utf-8'))
        except:
            print('socket error')
        self._angle_speed = 150
        self._angle_acc = 300
        code = self._arm.set_servo_angle(angle=[51.2, -8.7, 13.8, 95.0, 86.0, 17.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[-16.2, -19.3, 42.7, 82.0, 89.1, 55.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        code = self._arm.set_servo_angle(angle=[-19.9, -19.1, 48.7, 87.2, 98.7, 60.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_position(*[222.8, 0.9, 470.0, -153.7, 87.3, -68.7], speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        # code = self._arm.set_position(*[234.2, 129.8, 464.5, -153.7, 87.3, -68.7], speed=self._tcp_speed,
        #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #    return
        code = self._arm.set_position(*self.position_capsule_grab, speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.set_position(z=30, radius=-1, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                      wait=True)
        if not self._check_code(code, 'set_position'):
            return
        self._tcp_speed = 100
        self._tcp_acc = 1000
        code = self._arm.set_position(*[221.9, -5.5, 500.4, -153.7, 87.3, -68.7], speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        self._angle_speed = 60
        self._angle_acc = 100
        code = self._arm.set_servo_angle(angle=[-10.7, -2.4, 53.5, 50.4, 78.1, 63.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # code = self._arm.set_position(*[217.1, 125.8, 250.1, 170.8, 50.2, -99.2], speed=self._tcp_speed,
        #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #    return
        self._angle_speed = 160
        self._angle_acc = 1000
        code = self._arm.set_servo_angle(angle=[18.0, 11.2, 40.4, 90.4, 58.7, -148.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        # time.sleep(2)
        code = self._arm.set_servo_angle(angle=[25.2, 15.2, 42.7, 83.2, 35.0, -139.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[18.0, 11.2, 40.4, 90.4, 58.7, -148.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[25.2, 15.2, 42.7, 83.2, 35.0, -139.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        self._angle_speed = 120
        self._angle_acc = 1000
        code = self._arm.set_servo_angle(angle=[28.3, -9.0, 12.6, 85.9, 78.5, 20.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # code = self._arm.set_servo_angle(angle=[116.8, -9.0, 10.0, 107.1, 78.3, 20.0], speed=self._angle_speed,
        #                                mvacc=self._angle_acc, wait=False, radius=30.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #    return
        code = self._arm.set_servo_angle(angle=[149.3, -9.4, 10.9, 114.7, 69.1, 26.1], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # code = self._arm.set_servo_angle(angle=[179.0, -17.9, 17.7, 176.4, 61.3, 0.0], speed=self._angle_speed,
        #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #    return
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        try:
            self.clientSocket.send('motion_trash_finish'.encode('utf-8'))
        except:
            print('socket error')
        time.sleep(0.5)
    '''

    def motion_dance_a(self):  # designed 'poke'
        try:
            self.clientSocket.send('dance_a_start'.encode('utf-8'))
        except:
            print('socket error')

        self._angle_speed = 60
        self._angle_acc = 300
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(3)):
            if not self.is_alive:
                break
            code = self._arm.set_servo_angle(angle=[212.0, -21.0, 112.0, 207.0, -0.8, 7.3], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[212.0, -38.0, 100.3, 180.4, -6.4, 6.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        '''
        code = self._arm.set_servo_angle(angle=[329.0, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(3)):
            if not self.is_alive:
                break
            code = self._arm.set_servo_angle(angle=[329.0, -21.0, 112.0, 207.0, -0.8, 7.3], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[329.0, -38.0, 100.3, 180.4, -6.4, 6.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        '''
        self._angle_speed = 60
        self._angle_acc = 200
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

    def motion_dance_b(self):  # designed 'shake'
        try:
            self.clientSocket.send('dance_b_start'.encode('utf-8'))
        except:
            print('socket error')

        self._angle_speed = 70
        self._angle_acc = 200
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(4)):
            if not self.is_alive:
                break
            code = self._arm.set_servo_angle(angle=[220.7, -39.1, 67.0, 268.3, -40.0, -91.8], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[183.0, -39.1, 102.7, 220.0, -11.6, -140.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

    def motion_dance_c(self):  # designed '빙글빙글'
        try:
            self.clientSocket.send('dance_c_start'.encode('utf-8'))
        except:
            print('socket error')

        self._angle_speed = 150
        self._angle_acc = 700
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(3)):
            if not self.is_alive:
                break
            t1 = time.monotonic()
            code = self._arm.set_servo_angle(angle=[180.0, 70.0, 250.0, 173.1, 0.0, -135.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, -70.0, 110.0, 180.0, 0.0, 135.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            interval = time.monotonic() - t1
            if interval < 0.01:
                time.sleep(0.01 - interval)
        code = self._arm.set_servo_angle(angle=[180.0, 70.0, 250.0, 173.1, 0.0, -135.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        while True:
            try:
                self.clientSocket.send('dance_c_finish'.encode('utf-8'))
                break
            except:
                print('socket error')

    def motion_come_on(self):  # designed '컴온컴온
        try:
            self.clientSocket.send('comeon_start'.encode('utf-8'))
        except:
            print('socket error')

        self._angle_speed = 80
        self._angle_acc = 400
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[180.0, 70.0, 220.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(2)):
            if not self.is_alive:
                break
            t1 = time.monotonic()
            code = self._arm.set_servo_angle(angle=[180.0, 70.0, 220.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 62.0, 222.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 55.0, 222.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 45.0, 222.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 35.0, 224.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 25.0, 224.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 15.0, 226.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 5.0, 226.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 0.0, 228.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 5.0, 230.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 20.0, 226.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 35.0, 226.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 45.0, 228.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 55.0, 226.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 65.0, 224.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 70.0, 222.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            interval = time.monotonic() - t1
            if interval < 0.01:
                time.sleep(0.01 - interval)
        code = self._arm.set_servo_angle(angle=[180.0, 65.0, 222.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        while True:
            try:
                self.clientSocket.send('comeon_finish'.encode('utf-8'))
                break
            except:
                print('socket error')

    def motion_greet(self):
        try:
            self.clientSocket.send('greet_start'.encode('utf-8'))
        except:
            print('socket error')

        self._angle_speed = 100
        self._angle_acc = 350

        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 181.5, -1.9, -92.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 180.9, -28.3, -92.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 185.4, 30.8, -94.9], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 180.9, -28.3, -92.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 185.4, 30.8, -94.9], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 180.9, -28.3, -92.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 185.4, 30.8, -94.9], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        try:
            self.clientSocket.send('motion_greet finish'.encode('utf-8'))
        except:
            print('socket error')
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 181.5, -1.9, -92.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        while True:
            try:
                self.clientSocket.send('motion_greet_finish'.encode('utf-8'))
                break
            except:
                print('socket error')

    def motion_breath(self):
        pass

    def motion_sleep(self):  # designed 'sleep'
        try:
            self.clientSocket.send('sleep_start'.encode('utf-8'))
        except:
            print('socket error')

        for i in range(int(1)):
            if not self.is_alive:
                break
            for i in range(int(2)):
                if not self.is_alive:
                    break
                self._angle_speed = 20
                self._angle_acc = 200
                code = self._arm.set_servo_angle(angle=[179.0, -17.7, 29.0, 177.8, 43.8, -1.4], speed=self._angle_speed,
                                                 mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                self._angle_speed = 5
                self._angle_acc = 5
                code = self._arm.set_servo_angle(angle=[179.0, -10.2, 24.0, 178.2, 39.2, -2.0], speed=self._angle_speed,
                                                 mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
            self._angle_speed = 30
            self._angle_acc = 300
            code = self._arm.set_servo_angle(angle=[179.0, -17.7, 29.0, 177.8, 43.8, -1.4], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            for i in range(int(3)):
                if not self.is_alive:
                    break
                self._angle_speed = 180
                self._angle_acc = 1000
                code = self._arm.set_servo_angle(angle=[179.0, -17.7, 29.0, 199.8, 43.4, -11.0],
                                                 speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                code = self._arm.set_servo_angle(angle=[179.0, -17.7, 29.0, 157.3, 43.2, 12.7], speed=self._angle_speed,
                                                 mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
            self._angle_speed = 20
            self._angle_acc = 200
            code = self._arm.set_servo_angle(angle=[179.0, -17.7, 29.0, 177.8, 43.8, -1.4], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(2)
            if not self._check_code(code, 'set_pause_time'):
                return
        while True:
            try:
                self.clientSocket.send('sleep_finish'.encode('utf-8'))
                break
            except:
                print('socket error')

    def motion_clean_mode(self):
        pass

    def pin_off(self):
        self.clientSocket.send('pin_off_start'.encode('utf-8'))
        # cup_dispenser_up
        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        # press_up
        code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        self.clientSocket.send('pin_off_finish'.encode('utf-8'))

    def pin_test(self):
        time.sleep(3)
        code = self._arm.set_servo_angle(angle=[179.0, -17.7, 29.0, 177.8, 43.8, -1.4], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        time.sleep(2)
        code = self._arm.set_servo_angle(angle=[179.0, -17.7, 83.3, 177.8, 43.8, -1.4], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        time.sleep(3)
        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        time.sleep(3)
        code = self._arm.set_cgpio_analog(1, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return


    # Robot Main Run
    def run(self):
        try:
            while self.is_alive:
                # Joint Motion
                if self.state == 'icecreaming':
                    # --------------icecream start--------------------
                    try:
                        self.clientSocket.send('icecream_start'.encode('utf-8'))
                    except:
                        print('socket error')
                    time.sleep(int(self.order_msg['makeReq']['latency']))
                    self.motion_home()
                    # self.check_gripper()
                    while True:
                        if self.order_msg['makeReq']['latency'] in ['go', 'stop']:
                            break
                        time.sleep(0.2)
                    if self.order_msg['makeReq']['latency'] in ['go']:
                        self.motion_grab_capsule()
                        if self.order_msg['makeReq']['sealing'] in ['yes']:
                            self.motion_check_sealing()
                            try:
                                self.clientSocket.send('sealing_check'.encode('utf-8'))
                            except:
                                pass
                            count = 0
                            while True:
                                # if sealing_check request arrives or 5sec past
                                if self.order_msg['makeReq']['sealing'] in ['go', 'stop'] or count >= 5:
                                    print(self.order_msg['makeReq']['sealing'])
                                    break
                                time.sleep(0.2)
                                count += 0.2
                        if self.order_msg['makeReq']['sealing'] in ['go'] or self.order_msg['makeReq']['sealing'] not in ['yes', 'stop']:
                            #print('sealing_pass')
                            self.motion_place_capsule()
                            self.motion_grab_cup()
                            self.motion_topping()
                            self.motion_make_icecream()
                            self.motion_serve()
                            self.motion_trash_capsule()
                            self.motion_home()
                            print('icecream finish')
                            while True:
                                try:
                                    self.clientSocket.send('icecream_finish'.encode('utf-8'))
                                    break
                                except:
                                    time.sleep(0.2)
                                    print('socket_error')
                        else:
                            self.motion_place_fail_capsule()
                            self.motion_home()
                            self.clientSocket.send('icecream_cancel'.encode('utf-8'))
                            self.order_msg['makeReq']['sealing'] = ''
                    else:
                        while True:
                            try:
                                self.clientSocket.send('icecream_cancel'.encode('utf-8'))
                                break
                            except:
                                print('socket error')
                        self.order_msg['makeReq']['latency'] = 0
                    self.state = 'ready'

                elif self.state == 'test':
                    try:
                        self.clientSocket.send('test_start'.encode('utf-8'))
                    except:
                        print('socket error')
                    # self.motion_home()
                    # self.motion_grab_cup()
                    # self.motion_serve()

                elif self.state == 'greet':
                    self.motion_greet()
                    self.motion_home()
                    while True:
                        try:
                            self.clientSocket.send('greet_finish'.encode('utf-8'))
                            break
                        except:
                            print('socket error')
                            time.sleep(0.2)
                    print('greet finish')
                    self.state = 'ready'

                elif self.state == 'dance_random':
                    dance_num = random.randrange(1, 4)
                    if dance_num == 1:
                        self.motion_dance_a()
                    elif dance_num == 2:
                        self.motion_dance_b()
                    elif dance_num == 3:
                        self.motion_dance_c()
                    while True:
                        try:
                            self.clientSocket.send('dance_random_finish'.encode('utf-8'))
                            break
                        except:
                            print('socket error')
                            time.sleep(0.2)
                    self.state = 'ready'

                elif self.state == 'dance_a':
                    self.motion_dance_a()
                    self.motion_home()
                    while True:
                        try:
                            self.clientSocket.send('dance_a_finish'.encode('utf-8'))
                            break
                        except:
                            print('socket error')
                            time.sleep(0.2)
                    self.state = 'ready'

                elif self.state == 'dance_b':
                    self.motion_dance_b()
                    self.motion_home()
                    while True:
                        try:
                            self.clientSocket.send('dance_b_finish'.encode('utf-8'))
                            break
                        except:
                            print('socket error')
                            time.sleep(0.2)
                    self.state = 'ready'

                elif self.state == 'dance_c':
                    self.motion_dance_c()
                    self.motion_home()
                    # self.clientSocket.send('dance_c_finish'.encode('utf-8'))
                    self.state = 'ready'

                elif self.state == 'breath':
                    try:
                        self.clientSocket.send('breath_start'.encode('utf-8'))
                        time.sleep(5)
                        self.clientSocket.send('breath_finish'.encode('utf-8'))
                    except:
                        print('socket error')

                elif self.state == 'sleep':
                    self.motion_sleep()
                    self.motion_home()
                    while True:
                        try:
                            self.clientSocket.send('sleep_finish'.encode('utf-8'))
                            break
                        except:
                            print('socket error')
                            time.sleep(0.2)
                    self.state = 'ready'

                elif self.state == 'comeon':
                    print('come_on start')
                    self.motion_come_on()
                    # self.motion_home()
                    self.state = 'ready'

                elif self.state == 'clean_mode':
                    try:
                        self.clientSocket.send('clean_mode_start'.encode('utf-8'))
                    except:
                        print('socket error')
                    self.state = 'ready'

                    code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self.state = 'ready'

                elif self.state == 'clean_mode_end':
                    code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self.state = 'ready'


                elif self.state == 'ping':
                    print('ping checked')
                    # self.motion_home()
                    self.state = 'ready'

                else:
                    pass

                # self.state = 'ready'
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)

    def joint_state(self):
        while self.is_alive:
            print(f'joint temperature : {arm.temperatures}')
            time.sleep(0.5)
            print(f'joint current : {arm.currents}')
            time.sleep(10)

    
#-----------------------------------------------Hi5-cam-----------------------------------------------
    def check_capsule_position(self):
        # 마커를 생성하는 데 사용된 딕셔너리를 로드합니다.
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        # 기본값을 사용하여 탐지기 매개변수를 초기화합니다.
        parameters = aruco.DetectorParameters()
        # 각 마커의 마지막 감지 시간을 저장할 딕셔너리 초기화
        last_seen = {}
        # 웹캠을 통해 비디오 캡처를 시작합니다.
        cap = cv2.VideoCapture(2)
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            # ROI 영역 정의
            roi_x = 270
            roi_y = 0
            roi_width = 270
            roi_height = 60
            roi = frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]
            # ROI 영역에서 마커 감지
            corners, ids, rejectedImgPoints = aruco.detectMarkers(roi, aruco_dict, parameters=parameters)
            # 현재 시간 기록
            current_time = time.time()
            # 감지된 마커를 ROI 이미지에 그립니다.
            if ids is not None and len(ids) > 0:
                aruco.drawDetectedMarkers(roi, corners, ids)
                print(f"Detected ArUco markers: {ids.flatten()}")
                # 감지된 모든 마커의 마지막 감지 시간을 업데이트합니다.
                for id in ids.flatten():
                    last_seen[id] = current_time
                # 모든 마커에 대해 마지막 감지 시간 확인
                for id, last_time in last_seen.items():
                    if current_time - last_time > 3:
                        print(f"Action executed for marker ID {id} (not moved for 5 seconds)")
                        # 5초 동안 감지되지 않은 경우 수행할 동작

                        if id == 0:
                            self.capsule_position = 1
                        elif id == 1:
                            self.capsule_position = 2
                        else:
                            self.capsule_position = 3

                        # last_seen 딕셔너리에서 삭제하여 중복 동작 방지
                        del last_seen[id]
                        break
            else:
                print("No markers detected")
                print(self.capsule_position)
            # 원본 프레임에 ROI 경계를 그립니다.
            cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (255, 0, 0), 2)
            # 감지된 마커가 있는 이미지를 표시합니다.
            cv2.imshow('frame', frame)
            # 'q' 키를 누르면 루프를 종료합니다.
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            if cv2.waitKey(1) & self.capsule_position >= 1:
                break
        # 캡처 객체와 모든 창을 해제 및 닫습니다.
        cap.release()
        cv2.destroyAllWindows()

    def check_sealing(self):
        
        def detect_star_shape(image, canny_thresh1, canny_thresh2):
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                edged = cv2.Canny(blurred, canny_thresh1, canny_thresh2)
                contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                print(f"len(contours):",len(contours))
                if len(contours) == 1 or len(contours) == 2:
                    contour = contours[0]
                    epsilon = 0.02 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    print(f"len(approx):",len(approx))
                    if len(approx) >= 8 and len(approx) <= 12:  # Assuming a star has approximately 10 points
                        return True, approx, edged, len(contours)
                return False, None, edged, len(contours)
        def nothing(x):
            pass
        cap = cv2.VideoCapture(2)
    
        cv2.namedWindow('Frame')
        cv2.createTrackbar('Canny Thresh1', 'Frame', 50, 255, nothing)
        cv2.createTrackbar('Canny Thresh2', 'Frame', 112, 255, nothing)
        cv2.createTrackbar('Brightness', 'Frame', 50, 100, nothing)
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            # Get current position of the brightness trackbar
            brightness = cv2.getTrackbarPos('Brightness', 'Frame')
            # Apply brightness adjustment
            frame = cv2.convertScaleAbs(frame, alpha=1, beta=(brightness - 50) * 2)
            # Define the ROI (Region of Interest) starting at (425, 75)
            roi_x = 464
            roi_y = 118
            roi = frame[roi_y:roi_y + 35, roi_x:roi_x + 35]
            # Get current positions of the trackbars
            canny_thresh1 = cv2.getTrackbarPos('Canny Thresh1', 'Frame')
            canny_thresh2 = cv2.getTrackbarPos('Canny Thresh2', 'Frame')
            star_detected, star_contour, edged, contour_count = detect_star_shape(roi, canny_thresh1, canny_thresh2)
            
            # Draw the ROI rectangle
            cv2.rectangle(frame, (roi_x, roi_y), (roi_x + 35, roi_y + 35), (255, 0, 0), 2)
            cv2.putText(frame, f"Contours: {contour_count}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            # Display the edged image in the ROI
            frame[roi_y:roi_y + 35, roi_x:roi_x + 35] = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)
            cv2.imshow('Frame', frame)

            if star_detected:
                # Offset the contour points to the ROI position
                star_contour += [roi_x, roi_y]
                cv2.drawContours(frame, [star_contour], -1, (0, 255, 0), 3)
                cv2.putText(frame, "Star detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                self.detect_sealing = 1
            else:
                cv2.putText(frame, "Star not detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                self.detect_sealing = 0

            if cv2.waitKey(1) & self.detect_sealing >= 1:
                break
        # 캡처 객체와 모든 창을 해제 및 닫습니다.
        cap.release()
        cv2.destroyAllWindows()
        

    def hand_warning(self):
        def nothing(x):
            pass
        cap = cv2.VideoCapture(2)
        fgbg = cv2.createBackgroundSubtractorMOG2()
        # ROI 영역 정의
        roi_x = 52
        roi_y = 0
        roi_width = 500
        roi_height = 310
        # Mediapipe 손 감지 초기화
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(max_num_hands=3, min_detection_confidence=0.3)
        # 트랙바를 위한 윈도우 생성
        cv2.namedWindow('Frame')
        cv2.createTrackbar('Threshold', 'Frame', 0, 20000, nothing)  # 초기값 0, 최대값 20000
        # 로봇팔 동작 제어 함수 (예시)
        def control_robot_arm(action):
            if action == "stop":
                ### 여기다가 로봇팔 동작 멈추는 코드
                arm.set_state(3)
                print("Robot arm stopped.")
            elif action == "start":
                ### 여기다가 로봇팔이 동작하는 코드
                arm.set_state(0)
                print("Robot arm started.")
        # 초기 상태를 로봇팔 동작 중으로 설정
        robot_arm_active = True
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            # 트랙바에서 현재 threshold 값을 가져옴
            threshold = cv2.getTrackbarPos('Threshold', 'Frame')
            # 배경 차분을 사용하여 전경 마스크 생성
            fgmask = fgbg.apply(frame)
            # 특정 영역에 대한 침입 감지
            roi = fgmask[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]
            intrusion_detected = np.sum(roi) > threshold
            # Mediapipe를 사용하여 손 감지
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hands.process(frame_rgb)
            hand_in_roi = False
            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    for landmark in hand_landmarks.landmark:
                        # 손가락 랜드마크 좌표 계산
                        x = int(landmark.x * frame.shape[1])
                        y = int(landmark.y * frame.shape[0])
                        # 손가락 랜드마크가 ROI 영역 내에 있는지 확인
                        if roi_x < x < roi_x + roi_width and roi_y < y < roi_y + roi_height and intrusion_detected:
                            hand_in_roi = True
                            cv2.putText(frame, "Warning: Intrusion detected!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            break
                    # 손 랜드마크 그리기
                    mp.solutions.drawing_utils.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            # 원본 프레임에 ROI 경계를 그립니다.
            cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (255, 0, 0), 2)
            # 손이 ROI에 있는지에 따라 로봇팔 제어
            if hand_in_roi:
                if robot_arm_active:
                    control_robot_arm("stop")
                    robot_arm_active = False
            else:
                if not robot_arm_active:
                    control_robot_arm("start")
                    robot_arm_active = True
            # 감지된 마커가 있는 이미지를 표시합니다.
            cv2.imshow('Frame', frame)
            cv2.imshow('Foreground Mask', fgmask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()


    def tash_Cartesian(self):

        # YOLOv8 모델 로드
        model = YOLO('new_custom_m.pt')  # 모델 경로를 설정하세요.

        # 모델을 GPU로 이동
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        if torch.cuda.is_available():
            model.to(device)

        # GPU 사용 여부 출력
        print(f'Using device: {device}')

        # 캘리브레이션 데이터를 사용하여 카메라 매트릭스와 왜곡 계수를 설정
        camera_matrix = np.array([[474.51901407, 0, 302.47811758],
                                [0, 474.18970657, 250.66191453],
                                [0, 0, 1]])
        dist_coeffs = np.array([[-0.06544764, -0.07654065, -0.00761827, -0.00279316, 0.08062307]])

        # 1 픽셀당 1.55145mm 단위 변환 비율
        pixel_to_mm_ratio = 1.55145

        # 로봇 좌표계의 중앙값 (픽셀 단위)
        robot_origin_x = 295
        robot_origin_y = 184

        # 비디오 캡처 초기화 (카메라 ID는 0으로 설정, 필요 시 변경 가능)
        cap = cv2.VideoCapture(2)

        # 트랙바 콜백 함수 (아무 동작도 하지 않음)
        def nothing(x):
            pass

        # 트랙바를 위한 윈도우 생성
        cv2.namedWindow('Detection Results')
        cv2.createTrackbar('Confidence', 'Detection Results', 50, 100, nothing)  # 기본값 50, 범위 0-100

        # 객체 좌표를 저장할 딕셔너리
        object_coords = {}

        # 유사한 객체를 인식할 때 사용할 거리 임계값
        similarity_threshold = 50

        # 제어 모드 플래그
        control_mode = False

        def find_similar_object(center_x, center_y, label, object_coords, threshold):
            for obj_id, coords in object_coords.items():
                if obj_id.startswith(label) and coords:
                    avg_x = sum([coord[0] for coord in coords]) / len(coords)
                    avg_y = sum([coord[1] for coord in coords]) / len(coords)
                    distance = np.sqrt((avg_x - center_x) ** 2 + (avg_y - center_y) ** 2)
                    if distance < threshold:
                        return obj_id
            return None

        while True:
            # 트랙바에서 현재 Confidence 값 가져오기
            confidence_threshold = cv2.getTrackbarPos('Confidence', 'Detection Results') / 100.0

            # 프레임 캡처
            ret, frame = cap.read()
            if not ret:
                break

            # 왜곡 보정
            undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

            if not control_mode:
                # 객체 검출 수행
                results = model(undistorted_frame, conf=confidence_threshold)
                
                # 바운딩 박스 그리기
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        cls_id = int(box.cls)
                        label = model.names[cls_id]
                        confidence = box.conf.item()  # 신뢰도 추출

                        # 'cup' 및 'star' 객체만 추적
                        if label in ['cup', 'star'] and confidence >= confidence_threshold:
                            bbox = box.xyxy[0].tolist()  # 바운딩 박스를 리스트로 변환
                            x1, y1, x2, y2 = map(int, bbox)
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2

                            cv2.rectangle(undistorted_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                            # 텍스트 위치 계산
                            text = f'{label} {confidence:.2f}'
                            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)

                            # 텍스트가 바운딩 박스 내부, 오른쪽 상단에 표시되도록 위치 조정
                            text_x = x2 - text_width if x2 - text_width > 0 else x1
                            text_y = y1 - 2 if y1 - 2 > text_height else y1 + text_height + 2

                            # 텍스트 배경 상자 그리기
                            cv2.rectangle(undistorted_frame, (text_x, text_y - text_height - baseline), (text_x + text_width, text_y + baseline), (0, 255, 0), cv2.FILLED)

                            # 텍스트 그리기
                            cv2.putText(undistorted_frame, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

                            # 유사한 객체 찾기
                            similar_object_id = find_similar_object(center_x, center_y, label, object_coords, similarity_threshold)
                            
                            if similar_object_id:
                                object_id = similar_object_id
                            else:
                                object_id = f'{label}_{len(object_coords)}'
                                object_coords[object_id] = []
                                print(f"새 객체 발견: {object_id}")

                            # 좌표 추가, 20개까지만 저장
                            if len(object_coords[object_id]) < 20:
                                object_coords[object_id].append((center_x, center_y))
                                print(f"{object_id}의 좌표 추가됨: ({center_x}, {center_y})")
                                print(f"{object_id}의 좌표 개수: {len(object_coords[object_id])}")

                # 모든 객체가 20개의 좌표를 수집하면 제어 모드로 전환
                if all(len(coords) >= 20 for coords in object_coords.values()):
                    control_mode = True
                    print("제어 모드로 전환")

            else:
                # 모든 객체의 평균 좌표 계산
                avg_coords = {}
                for object_id, coords in object_coords.items():
                    avg_x = sum([coord[0] for coord in coords]) / 20
                    avg_y = sum([coord[1] for coord in coords]) / 20
                    avg_coords[object_id] = (avg_x, avg_y)

                # 카메라 화면의 중심 좌표
                center_x_cam, center_y_cam = 320, 240

                # 객체들을 중심으로부터의 거리 순으로 정렬 (가장 먼 객체부터)
                sorted_objects = sorted(avg_coords.items(), key=lambda item: np.sqrt((item[1][0] - center_x_cam) ** 2 + (item[1][1] - center_y_cam) ** 2), reverse=True)

                # 모든 객체 ID와 거리 순서대로 번호 표시
                for idx, (object_id, (avg_x, avg_y)) in enumerate(sorted_objects, start=1):
                    distance = np.sqrt((avg_x - center_x_cam) ** 2 + (avg_y - center_y_cam) ** 2)
                    display_text = f'{idx}. {object_id} ({distance:.2f} px)'
                    cv2.putText(undistorted_frame, display_text, (10, 30 + 30 * idx), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                # 가장 먼 객체부터 순차적으로 제어
                for object_id, (avg_x, avg_y) in sorted_objects:
                    print(f"{object_id}의 평균 좌표 계산됨: ({avg_x:.2f}, {avg_y:.2f})")

                    # 중심으로부터의 거리 계산
                    distance = np.sqrt((avg_x - center_x_cam) ** 2 + (avg_y - center_y_cam) ** 2)
                    print(f"{object_id}의 중심으로부터 거리 계산됨: {distance:.2f} 픽셀")

                    # 픽셀 좌표를 MM 좌표로 변환
                    robot_coords_mm_x = (avg_x - robot_origin_x) * pixel_to_mm_ratio * -1
                    robot_coords_mm_y = (avg_y - robot_origin_y) * pixel_to_mm_ratio
                    print(f"{object_id}의 로봇 좌표 계산됨: ({robot_coords_mm_x:.2f} mm, {robot_coords_mm_y:.2f} mm)")

                    # 로봇 제어 코드 실행
                    print(f"로봇 제어 코드 실행: {object_id}")


                    self.xy = [robot_coords_mm_x, robot_coords_mm_y]

                    code = self._arm.open_lite6_gripper()
                    if not self._check_code(code, 'open_lite6_gripper'):
                        return
                    time.sleep(2)

                    code = self._arm.set_position(*[self.xy[0], self.xy[1], 400, -179.999963, -0.544826, 88.453969], speed=20,
                                            mvacc=100, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return

                    code = self._arm.set_position(*[self.xy[0], self.xy[1], 250, -179.999963, -0.544826, 88.453969], speed=20,
                                                    mvacc=100, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return
                    
                    code = self._arm.close_lite6_gripper()
                    if not self._check_code(code, 'close_lite6_gripper'):
                        return
                    time.sleep(2)

                    code = self._arm.set_position(*[self.xy[0], self.xy[1], 400, -179.999963, -0.544826, 88.453969], speed=20,
                                                    mvacc=100, radius=0.0, wait=True)
                    if not self._check_code(code, 'set_position'):
                        return


                    # 여기에 로봇 제어 코드를 추가하세요



                # 모든 객체의 제어가 완료되면 객체 리스트 초기화 및 인식 모드로 전환
                object_coords.clear()
                control_mode = False
                print("모든 객체 제어 완료. 인식 모드로 전환.")

            # 결과가 포함된 이미지 표시
            cv2.imshow('Detection Results', undistorted_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


        
    def Dondurma_hand(self):
        self.dondurma_detected = False
        # Initialize webcam
        cap = cv2.VideoCapture(2)
        # Initialize HandDetector
        detector = HandDetector(maxHands=5, detectionCon=0.8)  # 최대 5개의 손을 인식

        # 임계값 설정 (이 값을 조정하여 손이 너무 가까워졌을 때를 결정)
        AREA_THRESHOLD = 100000  # 예시 값, 실제 상황에 맞게 조정 필요

        # 손 ID 저장
        tracked_hand_bbox = None

        while self.dondurma_someone:
            success, img = cap.read()
            if not success:
                break

            # Find hands
            hands, img = detector.findHands(img)

            if hands:
                # 인식된 손들을 면적 기준으로 정렬
                hands.sort(key=lambda x: x['bbox'][2] * x['bbox'][3], reverse=True)
                # print('hand:', hands)

                # 손마다 바운딩 박스를 표시
                for hand in hands:
                    x, y, w, h = hand['bbox']
                    cvzone.putTextRect(img, f'Area: {w * h}', (x, y - 10), scale=1, thickness=2, colorR=(255, 0, 0))
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

                if tracked_hand_bbox is None:
                    # 면적이 가장 큰 손의 바운딩 박스를 저장
                    tracked_hand_bbox = hands[0]['bbox']
                
                # 추적 중인 손을 찾기
                hand_to_track = None
                for hand in hands:
                    if hand['bbox'] == tracked_hand_bbox:
                        hand_to_track = hand
                        break
                
                if hand_to_track:
                    # Get the bounding box of the tracked hand
                    x, y, w, h = hand_to_track['bbox']
                    # Calculate the area of the bounding box
                    area = w * h

                    # 면적이 임계값을 초과하면 경고 메시지 표시
                    if area > AREA_THRESHOLD:
                        cvzone.putTextRect(img, "Oops! Too close, trying to steal the ice cream?", (50, 50), scale=1, thickness=2, colorR=(0, 0, 255))
                        # 여기서 로봇을 제어하는 코드를 추가할 수 있습니다.
                        # 예: 로봇을 n 차 뒤로 이동시키는 코드
                        # move_robot_backward(n)
                        self.dondurma_detected = True

                    # Display the area on the image
                    cvzone.putTextRect(img, f'Tracked Area: {int(area)}', (50, 100), scale=2, thickness=2, colorR=(255, 0, 0))
                    
                    # 추적 중인 손을 강조 표시
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 4)  # 빨간색 두꺼운 사각형
                    cvzone.putTextRect(img, 'Tracking', (x, y - 50), scale=2, thickness=2, colorR=(0, 0, 255))
                    
                else:
                    # 추적 중인 손을 찾을 수 없는 경우, 초기화
                    tracked_hand_bbox = None
            else:
                # 손이 없으면 ID 초기화
                tracked_hand_bbox = None

            # Display the image
            cv2.imshow("Dondurma_hand", img)

            # Break the loop on 'q' key press
            if cv2.waitKey(1) & self.dondurma_detected == True:
                cap.release()
                cv2.destroyAllWindows()
                break
                

        # Release the webcam and destroy all windows
        
    def trash_position(self):
        import logging

        # YOLOv8 모델 로드
        model = YOLO('newjeans.pt')  # 모델 경로를 설정하세요.

        # 모델을 GPU로 이동
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        if torch.cuda.is_available():
            model.to(device)

        # GPU 사용 여부 출력
        print(f'Using device: {device}')

        logging.getLogger('torch').setLevel(logging.ERROR)

        # 캘리브레이션 데이터를 사용하여 카메라 매트릭스와 왜곡 계수를 설정
        camera_matrix = np.array([[474.51901407, 0, 302.47811758],
                                [0, 474.18970657, 250.66191453],
                                [0, 0, 1]])
        dist_coeffs = np.array([[-0.06544764, -0.07654065, -0.00761827, -0.00279316, 0.08062307]])

        # 1 픽셀당 1.55145mm 단위 변환 비율
        pixel_to_mm_ratio = 1.55145

        # 로봇 좌표계의 중앙값 (픽셀 단위)
        robot_origin_x = 295
        robot_origin_y = 184

        # 비디오 캡처 초기화 (카메라 ID는 0으로 설정, 필요 시 변경 가능)
        cap = cv2.VideoCapture(2)

        # 트랙바 콜백 함수 (아무 동작도 하지 않음)
        def nothing(x):
            pass

        # 트랙바를 위한 윈도우 생성
        cv2.namedWindow('Detection Results')
        cv2.createTrackbar('Confidence', 'Detection Results', 50, 100, nothing)  # 기본값 50, 범위 0-100

        # 객체 좌표를 저장할 딕셔너리
        object_coords = {}

        # 유사한 객체를 인식할 때 사용할 거리 임계값
        similarity_threshold = 50

        # 제어 모드 플래그
        control_mode = False

        def find_similar_object(center_x, center_y, label, object_coords, threshold):
            for obj_id, coords in object_coords.items():
                if obj_id.startswith(label) and coords:
                    avg_x = sum([coord[0] for coord in coords]) / len(coords)
                    avg_y = sum([coord[1] for coord in coords]) / len(coords)
                    distance = np.sqrt((avg_x - center_x) ** 2 + (avg_y - center_y) ** 2)
                    if distance < threshold:
                        return obj_id
            return None

        flag = 1
        while True:
            # 트랙바에서 현재 Confidence 값 가져오기
            confidence_threshold = cv2.getTrackbarPos('Confidence', 'Detection Results') / 100.0

            # 프레임 캡처
            ret, frame = cap.read()
            if not ret:
                break

            # 왜곡 보정
            undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

            if not control_mode:
                # 객체 검출 수행
                results = model(undistorted_frame, conf=confidence_threshold)
                
                # 바운딩 박스 그리기
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        cls_id = int(box.cls)
                        label = model.names[cls_id]
                        confidence = box.conf.item()  # 신뢰도 추출

                        # 'cup' 및 'star' 객체만 추적
                        if label in ['side_cup', 'side_star'] and confidence >= confidence_threshold:
                            bbox = box.xyxy[0].tolist()  # 바운딩 박스를 리스트로 변환
                            x1, y1, x2, y2 = map(int, bbox)
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2

                            cv2.rectangle(undistorted_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                            # 텍스트 위치 계산
                            text = f'{label} {confidence:.2f}'
                            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)

                            # 텍스트가 바운딩 박스 내부, 오른쪽 상단에 표시되도록 위치 조정
                            text_x = x2 - text_width if x2 - text_width > 0 else x1
                            text_y = y1 - 2 if y1 - 2 > text_height else y1 + text_height + 2

                            # 텍스트 배경 상자 그리기
                            cv2.rectangle(undistorted_frame, (text_x, text_y - text_height - baseline), (text_x + text_width, text_y + baseline), (0, 255, 0), cv2.FILLED)

                            # 텍스트 그리기
                            cv2.putText(undistorted_frame, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

                            # 유사한 객체 찾기
                            similar_object_id = find_similar_object(center_x, center_y, label, object_coords, similarity_threshold)
                            
                            if similar_object_id:
                                object_id = similar_object_id
                            else:
                                object_id = f'{label}_{len(object_coords)}'
                                object_coords[object_id] = []
                                print(f"새 객체 발견: {object_id}")

                            # 좌표 추가, 20개까지만 저장
                            if len(object_coords[object_id]) < 20:
                                object_coords[object_id].append((center_x, center_y))
                                print(f"{object_id}의 좌표 추가됨: ({center_x}, {center_y})")
                                print(f"{object_id}의 좌표 개수: {len(object_coords[object_id])}")

                # 모든 객체가 20개의 좌표를 수집하면 제어 모드로 전환
                if all(len(coords) >= 20 for coords in object_coords.values()):
                    control_mode = True
                    print("제어 모드로 전환")

            else:
                # 모든 객체의 평균 좌표 계산
                avg_coords = {}
                for object_id, coords in object_coords.items():
                    avg_x = sum([coord[0] for coord in coords]) / 20
                    avg_y = sum([coord[1] for coord in coords]) / 20
                    avg_coords[object_id] = (avg_x, avg_y)

                # 카메라 화면의 중심 좌표
                center_x_cam, center_y_cam = 320, 240

                # 객체들을 중심으로부터의 거리 순으로 정렬 (가장 먼 객체부터)
                sorted_objects = sorted(avg_coords.items(), key=lambda item: np.sqrt((item[1][0] - center_x_cam) ** 2 + (item[1][1] - center_y_cam) ** 2), reverse=True)

                # 모든 객체 ID와 거리 순서대로 번호 표시
                for idx, (object_id, (avg_x, avg_y)) in enumerate(sorted_objects, start=1):
                    distance = np.sqrt((avg_x - center_x_cam) ** 2 + (avg_y - center_y_cam) ** 2)
                    display_text = f'{idx}. {object_id} ({distance:.2f} px)'
                    cv2.putText(undistorted_frame, display_text, (10, 30 + 30 * idx), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                # 가장 먼 객체부터 순차적으로 제어
                for object_id, (avg_x, avg_y) in sorted_objects:
                    print(f"{object_id}의 평균 좌표 계산됨: ({avg_x:.2f}, {avg_y:.2f})")

                    # 중심으로부터의 거리 계산
                    distance = np.sqrt((avg_x - center_x_cam) ** 2 + (avg_y - center_y_cam) ** 2)
                    print(f"{object_id}의 중심으로부터 거리 계산됨: {distance:.2f} 픽셀")

                    # 픽셀 좌표를 MM 좌표로 변환
                    robot_coords_mm_x = (avg_x - robot_origin_x) * pixel_to_mm_ratio * -1
                    robot_coords_mm_y = (avg_y - robot_origin_y) * pixel_to_mm_ratio
                    print(f"{object_id}의 로봇 좌표 계산됨: ({robot_coords_mm_x:.2f} mm, {robot_coords_mm_y:.2f} mm)")

                    self.tracking_dic[object_id].append(robot_coords_mm_x,robot_coords_mm_y)

                    # if flag:
                    #     self.tracking_position.append(robot_coords_mm_x)
                    #     self.tracking_position.append(robot_coords_mm_y)
                    #     flag = 0

                    # self.tracking_position = [robot_coords_mm_x, robot_coords_mm_y]
                    # self.tracking_cnt = len(sorted_objects)

                    # 로봇 제어 코드 실행
                    # 여기에 로봇 제어 코드를 추가하세요
                    print(f"로봇 제어 코드 실행: {object_id}")

                # 모든 객체의 제어가 완료되면 객체 리스트 초기화 및 인식 모드로 전환
                object_coords.clear()
                control_mode = False
                print("모든 객체 제어 완료. 인식 모드로 전환.")

            # 결과가 포함된 이미지 표시
            cv2.imshow('Detection Results', undistorted_frame)

            # if cv2.waitKey(1) & 0xFF == ord('q'):
            if cv2.waitKey(1) & len(self.tracking_dic) > 0:
                time.sleep(3)
                break

        cap.release()
        cv2.destroyAllWindows()




    def trash_angle(self):
        # 세그멘테이션 모델 로드
        model = YOLO('newjeans.pt')  # 세그멘테이션 모델 경로를 설정하세요.

        # 모델을 GPU로 이동
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        if torch.cuda.is_available():
            model.to(device)

        # GPU 사용 여부 출력
        print(f'Using device: {device}')

        # 캘리브레이션 데이터를 사용하여 카메라 매트릭스와 왜곡 계수를 설정
        camera_matrix = np.array([[474.51901407, 0, 302.47811758],
                                [0, 474.18970657, 250.66191453],
                                [0, 0, 1]])
        dist_coeffs = np.array([[-0.06544764, -0.07654065, -0.00761827, -0.00279316, 0.08062307]])

        # 비디오 캡처 초기화 (카메라 ID는 0으로 설정, 필요 시 변경 가능)
        cap = cv2.VideoCapture(2)

        # 트랙바 콜백 함수 (아무 동작도 하지 않음)
        def nothing(x):
            pass

        # 트랙바를 위한 윈도우 생성
        cv2.namedWindow('Segmentation Results')
        cv2.createTrackbar('Confidence', 'Segmentation Results', 50, 100, nothing)  # 기본값 50, 범위 0-100

        def calculate_angle(pt1, pt2):
            angle = np.arctan2(pt2[1] - pt1[1], pt2[0] - pt1[0]) * 180.0 / np.pi
            return angle

        while True:
            # 트랙바에서 현재 Confidence 값 가져오기
            confidence_threshold = cv2.getTrackbarPos('Confidence', 'Segmentation Results') / 100.0

            # 프레임 캡처
            ret, frame = cap.read()
            if not ret:
                break

            # 왜곡 보정
            undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

            # 객체 검출 수행
            results = model(undistorted_frame, conf=confidence_threshold)
            
            for result in results:
                masks = result.masks  # 세그멘테이션 마스크 가져오기
                for mask in masks.data.cpu().numpy():  # CPU 메모리로 복사한 후 numpy 배열로 변환
                    mask = mask.astype(np.uint8)  # numpy 배열을 uint8로 변환
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                    # 디버깅: 세그멘테이션 마스크 표시
                    debug_mask = cv2.cvtColor(mask * 255, cv2.COLOR_GRAY2BGR)
                    cv2.imshow('Debug Mask', debug_mask)

                    if contours:
                        cnt = max(contours, key=cv2.contourArea)
                        rect = cv2.minAreaRect(cnt)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        top_left = box[0]
                        top_right = box[1]
                        bottom_left = box[3]
                        diff_top = abs(top_right[0] - top_left[0])
                        diff_left = abs(bottom_left[1] - top_left[1])
                        cv2.drawContours(undistorted_frame, [box], 0, (0, 255, 0), 2)

                        # 중심점 계산
                        M = cv2.moments(cnt)
                        if M["m00"] != 0:
                            center_x = int(M["m10"] / M["m00"])
                            center_y = int(M["m01"] / M["m00"])
                        else:
                            center_x, center_y = 0, 0

                        center_point = (center_x, center_y)
                        cv2.circle(undistorted_frame, center_point, 5, (255, 0, 0), -1)

                        # 밑면의 중심점 찾기
                        bottom_points = sorted(box, key=lambda pt: pt[1], reverse=True)[:2]
                        bottom_center_x = int((bottom_points[0][0] + bottom_points[1][0]) / 2)
                        bottom_center_y = int((bottom_points[0][1] + bottom_points[1][1]) / 2)
                        bottom_center_point = (bottom_center_x, bottom_center_y)

                        cv2.circle(undistorted_frame, bottom_center_point, 5, (0, 0, 255), -1)
                        cv2.line(undistorted_frame, center_point, bottom_center_point, (0, 255, 255), 2)

                        # 기울기 계산
                        angle = calculate_angle(center_point, bottom_center_point)
                        if angle is not None:
                            if 85 < angle < 90:
                                adjusted_angle = 180 - angle
                                if diff_top > diff_left:
                                    adjusted_angle = angle 
                                else:
                                    adjusted_angle = angle + 90   
                            else:
                                adjusted_angle = 180 - angle
                        print(f"adjusted_angle : {adjusted_angle}")
                        cv2.putText(undistorted_frame, f"Angle: {angle:.2f}", (center_x, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        print(f"Detected angle: {angle:.2f}")

                        self.tracking_angle = adjusted_angle

                        # 세그멘테이션 마스크를 컬러로 변환
                        colored_mask = np.zeros_like(undistorted_frame)
                        colored_mask[mask == 1] = [0, 255, 0]  # Green color for the mask

                        # 원본 이미지에 세그멘테이션 마스크 오버레이
                        overlay = cv2.addWeighted(undistorted_frame, 0.7, colored_mask, 0.3, 0)

                        # 결과가 포함된 이미지 표시
                        cv2.imshow('Segmentation Results', overlay)

            # if cv2.waitKey(1) & 0xFF == ord('q'):
            if cv2.waitKey(1) & (self.tracking_angle != None):
                break

        cap.release()
        cv2.destroyAllWindows()


    def vision(self):
        def detect_star_shape(image, canny_thresh1, canny_thresh2):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edged = cv2.Canny(blurred, canny_thresh1, canny_thresh2)
            contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # print(f"len(contours):",len(contours))
            if len(contours) == 1 or len(contours) == 2:
                contour = contours[0]
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                # print(f"len(approx):",len(approx))
                if len(approx) >= 8 and len(approx) <= 12:  # Assuming a star has approximately 10 points
                    return True, approx, edged, len(contours)
            return False, None, edged, len(contours)

        def nothing(x):
            pass

        def control_robot_arm(action):
            if action == "stop":
                ### 여기다가 로봇팔 동작 멈추는 코드
                arm.set_state(3)
                print("Robot arm stopped.")
            elif action == "start":
                ### 여기다가 로봇팔이 동작하는 코드
                arm.set_state(0)
                print("Robot arm started.")
        # 초기 상태를 로봇팔 동작 중으로 설정

        cap = cv2.VideoCapture(2)

        # YOLOv8 model
        model = YOLO('new_custom_m_freeze8.pt')
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        if torch.cuda.is_available():
            model.to(device)
        print(f'Using device: {device}')

        # Mediapipe hand detection
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7)

        # ArUco marker detection
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters()
        last_seen = {}

        # Trackbars for thresholds and brightness
        cv2.namedWindow('Frame')
        cv2.createTrackbar('Threshold', 'Frame', 0, 20000, nothing)
        cv2.createTrackbar('Canny Thresh1', 'Frame', 50, 255, nothing)
        cv2.createTrackbar('Canny Thresh2', 'Frame', 112, 255, nothing)
        cv2.createTrackbar('Brightness', 'Frame', 50, 100, nothing)
        cv2.createTrackbar('Confidence', 'Frame', 50, 100, nothing)

        # Trackbars for trash detection
        cv2.createTrackbar('Diff_Thresh', 'Frame', 45, 255, nothing)
        cv2.createTrackbar('Hue Min', 'Frame', 0, 179, nothing)
        cv2.createTrackbar('Hue Max', 'Frame', 179, 179, nothing)
        cv2.createTrackbar('Sat Min', 'Frame', 0, 255, nothing)
        cv2.createTrackbar('Sat Max', 'Frame', 255, 255, nothing)
        cv2.createTrackbar('Val Min', 'Frame', 0, 255, nothing)
        cv2.createTrackbar('Val Max', 'Frame', 255, 255, nothing)

        # ROI definitions
        roi_x_large = 52
        roi_y_large = 0
        roi_width_large = 500
        roi_height_large = 310

        roi_x_medium = 270
        roi_y_medium = 0
        roi_width_medium = 270
        roi_height_medium = 60

        roi_x_small = 464
        roi_y_small = 118
        roi_width_small = 35
        roi_height_small = 35

        fgbg = cv2.createBackgroundSubtractorMOG2()

        # 초기 배경 이미지 변수
        initial_gray = None
        post_cleanup_gray = None
        detection_enabled = False
        cleanup_detection_enabled = False
        yolo_detection_enabled = False  # New variable to control YOLO detection
        robot_arm_active = True

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame from camera.")
                break
            
            # Trackbar values
            threshold = cv2.getTrackbarPos('Threshold', 'Frame')
            canny_thresh1 = cv2.getTrackbarPos('Canny Thresh1', 'Frame')
            canny_thresh2 = cv2.getTrackbarPos('Canny Thresh2', 'Frame')
            brightness = cv2.getTrackbarPos('Brightness', 'Frame')
            confidence_threshold = cv2.getTrackbarPos('Confidence', 'Frame') / 100.0
            
            # Trackbar values for trash detection
            diff_thresh = cv2.getTrackbarPos('Diff_Thresh', 'Frame')
            h_min = cv2.getTrackbarPos('Hue Min', 'Frame')
            h_max = cv2.getTrackbarPos('Hue Max', 'Frame')
            s_min = cv2.getTrackbarPos('Sat Min', 'Frame')
            s_max = cv2.getTrackbarPos('Sat Max', 'Frame')
            v_min = cv2.getTrackbarPos('Val Min', 'Frame')
            v_max = cv2.getTrackbarPos('Val Max', 'Frame')
            
            frame = cv2.convertScaleAbs(frame, alpha=1, beta=(brightness - 50) * 2)
            
            fgmask = fgbg.apply(frame)
            
            # Intrusion detection
            roi_large = fgmask[roi_y_large:roi_y_large + roi_height_large, roi_x_large:roi_x_large + roi_width_large]
            intrusion_detected = np.sum(roi_large) > threshold
            
            warning_detected = False  # Initialize warning detected flag
            trash_detected = False  # Initialize trash detection flag

            # Mediapipe hand detection
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hands.process(frame_rgb)
            hand_in_roi = False
            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    for landmark in hand_landmarks.landmark:
                        x = int(landmark.x * frame.shape[1])
                        y = int(landmark.y * frame.shape[0])
                        if roi_x_large < x < roi_x_large + roi_width_large and roi_y_large < y < roi_y_large + roi_height_large and intrusion_detected:
                            warning_detected = True
                            hand_in_roi = True
                            cv2.putText(frame, "Warning: Intrusion detected!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            break


                    # 손 랜드마크 그리기
                    mp.solutions.drawing_utils.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            if hand_in_roi:
                if robot_arm_active:
                    control_robot_arm("stop")
                    robot_arm_active = False
            else:
                if not robot_arm_active:
                    control_robot_arm("start")
                    robot_arm_active = True

            # ArUco marker detection
            roi_medium = frame[roi_y_medium:roi_y_medium + roi_height_medium, roi_x_medium:roi_x_medium + roi_width_medium]
            corners, ids, rejectedImgPoints = aruco.detectMarkers(roi_medium, aruco_dict, parameters=parameters)
            current_time = time.time()
            if ids is not None and len(ids) > 0:
                aruco.drawDetectedMarkers(roi_medium, corners, ids)
                for id in ids.flatten():
                    last_seen[id] = current_time
                for id, last_time in list(last_seen.items()):
                    if current_time - last_time > 3:
                        cv2.putText(frame, f"Action executed for marker ID {id}", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        
                        if id == 0:
                            self.capsule_position = 1
                        elif id == 1:
                            self.capsule_position = 2
                        
                        del last_seen[id]
                        break
            
            # Star shape detection
            small_roi = frame[roi_y_small:roi_y_small + roi_height_small, roi_x_small:roi_x_small + roi_width_small]
            star_detected, star_contour, edged, contour_count = detect_star_shape(small_roi, canny_thresh1, canny_thresh2)
            if star_detected:
                star_contour += [roi_x_small, roi_y_small]
                cv2.drawContours(frame, [star_contour], -1, (0, 255, 0), 3)
                cv2.putText(frame, "Star detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
                self.detect_sealing = 1
            else:
                cv2.putText(frame, "Star not detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
                self.detect_sealing = 0
            # Trash detection
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_gray = cv2.GaussianBlur(frame_gray, (21, 21), 0)
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # 초기값 설정 또는 해제
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                initial_gray = frame_gray
                detection_enabled = True
                yolo_detection_enabled = True  # Enable YOLO detection as well
                print("Initial background set, detection enabled.")
            
            # 검출 활성화된 경우에만 실행
            if detection_enabled and initial_gray is not None and not cleanup_detection_enabled:
                # 초기 이미지와 현재 이미지의 차이 계산
                frame_delta = cv2.absdiff(initial_gray, frame_gray)
                
                # 차이 이미지의 임계값 적용
                _, diff_mask = cv2.threshold(frame_delta, diff_thresh, 255, cv2.THRESH_BINARY)
                diff_mask = cv2.dilate(diff_mask, None, iterations=2)
                
                # Canny 엣지 검출
                edges = cv2.Canny(frame_gray, canny_thresh1, canny_thresh2)
                edges = cv2.dilate(edges, None, iterations=1)
                edges = cv2.bitwise_and(edges, edges, mask=diff_mask)
                
                # HSV 색상 범위에 따른 마스크 생성
                lower_bound = np.array([h_min, s_min, v_min])
                upper_bound = np.array([h_max, s_max, v_max])
                hsv_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
                
                # 탐지 영역 마스크 생성 (마커와 별 인식 영역 제외)
                detection_mask = np.zeros(diff_mask.shape, dtype=np.uint8)
                detection_mask[roi_y_large:roi_y_large + roi_height_large, roi_x_large:roi_x_large + roi_width_large] = 255
                detection_mask[roi_y_medium:roi_y_medium + roi_height_medium, roi_x_medium:roi_x_medium + roi_width_medium] = 0
                detection_mask[roi_y_small:roi_y_small + roi_height_small, roi_x_small:roi_x_small + roi_width_small] = 0
                
                # 최종 마스크 적용
                combined_mask = cv2.bitwise_or(diff_mask, edges)
                combined_mask = cv2.bitwise_and(combined_mask, combined_mask, mask=detection_mask)
                combined_mask = cv2.bitwise_and(combined_mask, hsv_mask)
                
                # 윤곽선 검출
                contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                detected_objects = []
                
                for cnt in contours:
                    if cv2.contourArea(cnt) > 80:  # 최소 면적 기준
                        x, y, w, h = cv2.boundingRect(cnt)
                        # 탐지된 객체가 지정된 탐지 영역 내에 있는지 확인
                        if (roi_x_large <= x <= roi_x_large + roi_width_large and
                            roi_y_large <= y <= roi_y_large + roi_height_large):
                            detected_objects.append((x, y, w, h))
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            trash_detected = True  # Set trash detected flag
            
            # YOLOv8 object detection within the large ROI
            if yolo_detection_enabled:
                yolo_roi = frame[roi_y_large:roi_y_large + roi_height_large, roi_x_large:roi_x_large + roi_width_large]
                results = model(yolo_roi)
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        cls_id = int(box.cls)
                        label = model.names[cls_id]
                        confidence = box.conf.item()
                        if confidence >= confidence_threshold:
                            bbox = box.xyxy[0].tolist()
                            x1, y1, x2, y2 = map(int, bbox)
                            x1 += roi_x_large
                            y1 += roi_y_large
                            x2 += roi_x_large
                            y2 += roi_y_large
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            if label == 'cup' or label == 'star':
                                text = f'{label}'
                                text_x = x1
                                text_y = y1 - 10 if y1 - 10 > 10 else y1 + 10
                                cv2.putText(frame, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            trash_detected = True  # Set trash detected flag
            
            # Draw ROI rectangles
            cv2.rectangle(frame, (roi_x_large, roi_y_large), (roi_x_large + roi_width_large, roi_y_large + roi_height_large), (255, 0, 0), 2)
            cv2.rectangle(frame, (roi_x_medium, roi_y_medium), (roi_x_medium + roi_width_medium, roi_y_medium + roi_height_medium), (0, 255, 0), 2)
            cv2.rectangle(frame, (roi_x_small, roi_y_small), (roi_x_small + roi_width_small, roi_y_small + roi_height_small), (0, 0, 255), 2)
            cv2.putText(frame, f"Contours: {contour_count}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
            
            # Display the edged image in the small ROI
            frame[roi_y_small:roi_y_small + roi_height_small, roi_x_small:roi_x_small + roi_width_small] = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)
            
            # Display "Warning Detected" message if a hand is detected
            if warning_detected:
                cv2.putText(frame, 'WARNING DETECTED!', (10, frame.shape[0] - 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

            # Display "Trash Detected" message
            if trash_detected:
                cv2.putText(frame, 'TRASH DETECTED', (10, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

            cv2.imshow('Frame', frame)
            if key == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
    
#-----------------------------------------------Hi5-motion-----------------------------------------------
    def motion_home(self):
        self.capsule_position = 0 
        self.detect_sealing = 0

        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return

        # press_up
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return

        # Joint Motion
        # self._angle_speed = 80
        # self._angle_acc = 200

        self._angle_speed = 20
        self._angle_acc = 200
        try:
            self.clientSocket.send('motion_home_start'.encode('utf-8'))
        except:
            print('socket error')
        print('motion_home start')

        code = self._arm.set_servo_angle(angle=self.position_home, speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)

        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)
        
        self.motion_num = 0
        print('motion_home finish')
        

    def motion_grab_capsule(self):
        print('motion_grab_capsule start')
        # 컵 추출 준비
        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        
        # Joint Motion
        # self._angle_speed = 100
        # self._angle_acc = 100

        # self._tcp_speed = 100
        # self._tcp_acc = 1000

        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)

        # self.check_capsule_position()
        # self.capsule_position = 1

        print(self.capsule_position)
        while True:
            if self.capsule_position == 1:
                self.return_capsule_position = self.capsule_position
                # 지그 가는 웨이포인트
                code = self._arm.set_servo_angle(angle=[176, 27, 29.9, 76.8, 92, 0], speed=100,
                                                    mvacc=100, wait=False, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                # 지그A 가는 웨이포인트
                code = self._arm.set_servo_angle(angle=[179.5, 28.3, 31.4, 113.1, 91.5, 0], speed=100,
                                                    mvacc=100, wait=True, radius=20.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return

                code = self._arm.set_position(*self.position_jig_A_grab, speed=30,
                                                mvacc=100, radius=0.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                break

            elif self.capsule_position == 2:
                self.return_capsule_position = self.capsule_position
                print("들어옴")
                # 지그 가는 웨이포인트
                code = self._arm.set_servo_angle(angle=[176, 27, 29.9, 76.8, 92, 0], speed=100,
                                                    mvacc=100, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                print("지그 가는 웨이포인트")
                # 지그B 캡슐 그랩
                code = self._arm.set_position(*self.position_jig_B_grab, speed=30,
                                                mvacc=100, radius=0.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return 
                

                print("지그B 캡슐 그랩")
                break

            # elif self.capsule_position == 3:
            #     self.return_capsule_position = 3
            #     # 지그 가는 웨이포인트
            #     code = self._arm.set_servo_angle(angle=[176, 27, 29.9, 76.8, 92, 0], speed=self._angle_speed,
            #                                         mvacc=self._angle_acc, wait=False, radius=0.0)
            #     if not self._check_code(code, 'set_servo_angle'):
            #         return
            #     # 지그C 잡으러가는 웨이포인트
            #     code = self._arm.set_servo_angle(angle=[182.6, 25.3, 27.2, 55.7, 91.5, 0], speed=self._angle_speed,
            #                                         mvacc=self._angle_acc, wait=False, radius=20.0)
            #     if not self._check_code(code, 'set_servo_angle'):
            #         return

            #     code = self._arm.set_position(*self.position_jig_C_grab, speed=self._tcp_speed,
            #                                     mvacc=self._tcp_acc, radius=0.0, wait=True)
            #     if not self._check_code(code, 'set_position'):
            #         return
            #     break

        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gri1pper'):
            return
        time.sleep(1)

        code = self._arm.set_position(z=100, radius=0, speed=100, mvacc=1000, relative=True,
                                        wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        #실링체크 웨이포인트
        code = self._arm.set_servo_angle(angle=[145, -27, 13.4, 95.7, 80.1, 156.4], speed=100,
                                            mvacc=100, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        self.motion_num = 1
        print('motion_grab_capsule finish')
    
    def motion_check_sealing(self):
        print('motion_check_sealing start')

        code = self._arm.set_servo_angle(angle=[179.1, -85.6, 13.1, 182.6, -3.2, 180], speed=100,
                                            mvacc=100, wait=True, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        self.motion_num = 2
        print('motion_check_sealing finish')


    def motion_place_capsule(self):
        print('motion_place_capsule start')
        angle_speed = 120
        angle_acc = 1000
        tcp_speed = 50
        tcp_acc = 100
        # try:
        #     self.clientSocket.send('motion_place_capsule start'.encode('utf-8'))
        # except:
        #     print('socket error')

        #토핑 아래로 지나가는 1
        code = self._arm.set_servo_angle(angle=[81.0, -10.8, 6.9, 103.6, 88.6, 9.6], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        #토핑 아래로 지나가는 2
        code = self._arm.set_servo_angle(angle=[10, -20.8, 7.1, 106.7, 79.9, 26.0], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        #토핑 아래로 지나 올라옴
        code = self._arm.set_servo_angle(angle=[8.4, -42.7, 23.7, 177.4, 31.6, 3.6], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        #탬핑기 바로 앞
        code = self._arm.set_servo_angle(angle=[8.4, -32.1, 55.1, 96.6, 29.5, 81.9], speed=angle_speed,
                                         mvacc=angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        #캡슐 삽입
        code = self._arm.set_position(*self.position_before_capsule_place, speed=tcp_speed,
                                      mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        #캡슐 내려놓음
        code = self._arm.set_position(*self.position_capsule_place, speed=tcp_speed,
                                      mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(2)
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return

        # 뒤로빠진 모션
        code = self._arm.set_position(*[233.4, 10.3, 471.1, -172.2, 87.3, -84.5], speed=tcp_speed,
                                      mvacc=tcp_acc, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        self.motion_num = 3
        print('motion_place_capsule finish')
        
        time.sleep(0.5)

    def motion_place_fail_capsule(self):
        print('motion_place_fail_capsule start')
        angle_speed = 120
        angle_acc = 1000
        tcp_speed = 50
        tcp_acc = 100

        code = self._arm.set_servo_angle(angle=[176, -2, 30.7, 79.5, 98.2, 29.3], speed=angle_speed,
                                        mvacc=angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        if self.return_capsule_position == 1:
            code = self._arm.set_position(*[-255.2, -133.8, 250, 68.3, 86.1, -47.0], speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return

            code = self._arm.set_position(*self.position_jig_A_grab, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif self.return_capsule_position == 2:
            code = self._arm.set_position(*[-152.3, -127.0, 250, 4.8, 89.0, -90.7], speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return

            code = self._arm.set_position(*self.position_jig_B_grab, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        # elif self.return_capsule_position == 3:
        #     code = self._arm.set_position(*[-76.6, -144.6, 250, 5.7, 88.9, -50.1], speed=self._tcp_speed,
        #                                     mvacc=self._tcp_acc, radius=20.0, wait=False)
        #     if not self._check_code(code, 'set_position'):
        #         return
        #     code = self._arm.set_position(*self.position_jig_C_grab, speed=self._tcp_speed,
        #                                     mvacc=self._tcp_acc, radius=0.0, wait=True)
        #     if not self._check_code(code, 'set_position'):
        #         return
        # else:
        #     pass    
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gri1pper'):
            return
        time.sleep(1)
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)
        
        code = self._arm.set_tool_position(z=-70, speed=tcp_speed,
                                            mvacc=tcp_acc, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_servo_angle(angle=self.position_home, speed=50,
                                            mvacc=100, wait=True, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        print('motion_place_fail_capsule finish')


    def motion_grab_cup(self):
        print('motion_grab_cup start')

        angle_speed = 120
        angle_acc = 1000
        tcp_speed = 50
        tcp_acc = 100

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return

        # 컵가는 웨이포인트
        code = self._arm.set_servo_angle(angle=[-2.8, -2.5, 45.3, 119.8, -79.2, -18.8], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        #컵 위치로 이동 잡기
        code = self._arm.set_position(*self.position_cup_grab, speed=tcp_speed,
                                        mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        
        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        time.sleep(1)
        
        # 컵 잡고 올리기
        code = self._arm.set_position(z=100, radius=0, speed=150, mvacc=1000, relative=True,
                                      wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        # 돌아가는 웨이포인트
        code = self._arm.set_servo_angle(angle=[2.9, -28.9, 26.2, 110.2, -26.1, -30.1], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # 돌아가는 웨이포인트
        code = self._arm.set_servo_angle(angle=[3, -8.7, 26.2, 107.4, 60.5, 19.7], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # # 아이스크림 위치
        # code = self._arm.set_position(*[243.1, 134.7, 300, -59.6, 88.5, 29.5], speed=self._tcp_speed,
        #                                   mvacc=self._tcp_acc, radius=20.0, wait=False)
        # if not self._check_code(code, 'set_position'):
        #     return
        
        # # 토핑가는 웨이포인트
        # code = self._arm.set_servo_angle(angle=[34.5, -33.9, 18, 101.1, 73.8, 49.2], speed=self._angle_speed,
        #                                     mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return


        self.motion_num = 4
        print('motion_grab_cup finish')


    def motion_helix(self, radius, height, turns, mode, speed=20):
        # 나선의 중점 설정
        if mode == 0:
            topping_A_center = self.position_topping_A
            topping_A_center[2] = topping_A_center[2]+30

            helix_position = topping_A_center
        elif mode == 1:
            topping_B_center = self.position_topping_B
            topping_B_center[2] = topping_B_center[2]+30

            helix_position = topping_B_center
        elif mode == 2:
            topping_C_center = self.position_topping_C
            topping_C_center[2] = topping_C_center[2]+30

            helix_position = topping_C_center
        elif mode == 3:
            icecream_center = self.position_icecream
            icecream_center[2] = icecream_center[2]+60

            helix_position = icecream_center
            

        # t 값 생성
        t_values = np.linspace(0, turns * 2 * np.pi, 1000)  # 100개의 점을 생성

        # x, y, z 좌표 계산
        x_values = radius * np.cos(t_values) + helix_position[0]
        y_values = radius * np.sin(t_values) + helix_position[1]
        z_values = height * t_values + helix_position[2]

        # 시작점 설정
        start_point = (x_values[0], y_values[0], z_values[0])
        end_point = (x_values[-1], y_values[-1], z_values[-1])

        a = [start_point[0], start_point[1], start_point[2], 
                                        helix_position[3], helix_position[4], helix_position[5]]
    
        # 시작 위치로 이동
        code = self._arm.set_position(*[start_point[0], start_point[1], start_point[2], 
                                        helix_position[3], helix_position[4], helix_position[5]], 
                                        speed=speed, mvacc=100, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # 나선을 따라 이동
        for x, y, z in zip(x_values, y_values, z_values):
            code = self._arm.set_position(*[x, y, z, helix_position[3], helix_position[4], helix_position[5]], 
                                        speed=speed, mvacc=100, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return

        # 끝점으로 이동
        code = self._arm.set_position(*[helix_position[0], helix_position[1], end_point[2],
                                        helix_position[3], helix_position[4], helix_position[5]], 
                                        speed=speed, mvacc=100, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

    def motion_spiral(self, radius, total_height, total_turns=5, speed=20):
        # 현재 로봇 위치 가져오기
        tuple_c = self._arm.get_position()
        list_c = tuple_c[1]
        current_position = list_c[:6]

        # 중심 좌표 설정
        cx = current_position[0]
        cy = current_position[1]
        cz = current_position[2]
        num_points = 1000

        # total_height가 양수이면 위로, 음수이면 아래로 원뿔을 생성
        z = np.linspace(0, total_height, num_points)  # 높이가 0에서 total_height까지 증가 또는 감소
        r = radius * (1 - np.abs(z / total_height))  # z의 절대값을 사용하여 반지름이 줄어듦

        theta = np.linspace(0, total_turns * 2 * np.pi, num_points)  # 0에서 total_turns * 2π까지 회전

        # 극 좌표를 직교 좌표로 변환하고 중심 좌표 더하기
        x = r * np.cos(theta) + cx
        y = r * np.sin(theta) + cy
        z = cz + z  # cz를 기준으로 z를 증가 또는 감소시킴

        # 로봇의 위치를 시간에 따라 업데이트하는 예제
        for i in range(num_points):
            current_x = x[i]
            current_y = y[i]
            current_z = z[i]

            # 로봇의 자세는 원래의 자세를 유지 (roll, pitch, yaw)
            roll = current_position[3]
            pitch = current_position[4]
            yaw = current_position[5]

            # 위치 설정 명령
            code = self._arm.set_position(current_x, current_y, current_z, roll, pitch, yaw,
                                          speed=speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return

    def motion_make_icecream(self):
        print('motion_make_icecream start')
        # 아이스크림 위치
        code = self._arm.set_position(*[243.1, 134.7, 300, -59.6, 88.5, 29.5], speed=150,
                                        mvacc=1000, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        self.sound.Effect_play(f'making.mp3')
        # 아이스크림 추출 준비
        code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        
        code = self._arm.set_position(z=60, radius=0, speed=30, mvacc=100, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        time.sleep(7)

        # self.motion_sprial2()
        # self.motion_spiral3(8, -5, 3, 30)
        self.motion_spiral(radius=15, total_height=-40, total_turns=5, speed=50)

        # self.motion_helix(8, -1, 3, 3, 30)

        time.sleep(5)
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        
        self.motion_num = 5
        print('motion_make_icecream finish')


    def motion_get_topping(self, topping):
        tcp_speed = 50
        tcp_acc = 100

        if topping == 0:
            #-----------토핑 A----------
            code = self._arm.set_position(*self.position_topping_B, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return 

            code = self._arm.set_position(*self.position_topping_A, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return 
            
            code = self._arm.set_position(z=30, radius=0, speed=tcp_speed, mvacc=tcp_acc, relative=True,
                                            wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                    return
            time.sleep(0.5)
            self.motion_helix(10, -1, 1, 0, 40)

            code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                    return
            time.sleep(0.5)
            
        elif topping == 1:
            #-----------토핑 B----------
            code = self._arm.set_position(*self.position_topping_B, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return 
            
            code = self._arm.set_position(z=30, radius=0, speed=tcp_speed, mvacc=tcp_acc, relative=True,
                                            wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                    return
            time.sleep(0.5)
            self.motion_helix(10, -1, 1, 1, 40)

            code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                    return
            time.sleep(0.5)
            
        elif topping == 2:
            #-----------토핑 C----------
            code = self._arm.set_position(*self.position_topping_B, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return 

            code = self._arm.set_position(*self.position_topping_C, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return 
            
            code = self._arm.set_position(z=30, radius=0, speed=tcp_speed, mvacc=tcp_acc, relative=True,
                                            wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_cgpio_digital(2, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                    return
            time.sleep(0.5)
            self.motion_helix(10, -1, 1, 2, 40)

            code = self._arm.set_cgpio_digital(2, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                    return
            time.sleep(0.5)
            

    def motion_topping(self, topping_list):
        print('motion_topping start')
        angle_speed = 100
        angle_acc = 1000
        #토핑 아래로 지나가는
        code = self._arm.set_servo_angle(angle=[16.816311, -37.649286, 4.669835, 96.031508, 82.383711, 41.868891], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # 토핑 아래
        code = self._arm.set_servo_angle(angle=[126.573252, -53.387978, 1.068108, 182.537822, 35.668972, -4.317638], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=00.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        for select in topping_list:
            print(select)
            if  select == 'oreo':
                self.motion_get_topping(0)
            elif  select == 'chocoball':
                self.motion_get_topping(1)
            elif  select == 'cereal':
                self.motion_get_topping(2)
            else:
                pass

        # 배출 웨이포인트
        code = self._arm.set_servo_angle(angle=self.position_home, speed=30,
                                         mvacc=100, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_servo_angle(angle=self.position_finish, speed=30,
                                            mvacc=100, wait=True, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        self.motion_num = 6
        print('motion_topping finish')

    
    def motion_Dondurma(self):
        print('motion_Dondurma start')

        self.dondurma_someone = True

        code = self._arm.set_position(*self.Dondurma_start_position, speed=100,
                                            mvacc=5000, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        self.sound.Effect_play(f'Dondurma_start.mp3')
        
        Dondurma_count = random.randrange(2,5)
        print(Dondurma_count)
        Dondurma_y_axis = 500

        for i in range(0,Dondurma_count):
            if self.dondurma_someone == True:
                Dondurma_y_axis = Dondurma_y_axis-abs((self.Dondurma_start_position[1]-self.Dondurma_end_position[1])/Dondurma_count)
                Dondurma_x_axis = random.randrange(-260,260)
                Dondurma_z_axis = random.randrange(240,340)

                # print("{} / {}  y = {} x = {} z = {}" .format(i,Dondurma_count,Dondurma_y_axis,Dondurma_x_axis,Dondurma_z_axis))
                
                self.Dondurma_hand()
                if self.dondurma_detected:
                    code = self._arm.set_position(*[Dondurma_x_axis, -Dondurma_y_axis, Dondurma_z_axis, self.Dondurma_end_position[3],
                                                    self.Dondurma_end_position[4], self.Dondurma_end_position[5]], speed=450, mvacc=5000, radius=0.0, wait=False)
                    if not self._check_code(code, 'set_position'):
                        return
                    
                    self.sound.Effect_play(f'Dondurma_effect_{i}.mp3')
                print(f'i                      {i}')
                time.sleep(1)
            else:
                break

        code = self._arm.set_position(*self.Dondurma_end_position,
                                        speed=100, mvacc=1000, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        self.sound.Effect_play(f'THX.mp3')
        time.sleep(10)
        
        # print(Dondurma_count)
        # print(Dondurma_start_position[1] - Dondurma_end_position[1])
        # print(abs((Dondurma_start_position[1]-Dondurma_end_position[1])/Dondurma_count))

        # code = self._arm.set_position(y=abs((Dondurma_start_position[1]-Dondurma_end_position[1])/Dondurma_count), radius=0, speed=10, mvacc=50, relative=True, wait=False)
        # if not self._check_code(code, 'set_position'):
        #     return

        self.motion_num = 7
        print('motion_Dondurma finish')

    def motion_stack_icecream(self):
        angle_speed = 100
        angle_acc = 1000
        tcp_speed = 50
        tcp_acc = 100

        self.sound.Effect_play(f'stack_icecream.mp3')
        code = self._arm.set_position(*self.Dondurma_end_position,
                                        speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.set_position(*[60.558903, -156.656799, 285, 54.735632, 89.999981, -82.133271],
                                        speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(z=178, speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(2)

        code = self._arm.set_position(*[117.877991, -102.960449, 178, 54.735632, 89.999981, -82.133271],
                                        speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.set_position(z=300, speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.set_servo_angle(angle=[270, -7.800018, 32.599981, 267.599996, 90, -40], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=self.position_home, speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        
    def motion_dump_icecream(self):
        angle_speed = 100
        angle_acc = 1000
        tcp_speed = 50
        tcp_acc = 100

        self.sound.Effect_play(f'dump_icecream.mp3')
        code = self._arm.set_servo_angle(angle=[270, -7.800018, 32.599981, 267.599996, 90, -40], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_position(*[117.877991, -102.960449, 300, 54.735632, 89.999981, -82.133271],
                                        speed=tcp_speed, mvacc=tcp_acc, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(z=178, speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(0.5)
        
        code = self._arm.set_position(*[60.558903, -156.656799, 178, 54.735632, 89.999981, -82.133271],
                                        speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(2)

        code = self._arm.set_position(z=250, speed=tcp_speed, mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_servo_angle(angle=[180, 15.58, 35, 269.87, 80, -19.34], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[135, -6, 11, 220, 70, 130], speed=50,
                                            mvacc=100, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # # 터는 동작
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)

        code = self._arm.set_servo_angle(angle=[135, -6, 11, 230, 60, 130], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[135, -6, 11, 220, 70, 130], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[135, -6, 11, 230, 60, 130], speed=angle_speed,
                                            mvacc=angle_acc, wait=True, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(2)

        code = self._arm.set_servo_angle(angle=self.position_home, speed=50,
                                            mvacc=100, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
    
        

    def motion_trash_capsule(self):
        self.motion_num = 8
        print('motion_trash_capsule start')
        angle_speed = 100
        angle_acc = 1000
        tcp_speed = 50
        tcp_acc = 100
        # try:
        #     self.clientSocket.send('motion_trash_start'.encode('utf-8'))
        # except:
        #     print('socket error')
        
        # 홈에서 템핑가는 토핑밑
        code = self._arm.set_servo_angle(angle=[51.2, -8.7, 13.8, 95.0, 86.0, 17.0], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # 홈에서 템핑가는 토핑밑에서 올라옴
        code = self._arm.set_servo_angle(angle=[-16.2, -19.3, 42.7, 82.0, 89.1, 55.0], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        self.sound.Effect_play(f'remove_capsule.mp3')
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        code = self._arm.set_servo_angle(angle=[-19.9, -19.1, 48.7, 87.2, 98.7, 60.0], speed=angle_speed,
                                         mvacc=angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        # 템핑 앞
        code = self._arm.set_position(*[222.8, 0.9, 470.0, -153.7, 87.3, -68.7], speed=tcp_speed,
                                      mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 캡슐 집고 올림
        code = self._arm.set_position(*self.position_capsule_grab, speed=tcp_speed,
                                      mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.set_position(z=30, radius=-1, speed=tcp_speed, mvacc=tcp_acc, relative=True,
                                      wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 캡슐 집고 올림
        code = self._arm.set_position(*[221.9, -5.5, 500.4, -153.7, 87.3, -68.7], speed=tcp_speed,
                                      mvacc=tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 탬핑에서부터 버리는 동작
        code = self._arm.set_servo_angle(angle=[-10.7, -2.4, 53.5, 50.4, 78.1, 63.0], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # 뒤집어서 쓰래기통 위
        code = self._arm.set_servo_angle(angle=[18.0, 11.2, 40.4, 90.4, 58.7, -148.8], speed=angle_speed,
                                         mvacc=angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        # 터는 동작
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        # time.sleep(2)
        code = self._arm.set_servo_angle(angle=[25.2, 15.2, 42.7, 83.2, 35.0, -139.8], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[18.0, 11.2, 40.4, 90.4, 58.7, -148.8], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[25.2, 15.2, 42.7, 83.2, 35.0, -139.8], speed=angle_speed,
                                         mvacc=angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        
        # 털고 복귀하는 토핑 밑
        code = self._arm.set_servo_angle(angle=[28.3, -9.0, 12.6, 85.9, 78.5, 20.0], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        
        code = self._arm.set_servo_angle(angle=[149.3, -9.4, 10.9, 114.7, 69.1, 26.1], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        # 홈 위치
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=angle_speed,
                                         mvacc=angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # try:
        #     self.clientSocket.send('motion_trash_finish'.encode('utf-8'))
        # except:
        #     print('socket error')
        # time.sleep(0.5)

        self.motion_num = 8
        print('motion_trash_capsule finish')


    def motion_recovery(self, num):
        angle_speed = 60
        angle_acc = 500
        tcp_speed = 50
        tcp_acc = 100

        if num in [1, 2, 6]:
            print("복구 시작 : 캡슐 잡은 상태")
            self.motion_place_fail_capsule()

        elif num in [3, 4]:
            print("복구 시작 : 캡슐 삽입한 상태")

            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)

            # # 템핑 앞
            # code = self._arm.set_position(*[222.8, 0.9, 470.0, -153.7, 87.3, -68.7], speed=self._tcp_speed,
            #                             mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #     return
            code = self._arm.set_servo_angle(angle=[-15.503321, -4.641989, 71.499862, 85.082915, 99.572387, 74.310964], speed=30,
                                            mvacc=100, wait=False, radius=10.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            # 캡슐 집고 올림
            code = self._arm.set_position(*self.position_capsule_grab, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            time.sleep(1)
            code = self._arm.set_position(z=25, radius=-1, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            self._tcp_speed = 100
            self._tcp_acc = 1000

            # 캡슐 빼기
            code = self._arm.set_position(*[221.9, -15.5, 500.4, -153.7, 87.3, -68.7], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            

            code = self._arm.set_servo_angle(angle=[-10.7, -2.4, 53.5, 70, 78.1, 63.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=10.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            #토핑 아래로 지나가는 2
            code = self._arm.set_servo_angle(angle=[10, -20.8, 7.1, 106.7, 79.9, 26.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=50.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            #토핑 아래로 지나가는 1
            code = self._arm.set_servo_angle(angle=[81.0, -10.8, 6.9, 103.6, 88.6, 9.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=40.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            self.motion_place_fail_capsule()

        elif num == 5:
            print("복구 시작 : 아이스크림 제조한 상태")
            #토핑 아래로 지나가는
            code = self._arm.set_servo_angle(angle=[16.816311, -37.649286, 4.669835, 96.031508, 82.383711, 41.868891], speed=angle_speed,
                                                mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            # 토핑 아래
            code = self._arm.set_servo_angle(angle=[126.573252, -53.387978, 1.068108, 182.537822, 35.668972, -4.317638], speed=angle_speed,
                                                mvacc=angle_acc, wait=False, radius=00.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            # 배출 웨이포인트
            code = self._arm.set_servo_angle(angle=self.position_home, speed=30,
                                                mvacc=100, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            self.motion_place_fail_capsule()


    def motion_grap_squeegee(self):
        #스키지 잡으러 가기
        code = self._arm.set_position(*[-249, 128.5, 171, -90, 90, 90], speed=100,
                                                mvacc=1000, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        # 스키지 위치
        code = self._arm.set_position(*[-323.5, 128.5, 171, 90, 90, -90], speed=30,
                                                mvacc=100, radius=00.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(2)

        # 우측 배드 초기 위치
        code = self._arm.set_position(*[-200 ,  0  , 400 , 180, 0, 90], speed=100,
                                        mvacc=1000, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        

    def motion_sweeping_right(self):        
        up_speed = 200
        up_acc = 2000
        down_speed = 100
        down_acc = 1000
        code = self._arm.set_position(*[-130 ,  -83  , 400 , 180.00002, -0.544826, 88.453969], speed=up_speed,
                        mvacc=up_acc, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(z=265, speed=down_speed,
                                        mvacc=down_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return


        # 2.sweep
        code = self._arm.set_position(y=5, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(x=-180, y=30, speed=down_speed,
                                        mvacc=down_acc, radius=100.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(y=60, speed=down_speed,
                                        mvacc=down_acc, radius=100.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # 3.up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        #6. move to point 3
        code = self._arm.set_position(*[-250, -83, 368.65, 180.00002, -0.544826, 88.453969], speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.set_position(z=265, speed=down_speed,
                        mvacc=down_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # sweep 
        code = self._arm.set_position(y=60, speed=down_speed,
                                mvacc=down_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        # up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # rotate and 7.move to 5 again  
        code = self._arm.set_position(*[-180,  -153,  368.65, -180.00002, 0.544826, -88.453969 ], speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # down
        code = self._arm.set_position(z=265, speed=down_speed,
                                mvacc=down_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # sweep
        code = self._arm.set_position(y=-30, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(x=-132, y=-10, speed=down_speed,
                                        mvacc=down_acc, radius=100.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(y=70, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 9.up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # 10.move to 7 & down
        code = self._arm.set_position(*[-250,  -153.66,  368.65, -180.00002, 0.544826, -88.453969], speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(z=265, speed=down_speed,
                                mvacc=down_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        # 11.sweep
        code = self._arm.set_position(y=70, speed=down_speed,
                                    mvacc=down_acc, radius=-20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        #up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                    mvacc=up_acc, radius=-20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        #side sweep        
        code = self._arm.set_position(*[-200.0, 64.623001, 368.649994, 180, 0, -0], speed=up_speed,
                                    mvacc=up_acc, radius=-20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[-290, 169.60968, 265, 180, 0.0, -0.0], speed=down_speed,
                                    mvacc=down_acc, radius=-20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(x=150, speed=down_speed,
                                    mvacc=down_acc, radius=-20.0, relative=True,wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # # 우측 배드 초기 위치
        # code = self._arm.set_position(*[-200 ,  0  , 400 , 180, 0, 90], speed=50,
        #                                 mvacc=self._tcp_acc, radius=20.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return


    def motion_sweeping_left(self): 
        up_speed = 200
        up_acc = 2000
        down_speed = 100
        down_acc = 1000

        # 좌측 배드 초기 위치
        code = self._arm.set_position(*[200,0,400, 180, 0.0, 90], speed=up_speed,
                                mvacc=up_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        # move to point (1)
        code = self._arm.set_position(*[135,-53.5, 368.65, 180, 0.0, 90], speed=up_speed,
                                mvacc=up_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        # down
        code = self._arm.set_position(z=265, speed=down_speed,
                                        mvacc=down_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        # 2.sweep
        code = self._arm.set_position(y=5, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(x=185, y=30, speed=down_speed,
                                        mvacc=down_acc, radius=100.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(y=60, speed=down_speed,
                                        mvacc=down_acc, radius=100.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                mvacc=up_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        # move to point (3)
        code = self._arm.set_position(*[262,-53.5, 368.65, 180, 0, 90], speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return              
        # down
        code = self._arm.set_position(z=265, speed=down_speed,
                                        mvacc=down_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        # 1st sweep
        code = self._arm.set_position(y=60, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return        
        #up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                mvacc=up_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return            
        #rotate & move to point (5)
        code = self._arm.set_position(*[185, -140, 368.65, -180, 0.0, -90], speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        #down
        code = self._arm.set_position(z=265, speed=down_speed,
                                        mvacc=down_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 8.sweep 53.66에서 수정함 -> 56.66 -> 59.66
        code = self._arm.set_position(y=-30, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(x=132, y=-10, speed=down_speed,
                                        mvacc=down_acc, radius=100.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(y=70, speed=down_speed,
                                        mvacc=down_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        #up
        code = self._arm.set_position(z=368.65, speed=up_speed,
                                mvacc=up_acc, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        #move to point(7)
        code = self._arm.set_position(*[262, -140, 368.65, -180,0,-90], speed=up_speed,
                                        mvacc=up_acc, radius=0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        #down
        code = self._arm.set_position(z=265, speed=down_speed,
                                        mvacc=down_acc, radius=0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 2st sweep
        code = self._arm.set_position(y=70, speed=down_speed,
                                        mvacc=down_acc, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # # 좌측 배드 초기 위치
        # code = self._arm.set_position(*[200,0,400, 180, 0.0, 90], speed=self._tcp_speed,
        #                         mvacc=self._tcp_acc, radius=0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return

        # code = self._arm.set_position(*[290, 169.60968, 265.1, -180, 0.0, -180.0], speed=self._tcp_speed,
        #                             mvacc=self._tcp_acc, radius=-20.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return
        # code = self._arm.set_position(x=-150, speed=self._tcp_speed,
        #                             mvacc=self._tcp_acc, radius=-20.0, relative=True,wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return


    def motion_place_squeegee(self):
        # 우측 배드 초기 위치
        code = self._arm.set_position(*[-200 ,  0  , 400 , 180, 0, 90], speed=100,
                                        mvacc=1000, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        # 스키지 위치
        code = self._arm.set_position(*[-323.5, 128.5, 200, 90, 90, -90], speed=100,
                                                mvacc=1000, radius=00.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(z=172, speed=30,
                                                mvacc=100, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(1)

        code = self._arm.set_position(*[-249, 128.5, 172.5, -90, 90, 90], speed=30,
                                                mvacc=100, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
    def motion_pick_felldown_trash(self, target_x, target_y, deg):
        code = self._arm.set_position(x=target_x, y=target_y, yaw=deg, speed=100,
                                        mvacc=1000, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(z=250, speed=30,
                                        mvacc=100, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(2)

        code = self._arm.set_position(z=300, speed=100,
                                        mvacc=1000, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
    def motion_dump_trash(self):
        angle_speed = 50
        angle_acc = 500
        tcp_speed = 50
        tcp_acc = 100

        time.sleep(3)
        tuple_c = arm.get_position()
        list_c = tuple_c[1]
        current_position = list_c[:6]
        print(f'current_position[0] {current_position[0]}')

        if current_position[0] > 0:
            # left way point
            code = self._arm.set_servo_angle(angle=[41.185925, -42.989596, 31.248774, -0.0, 74.238371, 176.185954], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            # # left way point
            # code = self._arm.set_position(*[80, 70, 350.0, 180, -0.0, -135], speed=self._tcp_speed,
            #                                 mvacc=self._tcp_acc, radius=20.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #     return
            # middle
            code = self._arm.set_position(*[3.7e-05, 120, 280.0, -180, 0, -90], speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(y=180, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            # 터는 동작
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)

            code = self._arm.set_servo_angle(angle=[90, -12.6, 32, 0, 25, 180], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[90, -12.6, 25, 0, 30, 180], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[90, -12.6, 32, 0, 25, 180], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[90, -12.6, 25, 0, 30, 180], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(2)

            # right way point
            code = self._arm.set_position(*[-80, 70, 350, 180, -0.0, -45], speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
        else:
            # right way point
            code = self._arm.set_servo_angle(angle=[138.814095, -42.989596, 31.248774, -0.0, 74.238371, 183.814085], speed=angle_speed,
                                            mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            # # right way point
            # code = self._arm.set_position(*[-80, 70, 350, 180, -0.0, -45], speed=self._tcp_speed,
            #                                 mvacc=self._tcp_acc, radius=20.0, wait=False)
            # if not self._check_code(code, 'set_position'):
            #     return
            # middle
            code = self._arm.set_position(*[3.7e-05, 120, 280.0, -180, 0, -90], speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(y=180, speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)

            code = self._arm.set_servo_angle(angle=[90, -12.6, 32, 0, 25, 180], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[90, -12.6, 25, 0, 30, 180], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[90, -12.6, 32, 0, 25, 180], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[90, -12.6, 25, 0, 30, 180], speed=angle_speed,
                                         mvacc=angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(2)

            # right way point
            code = self._arm.set_position(*[-80, 70, 350, 180, -0.0, -45], speed=tcp_speed,
                                            mvacc=tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
    def motion_cleaning(self, mode, target_x=None, target_y=None, target_deg=None):
        tcp_speed = 50
        tcp_acc = 100

        time.sleep(3)
        tuple_c = arm.get_position()
        list_c = tuple_c[1]
        current_position = list_c[:6]
        print(f'current_position[0] {current_position[0]}')

        self.sound.Effect_play(f'cleaning.mp3')

        if mode == 0: #Squeeze
            if current_position[0] > 0:
                self.left_to_right()
            else :
                pass
                
            code = self._arm.set_servo_angle(angle=self.position_home, speed=50,
                                        mvacc=100, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            self.motion_grap_squeegee()
            
            self.motion_sweeping_right()

            self.right_to_left()
            
            self.motion_sweeping_left()

            self.left_to_right()

            self.motion_place_squeegee()

            code = self._arm.set_servo_angle(angle=self.position_home, speed=50,
                                                    mvacc=100, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

        elif mode == 1: #felldown
            if current_position[0] > 0 and target_x > 0:
                # 좌측 배드 초기 위치
                code = self._arm.set_position(*[200,0,400, 180, 0.0, 90], speed=tcp_speed,
                                        mvacc=tcp_acc, radius=0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
            elif current_position[0] < 0 and target_x < 0:
                
                # 우측 배드 초기 위치
                code = self._arm.set_position(*[-200,  0, 400, 180, 0, 90], speed=tcp_speed,
                                                mvacc=tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return

            elif current_position[0] > 0 and target_x < 0:
                self.left_to_right()

                # 우측 배드 초기 위치
                code = self._arm.set_position(*[-200 ,  0  , 400 , 180, 0, 90], speed=tcp_speed,
                                                mvacc=tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
            elif current_position[0] < 0 and target_x > 0:
                self.right_to_left()

                # 좌측 배드 초기 위치
                code = self._arm.set_position(*[200,0,400, 180, 0.0, 90], speed=tcp_speed,
                                        mvacc=tcp_acc, radius=0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return

            self.motion_pick_felldown_trash(target_x, target_y, target_deg)

            self.motion_dump_trash()
            
            code = self._arm.set_servo_angle(angle=self.position_home, speed=50,
                                                    mvacc=100, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return



    def right_to_left(self):
        # # right way point
        # code = self._arm.set_position(*[-156.067001, 70, 290.0, 180, -0.0, -45], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=20.0, wait=False)
        # if not self._check_code(code, 'set_position'):
        #     return
        # # middle
        # code = self._arm.set_position(*[3.7e-05, 120, 270., -180, 0, -90], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=20.0, wait=False)
        # if not self._check_code(code, 'set_position'):
        #     return
        # # left way point
        # code = self._arm.set_position(*[156.067001, 70, 290.0, 180, -0.0, -135], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=20.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return
        
        # right way point
        code = self._arm.set_servo_angle(angle=[138.814095, -42.989596, 31.248774, -0.0, 74.238371, 183.814085], speed=50,
                                        mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # # right way point
        # code = self._arm.set_position(*[-80, 70, 350, 180, -0.0, -45], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=20.0, wait=False)
        # if not self._check_code(code, 'set_position'):
        #     return
        # middle
        code = self._arm.set_position(*[3.7e-05, 120, 280.0, -180, 0, -90], speed=150,
                                        mvacc=1000, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        # left way point
        code = self._arm.set_position(*[80, 70, 350.0, 180, -0.0, -135], speed=150,
                                        mvacc=1000, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
    def left_to_right(self):
        # # left way point
        # code = self._arm.set_position(*[156.067001, 70, 290.0, 180, -0.0, -135], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=20.0, wait=False)
        # if not self._check_code(code, 'set_position'):
        #     return
        # # middle
        # code = self._arm.set_position(*[3.7e-05, 120, 270., -180, 0, -90], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=20.0, wait=False)
        # if not self._check_code(code, 'set_position'):
        #     return
        # # right way point
        # code = self._arm.set_position(*[-156.067001, 70, 290.0, 180, -0.0, -45], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=20.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return

        # left way point
        code = self._arm.set_servo_angle(angle=[41.185925, -42.989596, 31.248774, -0.0, 74.238371, 176.185954], speed=50,
                                        mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # # left way point
        # code = self._arm.set_position(*[80, 70, 350.0, 180, -0.0, -135], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=20.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return
        
        # middle
        code = self._arm.set_position(*[3.7e-05, 120, 280.0, -180, 0, -90], speed=150,
                                        mvacc=1000, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        # right way point
        code = self._arm.set_position(*[-80, 70, 350, 180, -0.0, -45], speed=150,
                                        mvacc=1000, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
    def motion_tracking(self):

        flag = 1

        while True:
            tuple_c = arm.get_position()
            list_c = tuple_c[1]
            current_position = list_c[:6]

            if len(self.tracking_position) == 2:
                print(f'\n\n               로봇         \n\n\n{self.tracking_position[0]} , {self.tracking_position[1]}\n\n\n\n\n')


                a = (self.tracking_position[0] - current_position[0])

                b = (self.tracking_position[1] - current_position[1])
                
                if flag == 1:
                    print('들어옴')
                    code = self._arm.set_position(x=a,y=b, speed=50, relative=True,
                                            mvacc=100, radius=0.0, wait=True)
                    flag = 0
                print(f'{a} , {b},  {(a > 1 or a < -1)}, {(b > 1 or b < -1)}')
                time.sleep(0.5)

                if (a > 1 or a < -1) or (b > 1 or b < -1):
                    print(f'들어옴 {current_position[0]}        {a}')
                    flag = 1



    def test_main(self):

        #----------initial----------
        # self.sound = Hi5_Sound()
        # self.sound.sound_init()

        # self.sound.BGM_play()

        # self.motion_home()


        #----------제조 과정----------완

        # # JSON 메시지
        # json_message = '''
        # {
        #     "OR": [
        #         {"orderId": "1", "icecream": "choco", "topping":"cereal,oreo,chocoball"}
        #     ]
        # }
        # '''

        # # # JSON 메시지 파싱
        # data = json.loads(json_message)

        # for order in data['OR']:
        #     self.icecream = order['icecream']
        #     self.topping = order['topping']
        #     print(f"icecream: {self.icecream}, topping: {self.topping}")

        #     # 토핑의 갯수 계산
        #     self.topping_list = [t.strip() for t in self.topping.split(',')]
        #     # topping_count = len(topping_list)
            
        #     print(self.topping_list)
        #     # self.a = True

        # # # 필요한 정보 추출
        # # icecream = data["OR"]["icecream"]
        # # topping = data["OR"]["topping"]
        # # print(f"icecream: {icecream}, topping: {topping}")

        # # topping_list = [t.strip() for t in topping.split(',')]
        # # print(topping_list)

        # self.sound.Effect_play(f'order.mp3')
        # self.sound.Effect_play(f'{self.icecream}.mp3')
        # for i in self.topping_list:
        #     self.sound.Effect_play(f'{i}')
        # self.sound.Effect_play(f'makestart.mp3')


        # self.capsule_position = 1

        # self.motion_grab_capsule()1

        # self.motion_check_sealing()2

        # self.motion_place_capsule()3

        # self.motion_grab_cup()4

        # self.motion_make_icecream()5

        # self.motion_topping(self.topping_list)6

        # self.motion_Dondurma()7

        # self.motion_trash_capsule()8

        # self.motion_home()


        # 실링 체크 실패했을 때
        # self.motion_place_fail_capsule()
        # self.sound.Effect_play(f'sealing.mp3')


        # 돈두르마시 감지 안됐을 때
        # self.motion_stack_icecream()

        # 아이스크림 지그에 5분 이상 있었을 때
        # self.motion_dump_icecream()


        #----------청소 과정---------- 
        # self.motion_home()

        # self.motion_cleaning(0)
        # self.motion_cleaning(1, self.tracking_position[0], self.tracking_position[1], self.tracking_angle)

        # # self.trash_position()
        # # for i in range(0,self.tracking_cnt):
        # #     self.trash_angle()
            
        # #     if self.tracking_angle != None:
        # #         print(f"\n\n\n x = {self.tracking_position[0]} y =  {self.tracking_position[1]} deg = {self.tracking_angle}\n\n\n")
        # #         # self.motion_cleaning(1, self.tracking_position[0], self.tracking_position[1], self.tracking_angle)
        # #         self.tracking_position.clear()
        # #         self.tracking_angle = None

        # self.motion_home()

   









        # print('get_position :{0} \nget_position(radian) :{1} \nget_servo_angle :{2}'.format(arm.get_position(), arm.get_position(is_radian=True), arm.get_servo_angle()))
        
        #-------init part-------

        # good = 0

        # self.vision()
        # time.sleep(10)
        # self.motion_home()

        # while True:
        #     if self.icecream:
        #         self.motion_home()

        #         self.motion_grab_capsule()

        #         self.motion_check_sealing()

        #         # self.check_sealing()
        #         time.sleep(5)

        #         if self.detect_sealing:
        #             self.motion_place_capsule()
        #             good = 1

        #         else:
        #             self.motion_place_fail_capsule()
        #             good = 0

        #         if good:
        #             self.motion_grab_cup()

        #             self.motion_make_icecream()
                    
        #             self.motion_icecream_to_topping()

        #             self.topping_combination(self.topping_list)

        #             self.motion_vent()

        #             self.motion_Dondurma()

        #             self.motion_trash_capsule()

        #             self.motion_home()
        #         break


# print('get_position :{0} \nget_position(radian) :{1} \nget_servo_angle :{2}'.format(arm.get_position(), arm.get_position(is_radian=True), arm.get_servo_angle()))

        

#  def run(self):
        # try:
        #     while self.is_alive:
        #         # Joint Motion
        #         if self.state == 'icecreaming':
        #             # --------------icecream start--------------------
        #             try:
        #                 self.clientSocket.send('icecream_start'.encode('utf-8'))
        #             except:
        #                 print('socket error')
        #             time.sleep(int(self.order_msg['makeReq']['latency']))

        #             self.motion_home()
        #             while True:
        #                 if self.order_msg['makeReq']['latency'] in ['go', 'stop']:
        #                     break
        #                 time.sleep(0.2)
        #             if self.order_msg['makeReq']['latency'] in ['go']:
        #                 self.motion_grab_capsule()
        #                 if self.order_msg['makeReq']['sealing'] in ['yes']:
        #                     self.motion_check_sealing()
        #                     try:
        #                         self.clientSocket.send('sealing_check'.encode('utf-8'))
        #                     except:
        #                         pass
        #                     count = 0
        #                     while True:
        #                         # if sealing_check request arrives or 5sec past
        #                         if self.order_msg['makeReq']['sealing'] in ['go', 'stop'] or count >= 5:
        #                             print(self.order_msg['makeReq']['sealing'])
        #                             break
        #                         time.sleep(0.2)
        #                         count += 0.2
        #                 if self.order_msg['makeReq']['sealing'] in ['go'] or self.order_msg['makeReq']['sealing'] not in ['yes', 'stop']:
        #                     #print('sealing_pass')
        #                     self.motion_place_capsule()
        #                     self.motion_grab_cup()
        #                     self.motion_topping()
        #                     self.motion_make_icecream()
        #                     self.motion_serve()
        #                     self.motion_trash_capsule()
        #                     self.motion_home()
        #                     print('icecream finish')
        #                     while True:
        #                         try:
        #                             self.clientSocket.send('icecream_finish'.encode('utf-8'))
        #                             break
        #                         except:
        #                             time.sleep(0.2)
        #                             print('socket_error')
        #                 else:
        #                     self.motion_place_fail_capsule()
        #                     self.motion_home()
        #                     self.clientSocket.send('icecream_cancel'.encode('utf-8'))
        #                     self.order_msg['makeReq']['sealing'] = ''
        #             else:
        #                 while True:
        #                     try:
        #                         self.clientSocket.send('icecream_cancel'.encode('utf-8'))
        #                         break
        #                     except:
        #                         print('socket error')
        #                 self.order_msg['makeReq']['latency'] = 0
        #             self.state = 'ready'

        #         elif self.state == 'test':
        #             try:
        #                 self.clientSocket.send('test_start'.encode('utf-8'))
        #             except:
        #                 print('socket error')
        #             # self.motion_home()
        #             # self.motion_grab_cup()
        #             # self.motion_serve()

        #         else:
        #             pass

        #         # self.state = 'ready'
        # except Exception as e:
        #     self.pprint('MainException: {}'.format(e))
        # self.alive = False
        # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        # self._arm.release_state_changed_callback(self._state_changed_callback)
        # if hasattr(self._arm, 'release_count_changed_callback'):
        #     self._arm.release_count_changed_callback(self._count_changed_callback)





        # ------------------하이파이브 모션
        # code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
        #                             mvacc=self._angle_acc, wait=True, radius=100.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        # code = self._arm.set_servo_angle(angle=[272.9, -1.6, 147.1, 176.3, 27.9, 4.9], speed=self._angle_speed,
        #                             mvacc=self._angle_acc, wait=True, radius=100.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        # code = self._arm.set_servo_angle(angle=[273, 23.9, 147, 178.5, 57.3, -3.5 ], speed=self._angle_speed,
        #                             mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        # code = self._arm.set_servo_angle(angle=[273, 25, 147, 178.8, 40, -5.7], speed=self._angle_speed1,
        #                             mvacc=self._angle_acc, wait=False, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        # code = self._arm.set_servo_angle(angle=[273, 23.9, 147, 178.5, 57.3, -3.5 ], speed=self._angle_speed1,
        #                             mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        # code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
        #                             mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
            # return
    
        # code = self._arm.open_lite6_gripper()
        # if not self._check_code(code, 'open_lite6_gripper'):
        #     return
            
        # # code = self._arm.set_servo_angle(angle=[180.5, 3.5, 14.1, 180.8, 81.7, -5.1], speed=self._angle_speed,
        # #                             mvacc=self._angle_acc, wait=True, radius=0.0)
        # # if not self._check_code(code, 'set_servo_angle'):
        # #     return
        # # code = self._arm.set_servo_angle(angle=[320.9, 3.5, 14, 186.7, 80.6, -2.3], speed=self._angle_speed,
        # #                             mvacc=self._angle_acc, wait=True, radius=0.0)
        # # if not self._check_code(code, 'set_servo_angle'):
        # #     return        
        # code = self._arm.set_servo_angle(angle=[320.8, 21.6, 11, 192.9, 81.8, 2.4], speed=self._angle_speed,
        #                             mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return

        # code = self._arm.set_cgpio_analog(0, 5)
        # if not self._check_code(code, 'set_cgpio_analog'):
        #     return
        # code = self._arm.set_cgpio_analog(1, 5)
        # if not self._check_code(code, 'set_cgpio_analog'):
        #     return
        # time.sleep(3)
        # code = self._arm.set_cgpio_analog(0, 0)
        # if not self._check_code(code, 'set_cgpio_analog'):
        #     return
        # time.sleep(3)
        # code = self._arm.set_cgpio_analog(1, 0)
        # if not self._check_code(code, 'set_cgpio_analog'):
        #     return




        # code = self._arm.close_lite6_gripper()
        # if not self._check_code(code, 'close_lite6_gripper'):
        #     return
            
        # code = self._arm.set_position(*[126.2, -120.5, 231.4, -179.8, 71.5, 127.6], speed=self._angle_speed,
        #                             mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
            
        # code = self._arm.set_position(*[159.7, -140.1, 280.1, 139.7, 77.4, 67.3], speed=self._angle_speed,
        #                             mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return


    # def stop_task(self):
    #     while True:
    #         command = input("Enter 'stop' to pause, 'start' to resume: ")
    #         if command == '1':
    #             # self.invaded = 1
    #             self._arm.set_state(3)
    #         elif command == '2':
    #             # self.invaded = 0
    #             self._arm.set_state(0)
    #             print("stop_task: "+str(self.invaded))


if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.185', baud_checkset=False)
    robot_main = RobotMain(arm)

    # target에 함수 객체를 넘겨줌
    # robot_thread = Thread(target=robot_main.test_main)

    # 스레드 시작
    # robot_thread.start()

    

    # 메인 쓰레드와 stop 쓰레드가 종료되지 않도록 대기
    # robot_thread.join()
    


    # socket_thread = Thread(target=robot_main.socket1_connect)
    # socket_thread.start()
    # print('socket_thread start')
    

    # joint_state_thread = threading.Thread(target=robot_main.joint_state)
    # joint_state_thread.start()
    # print('joint_state_thread_started')
    

    run_thread = threading.Thread(target=robot_main.test_main)
    run_thread.start()
    print('robot_thread_start')
    

    # warning_thread = Thread(target=robot_main.vision)
    # warning_thread.start()
    # print('warning_thread_start')

    # trash_thread = Thread(target=robot_main.trash_position)
    # trash_thread.start()
    # print('trash_thread_start')
    

    # socket_thread.join()
    # joint_state_thread.join()
    run_thread.join()
    # trash_thread.join()
    # warning_thread.join()

    
