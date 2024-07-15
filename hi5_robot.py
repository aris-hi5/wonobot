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
import numpy as np
import time


import random

from hi5_sound import Hi5_Sound

import threading


class RobotMain(object):
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

        self.capsule_position = 0 
        self.detect_sealing = 0

        self.invaded = 0


        # self.position_home = [180, -42.1, 7.4, 186.7, 41.5, 0] #angle
        
        
        self.position_jig_A_grab = [-255.2, -133.8, 200, 68.3, 86.1, -47.0] #linear
        self.position_jig_B_grab = [-152.3, -127.0, 200, 4.8, 89.0, -90.7] #linear
        self.position_jig_C_grab = [-76.6, -144.6, 200, 5.7, 88.9, -50.1] #linear
        self.position_sealing_check = [-136.8, 71.5, 307.6, 69.6, -73.9, -59] #Linear
        self.position_capsule_place = [234.9, 135.9, 465.9, 133.6, 87.2, -142.1] #Linear
        self.position_before_capsule_place = self.position_capsule_place.copy()
        self.position_before_capsule_place[2] += 25
        self.position_cup_grab = [214.0, -100.2, 145.5, -25.6, -88.5, 97.8] #linear

        
        
        self.position_icecream_with_topping = [168.7, 175.6, 359.5, 43.9, 88.3, 83.3] #Linear
        self.position_icecream_no_topping = [48.4, -13.8, 36.3, 193.6, 42.0, -9.2] #angle
        self.position_jig_A_serve = [-258.7, -136.4, 208.2, 43.4, 88.7, -72.2] #Linear
        self.position_jig_B_serve = [-166.8, -126.5, 200.9, -45.2, 89.2, -133.6] #Linear
        self.position_jig_C_serve = [-63.1, -138.2, 199.5, -45.5, 88.1, -112.1] #Linear
        self.position_capsule_grab = [234.2, 129.8, 464.5, -153.7, 87.3, -68.7] #Linear


        #--------Hi5----------
        # self.position_home = [180, -45, 0, 180, 45, 0] #angle
        self.position_home = [180.00002, -24.999982, 14.999978, 180.00002, 50.000021, 0.0]


        self.position_icecream = [243.1, 134.7, 300, -59.6, 88.5, 29.5] #linear
        self.position_finish = [270, -45, 0, 180, 45, 0] #angle
        self.position_topping_A = [-194.485367, 165.352158, 300, 15.641404, 89.68411, 143.873541] #Linear
        self.position_topping_B = [-133.771973, 141.502975, 300, 53.655951, 89.68411, 143.873541] #Linear
        self.position_topping_C = [-63.588264, 163.637115, 300, 90, 89.68411, 143.873541] #Linear

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
                        self.is_alive = False
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

    # =================================  motion  =======================================
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
        self._angle_speed = 80
        self._angle_acc = 200
        try:
            self.clientSocket.send('motion_home_start'.encode('utf-8'))
        except:
            print('socket error')
        print('motion_home start')
        # designed home
        # code = self._arm.set_servo_angle(angle=[179.0, -17.9, 17.7, 176.4, 61.3, 5.4], speed=self._angle_speed,
        #                                  mvacc=self._angle_acc, wait=True, radius=10.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        code = self._arm.set_servo_angle(angle=self.position_home, speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        print('motion_home finish')
        # self.clientSocket.send('motion_home_finish'.encode('utf-8'))

    def motion_grab_capsule(self):

        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return

        # Joint Motion
        self._angle_speed = 100
        self._angle_acc = 100

        self._tcp_speed = 100
        self._tcp_acc = 1000

        '''
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        '''
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        try:
            self.clientSocket.send('motion_grab_capsule_start'.encode('utf-8'))
        except:
            print('socket error')

        # code = self._arm.set_servo_angle(angle=[175.4, 28.7, 23.8, 84.5, 94.7, -5.6], speed=self._angle_speed,
        #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #    return

        if self.order_msg['makeReq']['jigNum'] in ['A']:
            # code = self._arm.set_servo_angle(angle=[166.1, 30.2, 25.3, 75.3, 93.9, -5.4], speed=self._angle_speed,
            #                                  mvacc=self._angle_acc, wait=True, radius=0.0)
            # if not self._check_code(code, 'set_servo_angle'):
            #     return
            pass
        else:

            code = self._arm.set_servo_angle(angle=[176, 31.7, 31, 76.7, 91.2, -1.9], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            # code = self._arm.set_servo_angle(angle=[166.1, 30.2, 25.3, 75.3, 93.9, -5.4], speed=self._angle_speed,
            #                                  mvacc=self._angle_acc, wait=False, radius=20.0)
            # if not self._check_code(code, 'set_servo_angle'):
            #     return


        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)

        if self.order_msg['makeReq']['jigNum'] == 'A':
            code = self._arm.set_servo_angle(angle=[179.5, 33.5, 32.7, 113.0, 93.1, -2.3], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            # code = self._arm.set_position(*[-255.4, -139.3, 193.5, -12.7, 87.2, -126.1], speed=self._tcp_speed,
            #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return

            code = self._arm.set_position(*self.position_jig_A_grab, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif self.order_msg['makeReq']['jigNum'] == 'B':

            code = self._arm.set_position(*self.position_jig_B_grab, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif self.order_msg['makeReq']['jigNum'] == 'C':
            code = self._arm.set_servo_angle(angle=[182.6, 27.8, 27.7, 55.7, 90.4, -6.4], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            # code = self._arm.set_position(*[-76.6, -144.6, 194.3, 5.7, 88.9, -50.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*self.position_jig_C_grab, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return

        time.sleep(1)
        if self.order_msg['makeReq']['jigNum'] == 'C':
            code = self._arm.set_position(z=150, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                          wait=False)
            if not self._check_code(code, 'set_position'):
                return
            self._tcp_speed = 200
            self._tcp_acc = 1000
            code = self._arm.set_tool_position(*[0.0, 0.0, -90.0, 0.0, 0.0, 0.0], speed=self._tcp_speed,
                                               mvacc=self._tcp_acc, wait=False)
            if not self._check_code(code, 'set_position'):
                return
        else:
            code = self._arm.set_position(z=100, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                          wait=False)
            if not self._check_code(code, 'set_position'):
                return

        self._angle_speed = 180
        self._angle_acc = 500

        if self.order_msg['makeReq']['sealing'] in ['yes']:
            code = self._arm.set_servo_angle(angle=[145, -18.6, 10.5, 97.5, 81.4, 145], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        else :
            code = self._arm.set_servo_angle(angle=[146.1, -10.7, 10.9, 102.7, 92.4, 24.9], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        try:
            self.clientSocket.send('motion_grab_capsule_finish'.encode('utf-8'))
        except:
            print('socket error')

    def motion_check_sealing(self):
        print('sealing check')
        self._angle_speed = 200
        self._angle_acc = 200
        self.clientSocket.send('motion_sheck_sealing'.encode('utf-8'))
        code = self._arm.set_position(*self.position_sealing_check, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

    def motion_place_fail_capsule(self):
        code = self._arm.set_servo_angle(angle=[176, -2, 30.7, 79.5, 98.2, 29.3], speed=self._angle_speed,
                                        mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        if self.capsule_position == 1:
            code = self._arm.set_position(*[-255.2, -133.8, 250, 68.3, 86.1, -47.0], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return

            code = self._arm.set_position(*self.position_jig_A_grab, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=20.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif self.capsule_position == 2:
            code = self._arm.set_position(*[-152.3, -127.0, 250, 4.8, 89.0, -90.7], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return

            code = self._arm.set_position(*self.position_jig_B_grab, speed=self._tcp_speed-50,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif self.capsule_position == 3:
            code = self._arm.set_position(*[-76.6, -144.6, 250, 5.7, 88.9, -50.1], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=20.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*self.position_jig_C_grab, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        else:
            pass    
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gri1pper'):
            return
        time.sleep(1)
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.set_tool_position(z=-70, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        self.motion_home()

    # def motion_grab_cup(self):
    #     try:
    #         self.clientSocket.send('motion_grab_cup_start'.encode('utf-8'))
    #     except:
    #         print('socket error')

    #     code = self._arm.set_position(*[233.4, 10.3, 471.1, -172.2, 87.3, -84.5], speed=self._tcp_speed,
    #                                   mvacc=self._tcp_acc, radius=20.0, wait=False)
    #     if not self._check_code(code, 'set_position'):
    #         return
    #     code = self._arm.open_lite6_gripper()
    #     if not self._check_code(code, 'open_lite6_gripper'):
    #         return
    #     time.sleep(1)

    #     if self.order_msg['makeReq']['cupNum'] in ['A', 'B']:
    #         code = self._arm.set_servo_angle(angle=[-2.8, -2.5, 45.3, 119.8, -79.2, -18.8], speed=self._angle_speed,
    #                                          mvacc=self._angle_acc, wait=False, radius=30.0)
    #         if not self._check_code(code, 'set_servo_angle'):
    #             return
    #         # code = self._arm.set_position(*[193.8, -100.2, 146.6, 135.9, -86.0, -55.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
    #         # if not self._check_code(code, 'set_position'):
    #         #    return
    #         code = self._arm.set_position(*[195.0, -96.5, 200.8, -168.0, -87.1, -110.5], speed=self._tcp_speed,
    #                                       mvacc=self._tcp_acc, radius=10.0, wait=False)
    #         if not self._check_code(code, 'set_position'):
    #             return
    #         # code = self._arm.set_position(*[195.0, -96.5, 145.8, -168.0, -87.1, -110.5], speed=self._tcp_speed,
    #         #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
    #         # if not self._check_code(code, 'set_position'):
    #         #    return
    #         # code = self._arm.set_position(*[195.5, -96.6, 145.6, 179.0, -87.0, -97.1], speed=self._tcp_speed,
    #         #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
    #         # if not self._check_code(code, 'set_position'):
    #         #    return
    #         # code = self._arm.set_position(*[214.0, -100.2, 145.0, -25.6, -88.5, 95.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
    #         # if not self._check_code(code, 'set_position'):
    #         #    return
    #         code = self._arm.set_position(*self.position_cup_grab, speed=self._tcp_speed,
    #                                       mvacc=self._tcp_acc, radius=0.0, wait=True)
    #         if not self._check_code(code, 'set_position'):
    #             return
    #     code = self._arm.close_lite6_gripper()
    #     if not self._check_code(code, 'close_lite6_gripper'):
    #         return
    #     time.sleep(2)

    #     code = self._arm.set_position(z=120, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
    #                                   wait=True)
    #     if not self._check_code(code, 'set_position'):
    #         return
    #     code = self._arm.set_servo_angle(angle=[2.9, -31.0, 33.2, 125.4, -30.4, -47.2], speed=self._angle_speed,
    #                                      mvacc=self._angle_acc, wait=True, radius=0.0)
    #     if not self._check_code(code, 'set_servo_angle'):
    #         return

    #     code = self._arm.set_cgpio_analog(0, 5)
    #     if not self._check_code(code, 'set_cgpio_analog'):
    #         return
    #     code = self._arm.set_cgpio_analog(1, 5)
    #     if not self._check_code(code, 'set_cgpio_analog'):
    #         return
    #     try:
    #         self.clientSocket.send('motion_grab_cup_finish'.encode('utf-8'))
    #     except:
    #         print('socket error')

    #     time.sleep(0.5)

    # def motion_topping(self):
    #     try:
    #         self.clientSocket.send('motion_topping_start'.encode('utf-8'))
    #     except:
    #         print('socket error')

    #     print('send')

    #     if self.order_msg['makeReq']['topping'] == '1':
    #         code = self._arm.set_servo_angle(angle=[36.6, -36.7, 21.1, 85.6, 59.4, 44.5], speed=self._angle_speed,
    #                                          mvacc=self._angle_acc, wait=True, radius=0.0)
    #         if not self._check_code(code, 'set_servo_angle'):
    #             return

    #         if self.order_msg['makeReq']['jigNum'] == 'C':
    #             code = self._arm.set_position(*self.position_topping_C, speed=self._tcp_speed,
    #                                           mvacc=self._tcp_acc, radius=0.0, wait=True)
    #             if not self._check_code(code, 'set_position'):
    #                 return
    #             code = self._arm.set_cgpio_digital(2, 1, delay_sec=0)
    #             if not self._check_code(code, 'set_cgpio_digital'):
    #                 return
    #             code = self._arm.set_position(z=20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
    #                                           wait=False)
    #             if not self._check_code(code, 'set_position'):
    #                 return
    #             code = self._arm.set_pause_time(int(self.order_msg['makeReq']['toppingAmount']) - 3)
    #             if not self._check_code(code, 'set_pause_time'):
    #                 return
    #             code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
    #             if not self._check_code(code, 'set_cgpio_digital'):
    #                 return
    #             code = self._arm.set_pause_time(3)
    #             if not self._check_code(code, 'set_pause_time'):
    #                 return
    #             code = self._arm.set_cgpio_digital(2, 0, delay_sec=0)
    #             if not self._check_code(code, 'set_cgpio_digital'):
    #                 return

    #             code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc,
    #                                           relative=True, wait=False)
    #             if not self._check_code(code, 'set_position'):
    #                 return

    #         elif self.order_msg['makeReq']['jigNum'] in ['B']:
    #             code = self._arm.set_servo_angle(angle=[55.8, -48.2, 14.8, 86.1, 60.2, 58.7], speed=self._angle_speed,
    #                                              mvacc=self._angle_acc, wait=False, radius=20.0)
    #             if not self._check_code(code, 'set_servo_angle'):
    #                 return
    #             # code = self._arm.set_servo_angle(angle=[87.5, -48.2, 13.5, 125.1, 44.5, 46.2], speed=self._angle_speed,
    #             #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
    #             # if not self._check_code(code, 'set_servo_angle'):
    #             #    return
    #             code = self._arm.set_servo_angle(angle=self.position_topping_B, speed=self._angle_speed,
    #                                              mvacc=self._angle_acc, wait=True, radius=0.0)
    #             if not self._check_code(code, 'set_servo_angle'):
    #                 return
    #             code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
    #             if not self._check_code(code, 'set_cgpio_digital'):
    #                 return
    #             code = self._arm.set_position(z=20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
    #                                           wait=True)
    #             if not self._check_code(code, 'set_position'):
    #                 return
    #             code = self._arm.set_pause_time(int(self.order_msg['makeReq']['toppingAmount']) - 4)
    #             if not self._check_code(code, 'set_pause_time'):
    #                 return
    #             code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
    #             if not self._check_code(code, 'set_cgpio_digital'):
    #                 return
    #             code = self._arm.set_pause_time(4)
    #             if not self._check_code(code, 'set_pause_time'):
    #                 return
    #             code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
    #             if not self._check_code(code, 'set_cgpio_digital'):
    #                 return
    #             code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc,
    #                                           relative=True, wait=False)
    #             if not self._check_code(code, 'set_position'):
    #                 return
    #             code = self._arm.set_servo_angle(angle=[87.5, -48.2, 13.5, 125.1, 44.5, 46.2], speed=self._angle_speed,
    #                                              mvacc=self._angle_acc, wait=False, radius=10.0)
    #             if not self._check_code(code, 'set_servo_angle'):
    #                 return
    #             code = self._arm.set_position(*[43.6, 137.9, 350.1, -92.8, 87.5, 5.3], speed=self._tcp_speed,
    #                                           mvacc=self._tcp_acc, radius=10.0, wait=False)
    #             if not self._check_code(code, 'set_position'):
    #                 return

    #         elif self.order_msg['makeReq']['jigNum'] == 'A':
    #             code = self._arm.set_position(*self.position_topping_A, speed=self._tcp_speed,
    #                                           mvacc=self._tcp_acc, radius=0.0, wait=True)
    #             if not self._check_code(code, 'set_position'):
    #                 return
    #             code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
    #             if not self._check_code(code, 'set_cgpio_digital'):
    #                 return
    #             code = self._arm.set_pause_time(int(self.order_msg['makeReq']['toppingAmount']) - 1)
    #             if not self._check_code(code, 'set_pause_time'):
    #                 return
    #             code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
    #             if not self._check_code(code, 'set_cgpio_digital'):
    #                 return
    #             code = self._arm.set_pause_time(1)
    #             if not self._check_code(code, 'set_pause_time'):
    #                 return
    #             code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
    #             if not self._check_code(code, 'set_cgpio_digital'):
    #                 return
    #             code = self._arm.set_servo_angle(angle=[130.0, -33.1, 12.5, 194.3, 51.0, 0.0], speed=self._angle_speed,
    #                                              mvacc=self._angle_acc, wait=True, radius=0.0)
    #             if not self._check_code(code, 'set_servo_angle'):
    #                 return
    #             code = self._arm.set_position(*[-38.2, 132.2, 333.9, -112.9, 86.3, -6.6], speed=self._tcp_speed,
    #                                           mvacc=self._tcp_acc, radius=10.0, wait=False)
    #             if not self._check_code(code, 'set_position'):
    #                 return
    #             code = self._arm.set_position(*[43.6, 137.9, 350.1, -92.8, 87.5, 5.3], speed=self._tcp_speed,
    #                                           mvacc=self._tcp_acc, radius=10.0, wait=False)
    #             if not self._check_code(code, 'set_position'):
    #                 return
    #         # code = self._arm.set_position(*[165.1, 162.9, 362.5, -31.7, 86.6, 9.5], speed=self._tcp_speed,
    #         #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
    #         # if not self._check_code(code, 'set_position'):
    #         #    return
    #         code = self._arm.set_position(*self.position_icecream_with_topping, speed=self._tcp_speed,
    #                                       mvacc=self._tcp_acc, radius=0.0, wait=True)
    #         if not self._check_code(code, 'set_position'):
    #             return
    #     else:
    #         # code = self._arm.set_servo_angle(angle=[45.8, -17.9, 33.5, 186.9, 41.8, -7.2], speed=self._angle_speed,
    #         #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
    #         # if not self._check_code(code, 'set_servo_angle'):
    #         #    return
    #         code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
    #         if not self._check_code(code, 'set_cgpio_digital'):
    #             return
    #         code = self._arm.set_servo_angle(angle=self.position_icecream_no_topping, speed=self._angle_speed,
    #                                          mvacc=self._angle_acc, wait=True, radius=0.0)
    #         if not self._check_code(code, 'set_servo_angle'):
    #             return
    #     try:
    #         self.clientSocket.send('motion_topping_finish'.encode('utf-8'))
    #     except:
    #         print('socket error')

    #     time.sleep(0.5)

    # def motion_make_icecream(self):
    #     try:
    #         self.clientSocket.send('motion_make_icecream_start'.encode('utf-8'))
    #     except:
    #         print('socket error')
    #     if self.order_msg['makeReq']['topping'] == '1':
    #         time.sleep(5)
    #     else:
    #         time.sleep(8)
    #     try:
    #         self.clientSocket.send('motion_icecreaming_1'.encode('utf-8'))
    #     except:
    #         print('socket error')
    #     time.sleep(4)
    #     code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
    #                                   wait=True)
    #     if not self._check_code(code, 'set_position'):
    #         return
    #     try:
    #         self.clientSocket.send('motion_icecreaming_2'.encode('utf-8'))
    #     except:
    #         print('socket error')
    #     time.sleep(4)
    #     code = self._arm.set_position(z=-10, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
    #                                   wait=True)
    #     if not self._check_code(code, 'set_position'):
    #         return
    #     if not self._check_code(code, 'set_pause_time'):
    #         return
    #     try:
    #         self.clientSocket.send('motion_icecreaming_3'.encode('utf-8'))
    #     except:
    #         print('socket error')
    #     code = self._arm.set_position(z=-50, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
    #                                   wait=True)
    #     if not self._check_code(code, 'set_position'):
    #         return
    #     time.sleep(1)
    #     code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
    #     if not self._check_code(code, 'set_cgpio_digital'):
    #         return
    #     try:
    #         self.clientSocket.send('motion_make_icecream_finish'.encode('utf-8'))
    #     except:
    #         print('socket error')
    #     time.sleep(0.5)

    def motion_serve(self):
        try:
            self.clientSocket.send('motion_serve_start'.encode('utf-8'))
        except:
            print('socket error')
        code = self._arm.set_servo_angle(angle=[18.2, -12.7, 8.3, 90.3, 88.1, 23.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[146.9, -12.7, 8.3, 91.0, 89.3, 22.1], speed=self._angle_speed,
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
                    print('sendsendsendsnedasdhfaenbeijakwlbrsvz;ikbanwzis;fklnairskjf')
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

    
#-----------------------------------------------Hi5------------------------------------------------
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
                    if len(approx) <= 10:  # Assuming a star has approximately 10 points
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

                self.detect_sealing = 2

            if cv2.waitKey(1) & self.detect_sealing >= 1:
                break
        # 캡처 객체와 모든 창을 해제 및 닫습니다.
        cap.release()
        cv2.destroyAllWindows()

    def motion_grap_squeegee(self):
        # 스퀴지 웨이포인트
        code = self._arm.set_servo_angle(angle=[160.191895, 29.03074, 25.689881, 197.292459, 73.301699, 177.033703], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # 스퀴지 위치
        code = self._arm.set_position(*[-274.5, 135.3, 148.5, 128, -86.5, -179.1], speed=20,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        # 스퀴지 잡고 올리기
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(2)
        code = self._arm.set_position(z=20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_servo_angle(angle=[179.377616, -8.25065, 21.099343, 179.901127, 60.182169, -0.285619], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        

    def motion_Dondurma(self):
        Dondurma_start_position = [0.0, -500, 285, 54.735632, 89.999981, -35.264406] #linear
        Dondurma_end_position = [0.0, -180, 285, 54.735632, 89.999981, -35.264406] #linear

        code = self._arm.set_position(*Dondurma_start_position, speed=50,
                                            mvacc=5000, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        Dondurma_count = random.randrange(1,5)
        Dondurma_y_axis = 500

        for i in range(0,Dondurma_count):      
            Dondurma_y_axis = Dondurma_y_axis-abs((Dondurma_start_position[1]-Dondurma_end_position[1])/Dondurma_count)
            Dondurma_x_axis = random.randrange(-260,260)
            Dondurma_z_axis = random.randrange(230,340)

            print("{} / {}  y = {} x = {} z = {}" .format(i,Dondurma_count,Dondurma_y_axis,Dondurma_x_axis,Dondurma_z_axis))

            code = self._arm.set_position(*[Dondurma_x_axis, -Dondurma_y_axis, Dondurma_z_axis, Dondurma_end_position[3], Dondurma_end_position[4], Dondurma_end_position[5]], 
                                        speed=450, mvacc=5000, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
        code = self._arm.set_position(*Dondurma_end_position,
                                        speed=50, mvacc=5000, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # print(Dondurma_count)
        # print(Dondurma_start_position[1] - Dondurma_end_position[1])
        # print(abs((Dondurma_start_position[1]-Dondurma_end_position[1])/Dondurma_count))

        # code = self._arm.set_position(y=abs((Dondurma_start_position[1]-Dondurma_end_position[1])/Dondurma_count), radius=0, speed=10, mvacc=50, relative=True, wait=False)
        # if not self._check_code(code, 'set_position'):
        #     return
        
    def motion_place_capsule(self):
        try:
            self.clientSocket.send('motion_place_capsule_start'.encode('utf-8'))
        except:
            print('socket error')

        #토핑 아래로 지나가는 1
        code = self._arm.set_servo_angle(angle=[81.0, -10.8, 6.9, 103.6, 88.6, 9.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        #토핑 아래로 지나가는 2
        code = self._arm.set_servo_angle(angle=[10, -20.8, 7.1, 106.7, 79.9, 26.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # code = self._arm.set_servo_angle(angle=[27.0, -24.9, 7.2, 108.0, 76.4, 32.7], speed=self._angle_speed,
        #                                 mvacc=self._angle_acc, wait=False, radius=40.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #    return
        # code = self._arm.set_servo_angle(angle=[-0.9, -24.9, 10.4, 138.3, 66.0, 19.1], speed=self._angle_speed,
        #                                 mvacc=self._angle_acc, wait=False, radius=40.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #    return

        #토핑 아래로 지나 올라옴
        code = self._arm.set_servo_angle(angle=[8.4, -42.7, 23.7, 177.4, 31.6, 3.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # code = self._arm.set_servo_angle(angle=[8.4, -33.1, 51.8, 100.6, 29.8, 77.3], speed=self._angle_speed,
        #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #    return

        #탬핑기 바로 앞
        code = self._arm.set_servo_angle(angle=[8.4, -32.1, 55.1, 96.6, 29.5, 81.9], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # code = self._arm.set_position(*[241.7, 122.5, 487.8, -140, 86.1, -52.4], speed=self._tcp_speed,
        #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #    return
        # code = self._arm.set_position(*[241.7, 122.5, 467.8, -140, 86.1, -52.4], speed=self._tcp_speed,
        #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #    return
        # code = self._arm.set_position(*[234.9, 135.9, 486.5, 133.6, 87.2, -142.1], speed=self._tcp_speed,
        #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #    return

        #캡슐 삽입
        code = self._arm.set_position(*self.position_before_capsule_place, speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        #캡슐 내려놓음
        code = self._arm.set_position(*self.position_capsule_place, speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=0.0, wait=True)
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
        code = self._arm.set_position(*[233.4, 10.3, 471.1, -172.2, 87.3, -84.5], speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        try:
            self.clientSocket.send('motion_place_capsule_finish'.encode('utf-8'))
        except:
            print('socket error')
        time.sleep(0.5)

    def motion_grab_cup(self):
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return

        # 컵가는 웨이포인트
        code = self._arm.set_servo_angle(angle=[-2.8, -2.5, 45.3, 119.8, -79.2, -18.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        #컵 위치로 이동 잡기
        code = self._arm.set_position(*self.position_cup_grab, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
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
        code = self._arm.set_position(z=100, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                      wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        # 돌아가는 웨이포인트
        code = self._arm.set_servo_angle(angle=[2.9, -28.9, 26.2, 110.2, -26.1, -30.1], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # 돌아가는 웨이포인트
        code = self._arm.set_servo_angle(angle=[3, -8.7, 26.2, 107.4, 60.5, 19.7], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # 아이스크림 위치
        code = self._arm.set_position(*[243.1, 134.7, 300, -59.6, 88.5, 29.5], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # # 토핑가는 웨이포인트
        # code = self._arm.set_servo_angle(angle=[34.5, -33.9, 18, 101.1, 73.8, 49.2], speed=self._angle_speed,
        #                                     mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return



    def motion_helix(self, radius, height, turns, mode):
        # 나선의 중점 설정
        if mode == 1:
            icecream_center = self.position_icecream
            icecream_center[2] = icecream_center[2]+60

            helix_position = icecream_center

        elif mode == 2:
            topping_A_center = self.position_topping_A
            topping_A_center[2] = topping_A_center[2]+30

            helix_position = topping_A_center
        elif mode == 3:
            topping_B_center = self.position_topping_B
            topping_B_center[2] = topping_B_center[2]+30

            helix_position = topping_B_center
        elif mode == 4:
            topping_C_center = self.position_topping_C
            topping_C_center[2] = topping_C_center[2]+30

            helix_position = topping_C_center
            

        # t 값 생성
        t_values = np.linspace(0, turns * 2 * np.pi, 100)  # 100개의 점을 생성

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
                                        speed=20, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return

        # 나선을 따라 이동
        for x, y, z in zip(x_values, y_values, z_values):
            code = self._arm.set_position(*[x, y, z, helix_position[3], helix_position[4], helix_position[5]], 
                                        speed=20, mvacc=self._tcp_acc, radius=30.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return

        # 끝점으로 이동
        code = self._arm.set_position(*[helix_position[0], helix_position[1], end_point[2],
                                        helix_position[3], helix_position[4], helix_position[5]], 
                                        speed=20, mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
    def motion_make_icecream(self):
        # 아이스크림 위치
        code = self._arm.set_position(*[243.1, 134.7, 300, -59.6, 88.5, 29.5], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=20.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        # 아이스크림 추출 준비
        code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        
        code = self._arm.set_position(z=60, radius=0, speed=20, mvacc=self._tcp_acc, relative=True,
                                        wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        # code = self._arm.set_pause_time(3)
        # if not self._check_code(code, 'set_pause_time'):
        #     return

        time.sleep(7)

        self.motion_helix(8, -1, 5, 1)

        time.sleep(3)
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return

    def motion_icecream_to_topping(self):
        #토핑 아래로 지나가는
        code = self._arm.set_servo_angle(angle=[16.816311, -37.649286, 4.669835, 96.031508, 82.383711, 41.868891], speed=20,
                                         mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # 토핑 아래
        code = self._arm.set_servo_angle(angle=[126.573252, -53.387978, 1.068108, 182.537822, 35.668972, -4.317638], speed=20,
                                         mvacc=self._angle_acc, wait=False, radius=00.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

    def motion_get_topping(self, topping):
        if topping == 1:
            #-----------토핑 A----------
            code = self._arm.set_position(*self.position_topping_B, speed=20,
                                            mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return 

            code = self._arm.set_position(*[-194.485367, 165.352158, 300, 15, 89.68411, 143.873541], speed=20,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return 
            
            code = self._arm.set_position(z=30, radius=0, speed=20, mvacc=self._tcp_acc, relative=True,
                                            wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            self.motion_helix(8, -1, 5, 2)
        elif topping == 2:
            #-----------토핑 B----------
            code = self._arm.set_position(*self.position_topping_B, speed=20,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return 
            
            code = self._arm.set_position(z=30, radius=0, speed=20, mvacc=self._tcp_acc, relative=True,
                                            wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            self.motion_helix(8, -1, 5, 3)

        elif topping == 3:
            #-----------토핑 C----------
            code = self._arm.set_position(*self.position_topping_B, speed=20,
                                            mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return 

            code = self._arm.set_position(*self.position_topping_C, speed=20,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return 
            
            code = self._arm.set_position(z=30, radius=0, speed=20, mvacc=self._tcp_acc, relative=True,
                                            wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            self.motion_helix(8, -1, 5, 4)
        
    def motion_vent(self):
        # 배출 웨이포인트
        code = self._arm.set_servo_angle(angle=[180.0, -15.599979, 9.499984, 187.799981, 64.499979, 0.0,], speed=30,
                                         mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_servo_angle(angle=self.position_finish, speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

# code = self._arm.set_position(*[-178.926392, -47.471741, 200, -41.833539, 88.173277, -122.563235], speed=self._tcp_speed-50,
#                                 mvacc=self._tcp_acc, radius=0.0, wait=True)
# if not self._check_code(code, 'set_position'):
#     return

# code = self._arm.set_tool_position(*[0.0, 0.0, -5.0, 0.0, 0.0, 0.0], speed=10,
#                                     mvacc=self._tcp_acc, wait=False)
# if not self._check_code(code, 'set_position'):
#     return


# print('get_position :{0} \nget_position(radian) :{1} \nget_servo_angle :{2}'.format(arm.get_position(), arm.get_position(is_radian=True), arm.get_servo_angle()))

        # while True:
        #     print("!!!!!!!!!!!!!!1", self._arm._arm.get_common_info(101, return_val=True))
        # # print("!!!!!!!!!!!!!!1", self._arm.get_c31_error_info())

    

    

    def test_main(self):

        # print('get_position :{0} \nget_position(radian) :{1} \nget_servo_angle :{2}'.format(arm.get_position(), arm.get_position(is_radian=True), arm.get_servo_angle()))



        # self.trash_check()


        # # 토핑 1
       
        # code = self._arm.set_position(*self.position_topping_A, speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return
        # code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
        # if not self._check_code(code, 'set_cgpio_digital'):
        #     return
        # # 토핑 양만큼 멈춤
        # # code = self._arm.set_pause_time(int(self.order_msg['makeReq']['toppingAmount']) - 1)
        # # if not self._check_code(code, 'set_pause_time'):
        # #     return
        # code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
        # if not self._check_code(code, 'set_cgpio_digital'):
        #     return
        # code = self._arm.set_pause_time(1)
        # if not self._check_code(code, 'set_pause_time'):
        #     return
        # code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
        # if not self._check_code(code, 'set_cgpio_digital'):
        #     return
        # code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        # if not self._check_code(code, 'set_cgpio_digital'):
        #     return
        
        #-----토핑 시작-----
        # # def motion_topping(self):
        # try:
        #     self.clientSocket.send('motion_topping_start'.encode('utf-8'))
        # except:
        #     print('socket error')

        # print('send')

        # # 토핑가는 웨이포인트
        # code = self._arm.set_servo_angle(angle=[34.5, -33.9, 18, 101.1, 73.8, 49.2], speed=self._angle_speed,
        #                                     mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        
        # # user_input = input()
        # # if user_input == 'q':

        # code = self._arm.set_servo_angle(angle=[55.8, -48.2, 14.8, 86.1, 60.2, 58.7], speed=20,
        #                                          mvacc=self._angle_acc, wait=False, radius=20.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return

        # self.motion_grab_cup()


        # self.motion_make_icecream()


    
        # self.motion_home()

        # #실링체크 웨이포인트
        # code = self._arm.set_servo_angle(angle=[145, -27, 13.4, 95.7, 80.1, 156.4], speed=self._angle_speed,
        #                                     mvacc=self._angle_acc, wait=False, radius=30.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return

        # #실링체크
        # code = self._arm.set_servo_angle(angle=[179.1, -85.6, 13.1, 182.6, -3.2, 180], speed=self._angle_speed,
        #                                     mvacc=self._angle_acc, wait=True, radius=30.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        
        # self.motion_place_capsule()

        

        

        #  def motion_make_icecream(self):
        # 아이스크림 위치
        # code = self._arm.set_position(*[243.1, 134.7, 300, -59.6, 88.5, 29.5], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=20.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return 
        
        # code = self._arm.set_position(z=65, radius=0, speed=20, mvacc=self._tcp_acc, relative=True,
        #                                 wait=False)
        # if not self._check_code(code, 'set_position'):
        #     return

        # self.motion_grab_cup()



        # self.motion_make_icecream()
        # # print('get_position :{0} \nget_position(radian) :{1} \nget_servo_angle :{2}'.format(arm.get_position(), arm.get_position(is_radian=True), arm.get_servo_angle()))

    
        # self.motion_Dondurma()

        # 끊어서 가기
        #def motion_safemove(self, mode, goal, N):

        # get_position :(0, [-189.92067, -1.2e-05, 306.132355, 76.097733, 89.999981, -103.902287]) 
        # get_position(radian) :(0, [-189.92067, -1.2e-05, 306.132355, 1.328156, 1.570796, -1.813437]) 
        # get_servo_angle :(0, [180.00002, -24.999982, 14.999978, 180.00002, 50.000021, 0.0, 0.0])


        # get_position :(0, [-2e-06, -189.92067, 306.132355, 76.097733, 89.999981, -13.902305]) 
        # get_position(radian) :(0, [-2e-06, -189.92067, 306.132355, 1.328156, 1.570796, -0.242641]) 
        # get_servo_angle :(0, [270.000001, -24.999982, 14.999978, 180.00002, 50.000021, 0.0, 0.0])

        self.motion_home()

        mode = 2
        N = 10

        goal = [270.000001, -24.999982, 14.999978, 180.00002, 50.000021, 0.0]

        self._angle_speed = 30
        self._angle_acc = 500

        if mode == 1:
            data = arm.get_position()
            extracted_list = data[1]
            pre_position = extracted_list[:6]

            linspace_lists = [np.linspace(start, end, N) for start, end in zip(pre_position, goal)]

            # 결과 출력
            for i in range(N):
                step = [lst[i] for lst in linspace_lists]

                if self.invaded == 0:
                    code = self._arm.set_position(*step, speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                    if not self._check_code(code, 'set_position'):
                        return 
                    # time.sleep(0.1)
                    print("position: "+str(self.invaded))
                elif self.invaded == 1:
                    while True:
                        time.sleep(0.1)
                        if self.invaded == 0:
                            break
            
        elif mode == 2:
            data = arm.get_servo_angle()
            extracted_list = data[1]
            pre_angle = extracted_list[:6]

            linspace_lists = [np.linspace(start, end, N) for start, end in zip(pre_angle, goal)]

            time.sleep(5)
            # 결과 출력
            for i in range(N):
                step = [lst[i] for lst in linspace_lists]
                print("motion: "+str(self.invaded))

                if self.invaded == 0:
                    code = self._arm.set_servo_angle(angle=step, speed=self._angle_speed, mvacc=self._angle_acc, radius=0.0, wait=False)
                    if not self._check_code(code, 'set_servo_angle'):
                        return
                    # time.sleep(0.1)
                    print("motion: "+str(self.invaded))
                elif self.invaded == 1:
                    while True:
                        time.sleep(0.1)
                        if self.invaded == 0:
                            break
     
        '''
        self.motion_home()
        time.sleep(10)

        # motion_grab_capsule
        # 컵 추출 준비
        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        
        # Joint Motion
        self._angle_speed = 100
        self._angle_acc = 100

        self._tcp_speed = 100
        self._tcp_acc = 1000

        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)


        # user_input = input()
        self.check_capsule_position()
        

        # 지그 가는 웨이포인트
        code = self._arm.set_servo_angle(angle=[176, 27, 29.9, 76.8, 92, 0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return


        if self.capsule_position == 1:
        # if user_input == '1':
            # A지그 가는 웨이포인트
            code = self._arm.set_servo_angle(angle=[179.5, 28.3, 31.4, 113.1, 91.5, 0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            code = self._arm.set_position(*self.position_jig_A_grab, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif self.capsule_position == 2:
        # elif user_input == '2':
            # B 지그 캡슐 그랩
            code = self._arm.set_position(*self.position_jig_B_grab, speed=self._tcp_speed-50,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif self.capsule_position == 3:
        # elif user_input == '3':
            #C 지금 잡으러가는 웨이포인트
            code = self._arm.set_servo_angle(angle=[182.6, 25.3, 27.2, 55.7, 91.5, 0], speed=self._angle_speed,
                                                 mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            code = self._arm.set_position(*self.position_jig_C_grab, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return


        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gri1pper'):
            return
        time.sleep(1)

        code = self._arm.set_position(z=100, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        #실링체크 웨이포인트
        code = self._arm.set_servo_angle(angle=[145, -27, 13.4, 95.7, 80.1, 156.4], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        # motion_check_sealing()
        print('sealing check')
        self._angle_speed = 100
        self._angle_acc = 200
        # code = self._arm.set_position(*self.position_sealing_check, speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=0.0, wait=True)
        code = self._arm.set_servo_angle(angle=[179.1, -85.6, 13.1, 182.6, -3.2, 180], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        self.check_sealing()

        if self.detect_sealing== 1:
            self.motion_place_capsule()

        elif self.detect_sealing == 2:
            self.motion_place_fail_capsule()


        ''' 

     




        ''' 토핑 조합
        empty = 0
        oreo = 1
        chocoball = 2
        cereal = 3
        oreo chocoball = 4
        oreo cereal = 5
        chocoball cereal = 6
        oreo chocoball cereal = 7
        '''

        '''

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




        # # self.motion_serve()
        # # 서빙 웨이포인트1
        # code = self._arm.set_servo_angle(angle=[18.2, -12.7, 8.3, 90.3, 88.1, 23.6], speed=self._angle_speed,
        #                                  mvacc=self._angle_acc, wait=False, radius=20.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        # # 서빙 웨이포인트2
        # code = self._arm.set_servo_angle(angle=[146.9, -12.7, 8.3, 91.0, 89.3, 22.1], speed=self._angle_speed,
        #                                  mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return

        # self._tcp_speed = 100
        # self._tcp_acc = 1000

        # # A배출구 위
        # code = self._arm.set_position(*self.position_jig_A_serve, speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return
        # # 내려 놓기
        # code = self._arm.set_position(z=-18, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
        #                                 wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return
        # code = self._arm.open_lite6_gripper()
        # if not self._check_code(code, 'open_lite6_gripper'):
        #     return
        # time.sleep(1)

        # # 내려 놓고 복귀 웨이포인트
        # code = self._arm.set_position(*[-256.2, -126.6, 210.1, -179.2, 77.2, 66.9], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return
        # code = self._arm.stop_lite6_gripper()
        # if not self._check_code(code, 'stop_lite6_gripper'):
        #     return
        # time.sleep(0.5)
        
        # # 내려 놓고 복귀 웨이포인트2
        # code = self._arm.set_position(*[-242.8, -96.3, 210.5, -179.2, 77.2, 66.9], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return

        # # 내려 놓고 복귀 웨이포인트2
        # code = self._arm.set_position(*[-189.7, -26.0, 193.3, -28.1, 88.8, -146.0], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return

        # # 내려 놓고 복귀 웨이포인트3
        # code = self._arm.set_servo_angle(angle=[169.6, -8.7, 13.8, 85.8, 93.7, 19.0], speed=self._angle_speed,
        #                                  mvacc=self._angle_acc, wait=True, radius=10.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return

        # self._tcp_speed = 100
        # self._tcp_acc = 1000



        # self.motion_trash_capsule()

        # self.motion_home()









        # print('get_position :{0} \nget_position(radian) :{1} \nget_servo_angle :{2}'.format(arm.get_position(), arm.get_position(is_radian=True), arm.get_servo_angle()))
        
        # self.motion_grap_squeegee()

        
        



        # 하이파이브 모션
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

    def stop_task(self):
        while True:
            command = input("Enter 'stop' to pause, 'start' to resume: ")
            if command == '1':
                # self.invaded = 1
                self._arm.set_state(3)
            elif command == '2':
                # self.invaded = 0
                self._arm.set_state(0)
                print("stop_task: "+str(self.invaded))


if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.185', baud_checkset=False)
    robot_main = RobotMain(arm)
    # target에 함수 객체를 넘겨줌
    robot_thread = threading.Thread(target=robot_main.test_main)
    stop_thread = threading.Thread(target=robot_main.stop_task)

    # 스레드 시작
    robot_thread.start()
    stop_thread.start()

    # 메인 쓰레드와 stop 쓰레드가 종료되지 않도록 대기
    robot_thread.join()
    stop_thread.join()
    


    # socket_thread = Thread(target=robot_main.socket_connect)
    # socket_thread.start()
    # print('socket_thread start')
    # joint_state_thread = threading.Thread(target=robot_main.joint_state)
    # joint_state_thread.start()
    # print('joint_state_thread_started')
    # run_thread = threading.Thread(target=robot_main.run)
    # run_thread.start()
    # print('run_thread_started')
