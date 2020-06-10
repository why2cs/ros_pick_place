#!/usr/bin/env python
# -*- coding: utf_8 -*-

from __future__ import print_function
import socket
import json
import time
import sys


class Robot:

    def __init__(self, address='192.168.3.100', port=10001):
        '''创建指定通讯端口的机械臂实例'''

        self.__robotAddressPort = (address, port)
        self.__robotSocket = None
        self.__robotIsDisconnected = True

    def robotConnect(self):
        '''建立与机械臂的连接'''

        while self.__robotIsDisconnected:
            try:
                self.__robotSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.__robotSocket.connect(self.__robotAddressPort)
                self.__robotIsDisconnected=False
                print("【机械臂】 通讯建立成功！")
            except Exception as exc:
                print("【机械臂】 通讯建立失败！正在重连……", exc)
                time.sleep(5)

    def robotDisConnect(self):
        '''断开与机械臂的连接'''

        self.__robotSocket.close()
        self.__robotIsDisconnected=True
        print("【机械臂】 通讯断开成功！")

    def robotCheckConnected(self):
        '''确保与机械臂的连接正常'''

        if self.__robotIsDisconnected:
            self.robotConnect()

    def robotPowerOn(self):
        '''机械臂上电启动'''

        self.robotCheckConnected()

        stateJson=json.loads(self.robotAction('{"cmdName":"get_robot_state"}'))
        powerState=stateJson["power"]
        enabledState=stateJson["enable"]

        while powerState!="powered_on" or  enabledState!="robot_enabled":
            if powerState!="powered_on":
                self.robotAction('{"cmdName":"power_on"}')
                time.sleep(10)
            if enabledState!="robot_enabled":
                self.robotAction('{"cmdName":"enable_robot"}')
                time.sleep(10)
            stateJson=json.loads(self.robotAction('{"cmdName":"get_robot_state"}'))
            powerState=stateJson["power"]
            enabledState=stateJson["enable"]

    def robotPowerOff(self):
        '''机械臂关闭断电'''

        if self.robotIsRunning():
            self.robotProgramStop()
            time.sleep(10)

        self.robotCheckConnected()
        self.robotAction('{"cmdName":"disable_robot"}')
        time.sleep(10)
        self.robotAction('{"cmdName":"power_off"}')
        time.sleep(10)
        self.robotDisConnect()

    def robotIsRunning(self):
        '''检查机械臂是否为运行状态，返回布尔值'''

        self.robotCheckConnected()
        stateJson=json.loads(self.robotAction('{"cmdName":"get_program_state"}'))
        if stateJson["errorCode"]=="0" and stateJson["programState"] == "idle":
            return False
        else:
            return True

    def robotProgramRun(self, programName):
        '''运行通过app编程环境开发的机械臂程序，参数为app中的程序名称'''

        self.robotCheckConnected()
        self.robotAction('{{"cmdName":"load_program","programName":"{}.ngc"}}'.format(programName))
        self.robotAction('{"cmdName":"play_program"}')

    def robotProgramPause(self):
        '''暂停机械臂程序运行'''

        self.robotCheckConnected()
        self.robotAction('{"cmdName":"pause_program"}')

    def robotProgramResume(self):
        '''恢复机械臂程序运行'''

        self.robotCheckConnected()
        self.robotAction('{"cmdName":"resume_program"}')

    def robotProgramStop(self):
        '''停止机械臂程序运行'''

        self.robotCheckConnected()
        self.robotAction('{"cmdName":"stop_program"}')

    def robotJointMove(self, J1, J2, J3, J4, J5, J6, speed=50.0, isAbsoluteMotion=True):
        '''控制机械臂关节运动到指定位置
        参数:   J1-J6:机械臂1-6轴各关节角度值，单位为角度
                speed:关节运动速度，单位是 (度/秒)
                isAbsoluteMotion：True为绝对运动，False为相对运动
        返回值： 0：正常'''

        if isAbsoluteMotion:
            flag = 0
        else:
            flag = 1
        self.robotCheckConnected()
        stateJson = json.loads(self.robotAction('{{"cmdName":"joint_move","jointPosition":[{},{},{},{},{},{}],"speed":{}, "relFlag":{}}}'.format(J1, J2, J3, J4, J5, J6, speed, flag)))
        return stateJson["errorCode"]

    def robotEndMove(self, x, y, z, a, b, c, speed=50.0):
        '''控制机械臂末端运动到指定位置
        参数:   x,y,z,a,b,c:机械臂末端在机械臂笛卡尔坐标系中的位置。xyz单位为mm，abc单位为角度
                speed:关节运动速度，单位是 (度/秒)
        返回值： 0：正常
                2：逆解失败'''

        self.robotCheckConnected()
        stateJson = json.loads(self.robotAction('{{"cmdName":"end_move","endPosition":[{},{},{},{},{},{}], "speed":{}}}'.format(x, y, z, a, b, c, speed)))
        return stateJson["errorCode"]

    def robotMoveWaitComplete(self):
        '''控制机械臂阻塞移动'''

        self.robotCheckConnected()
        self.robotAction('{"cmdName":"wait_complete"}')

    def robotPoseStand(self):
        '''机械臂移动至竖直姿态'''

        self.robotCheckConnected()
        self.robotJointMove(0, 90, 0, 90, 180, 0, 20.0, True)        #   [0, 210, 874, -90, 0, 0, 20.0]
        self.robotMoveWaitComplete()

    def robotPoseHome(self):
        '''机械臂移动至Home姿态'''

        self.robotCheckConnected()
        self.robotJointMove(0, 90, 90, 90, -90, 0, 20.0, True)     #   [-406, 116, 374, -180, 0, 90]
        self.robotMoveWaitComplete()

    def robotJointPosition(self):
        '''获取机械臂当前关节角度，返回值为[J1,J2,J3,J4,J5,J6]'''

        self.robotCheckConnected()
        stateJson=json.loads(self.robotAction('{"cmdName":"get_data"}'))
        return stateJson["joint_actual_position"]

    def robotEndPosition(self):
        '''获取机械臂当前末端位置，返回值为[x,y,z,a,b,c]'''

        self.robotCheckConnected()
        stateJson=json.loads(self.robotAction('{"cmdName":"get_data"}'))
        return stateJson["actual_position"]

    def robotAction(self,commandString):
        '''发送指令至机械臂，并接收返回消息'''

        self.__robotSocket.sendall(commandString.encode("ascii"))
        time.sleep(0.1)
        recvdata = self.__robotSocket.recv(2048).decode("ascii")
        recvString = recvdata

        if len(recvString) > 160:
            recvString = recvString[0:160] + u"...... 【数据过长，后续省略】"
        print("【机械臂】 通讯返回信息：", recvString)
        time.sleep(0.01)
        return recvdata

    def robotActionWithoutResponse(self, commandString):
        '''已废弃！！！发送指令至机械臂，不接收返回消息'''

        self.__robotSocket.sendall(commandString.encode("ascii"))
        time.sleep(0.01)


if __name__ == '__main__':

    visionRobot = Robot('192.168.3.150',10001)

    if len(sys.argv)<2:
        print("请在运行该程序时添加相应参数！ 程序已结束！")
    else:
        if sys.argv[1]=="turnon":
            visionRobot.robotPowerOn()

        elif sys.argv[1]=="shutdown":
            visionRobot.robotPowerOff()

        elif sys.argv[1]=="pos":
            print(visionRobot.robotEndPosition())

        else:
            pass

# EOF