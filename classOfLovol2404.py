# dbc文件解析类
# 进行dbc的解析
# 四个消息字段：发送消息message('NAVI', 0x18ff911c, True, 8, None)、message('NAVI2', 0x18ff921c, True, 8, None)、
# message('NAVI3', 0x18ff931c, True, 8, None)，接收消息message('VC6', 0x18ffa127, True, 8, None)
# 要实现消息的解析和发送

import cantools
import os
import can
import time
import numpy as np
import threading


class CANMsgTrans(object):
    db = 0

    def __init__(self, dbc_name):
        self.dbc_name = dbc_name
        dbc_path = os.path.join(os.getcwd(), dbc_name)
        self.db = cantools.database.load_file(dbc_path)
        # print(self.db)

    def can_msg_produce(self, msg_name, msg_list):
        msg = self.db.get_message_by_name(msg_name)
        # 消息发送初始化
        msg_data = {}
        j = 0
        for i in msg.signal_tree:
            msg_data[i] = msg_list[j]
            j = j + 1
        print(msg_data)
        data = msg.encode(msg_data)
        # message = can.Message(arbitration_id=msg.frame_id, data=data, is_extended_id=False)
        msg_frame_id = msg.frame_id
        # if msg.frame_id == 0x21C:  #NAVI2
        #     msg_frame_id = 0x18FF921C
        # if msg.frame_id == 0x11C:   #NAVI
        #     msg_frame_id = 0x18FF911C
        # if msg.frame_id == 0x31C:   #NAVI3
        #     msg_frame_id = 0x18FF931C
        message = can.Message(arbitration_id=msg_frame_id, data=data, is_extended_id=True)
        return message


class Tractor(object):
    # 导航使能开关 0:自动驾驶使能关闭、1：开启、2：信号错误、3不使用
    # 前轮转向角命令
    NCU_1_msg_list = [0,0]
    #液压输出阀1/2/3/4动作控制  0：空档、4浮动、0x10：正向动作、0x40:反向动作
    NCU_2_msg_list = [0, 0, 0, 0]
    # 整机使能开关 0：关闭、1：开启、2信号错误、3：不使用 ；PTO使能开关；液压输出阀1使能开关、液压2、液压3、液压4；后PTO开关；远程差速锁指令；远程四驱指令
    # 远程方向档位控制指令：0：空档、1：后退档、2：前进档；远程目标车速指令：0-40km/h
    NCU_3_msg_list = [0, 0, 0, 0, 0, 0, 0,0,0,0, 0]
    # 摇臂开关（控制提升器动作）：0：停止、1：上升、2：下降、0x0E:fault、0x0f:not available
    # 外部使能开关：0：手动控制、1：导航控制、2：不使用、3：报错
    NCU_EHC_msg_list = [0, 0]
    # __can_msg_trans = CANMsgTrans("/home/wen/PycharmProjects/dongFengProj_1.0/component_0/dongFeng2204_2.dbc")
    __can_msg_trans = CANMsgTrans("lovol2404.dbc")

    # __msg = can_msg_trans.can_msg_produce("NAVI", self.__NAVI_msg_list)

    def __init__(self, flag):
        # 新增权限赋予
        # print(os.system("./component_0/modprobe_peak_usb_forDongFeng2204"))  # 这样会不会使得成为单例模式？
        # print(os.system("./component_0/modprobe_vcan"))    # 测试时候使用

        self.can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=250000)
        if flag == "send":  # 当前是发送消息的功能
            self.task_NCU_1 = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NCU_1", self.NCU_1_msg_list), period=0.1, duration=None,
                store_task=True)  # 一个task值对应一个can.Message.arbitration_id
            # task.modify_data(msg)

            self.task_NCU_2 = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NCU_2", self.NCU_2_msg_list), period=0.1,
                duration=None, store_task=True)
            print('debug NCU_2')
            self.task_NCU_3 = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NCU_3", self.NCU_3_msg_list), period=0.1,
                duration=None, store_task=True)
            self.task_NCU_EHC = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NCU_EHC", self.NCU_EHC_msg_list), period=0.1,
                duration=None, store_task=True)
        else:  # 当前是接受消息的功能
            pass

    def send_msg(self, msg_name, cmd_list):
        # 把can_msg的生成放进来
        can_msg = self.__can_msg_trans.can_msg_produce(msg_name, cmd_list)
        if msg_name == "NAVI":
            self.task_NCU_1.modify_data(can_msg)
        if msg_name == "NAVI2":
            self.task_NCU_2.modify_data(can_msg)
        if msg_name == "NAVI3":
            self.task_NCU_3.modify_data(can_msg)
        if msg_name == "NCU_EHC":
             self.task_NCU_EHC.modify_data(can_msg)

    def recv_msg_full_dbc_id(self):
        # 等待接收，直到收到信息才会返回。返回dbc里的两个反馈，其他的不反馈。
        get_data = self.can_bus.recv()
        while 1:
            if get_data.arbitration_id == 0x18FFC203:  # vcan会丢弃前面的，完整的是0x18FFA127,这个是扩展帧的原因
                # print("收到一条信息")
                # motion_info = self.__can_msg_trans.db.decode_message(get_data.arbitration_id, get_data.data)
                vehicle_status = self.__can_msg_trans.db.decode_message(0x18FFC203,
                                                                     get_data.data)  # eg:{'gearnum': 14, 'IO_vVeh': 51.1328125, 'Rx_valuePositionSensorFrmEHR': 68, 'Rx_valueSumofDraftFrmEHR': 85, 'Rx_staErr1': 0, 'Rx_staErr2': 1, 'Rx_rawwheelangle': 19}
                # 增加明文信息
                vehicle_status_dict = {"方向档位": vehicle_status["Gear_Status"], "驱动模式": vehicle_status["Drive_Mode_Status"],
                               "四轮驱动状态": vehicle_status["Four_Wheel_Drive_Status"],
                               "差速锁状态": vehicle_status["Differential_Lock_Status"],
                               "高低档状态": vehicle_status["HighandLow_Gear_Status"], "车速": vehicle_status["Speed_Status"]}
                # print(motion_dict)
                return [0x18FFC203, vehicle_status_dict]
            elif get_data.arbitration_id == 0x18FFC403:
                PTO_status = self.__can_msg_trans.db.decode_message(0x18FFC403,
                                                                       get_data.data)  # eg:{'Rx_staBreak': 17, 'Rx_trqEngActFrmEMS': -91, 'Rx_staWheelmotorError': 51, 'Rx_rawSteerlPos': 21828}
                PTO_status_dict = {"PTO离合器接合状态": PTO_status["PTO_Clutch_Status"], "PTO档位状态": PTO_status["PTO_Gear_Status"],
                                 "PTO输出轴转速": PTO_status["PTO_Output_Shaft_Speed"]}
                # print(security_dict)
                return [0x18FFC403, PTO_status_dict]
            elif get_data.arbitration_id == 0x18FFDAF0:
                steer_status = self.__can_msg_trans.db.decode_message(0x18FFDAF0,
                                                                    get_data.data)  # eg:{'Rx_staBreak': 17, 'Rx_trqEngActFrmEMS': -91, 'Rx_staWheelmotorError': 51, 'Rx_rawSteerlPos': 21828}
                steer_status_dict = {"前轮转角": steer_status["Front_Steer_Angle"]}
                # print(security_dict)
                return [0x18FFC403, steer_status_dict]
            else:
                # print("收到其他arbitration_id：", get_data.arbitration_id)
                get_data = self.can_bus.recv()
                # return [get_data.arbitration_id, get_data]


class VCUCmd(object):
    """
        本类进行拖拉机整车（横纵向运动、提升器、PTO、液压输出）控制指令的封装，按特定功能来设计函数。
    """
    # 类成员变量
    # 导航使能开关 0:自动驾驶使能关闭、1：开启、2：信号错误、3不使用
    # 前轮转向角命令
    NCU_1_msg_list = [1,0]
    #液压输出阀1/2/3/4动作控制  0：空档、4浮动、0x10：正向动作、0x40:反向动作
    NCU_2_msg_list = [0, 0, 0, 0]
    # 整机使能开关 0：关闭、1：开启、2信号错误、3：不使用 ；PTO使能开关；液压输出阀1使能开关、液压2、液压3、液压4；后PTO开关；远程差速锁指令；远程四驱指令
    # 远程方向档位控制指令：0：空档、1：后退档、2：前进档；远程目标车速指令：0-40km/h
    NCU_3_msg_list = [1, 1, 1, 1, 1, 1, 1,1,1,0, 0]
    # 摇臂开关（控制提升器动作）：0：停止、1：上升、2：下降、0x0E:fault、0x0f:not available
    # 外部使能开关：0：手动控制、1：导航控制、2：不使用、3：报错
    NCU_EHC_msg_list = [0, 1]

    # 按实现不同功能设计类方法
    def __init__(self, tractor):
        """
            进行变量的初始化
        """
        self.tractor = tractor

    def send_motion_ctrl_msg(self, Drive_Mode_Req, ShiftLevel_Req, Steering_Wheel_Angle, Target_Speed, Brk_En):
        """
            发送车辆横纵向控制指令。后期把参数换成英文。
        :param Drive_Mode_Req:0:手动模式、1：扭矩受导航请求控制、2：自动模式
        :param ShiftLevel_Req:字符串类型，分为“前进高档”、“前进低档”、“后退高档”、“后退低档”、“空挡”
        :param Steering_Wheel_Angle:
        :param Target_Speed:
        :param Brk_En:
        :return:
        """
        # 首先检查驾驶模式
        if Drive_Mode_Req == 0:  # 手动模式
            self.NCU_1_msg_list[0] = 0
            self.NCU_EHC_msg_list[1] = 0
            self.tractor.send_msg("NCU_1", self.NCU_1_msg_list)
            self.tractor.send_msg("NCU_EHC", self.NCU_EHC_msg_list)
            # return "手动模式" # debug20211001注释掉本句，想在手动驾驶的时候进行方向盘的控制

        # TODO:检查下有车速情况下发空挡是什么反应
        # 刹车的优先级最高放在这里，刹车后就返回(刹车转换成速度为0、不管转角)
        if Brk_En == 1:
            self.NCU_3_msg_list[9] = 0  # 挂空挡
            self.NCU_3_msg_list[10] = 0  # 修改车速(后期看是否要挂空挡)
            # 发送指令直接返回
            self.tractor.send_msg("NCU_3", self.NCU_3_msg_list)
            return "程序刹车制动"

        # 修改变量序列
        self.NCU_3_msg_list[10] = Target_Speed  # 修改车速

        if ShiftLevel_Req == "前进档":  # 测试下是否高效
            self.NCU_3_msg_list[9] = 2  # 修改档位(前进2、空挡0、后退1)
        elif ShiftLevel_Req == "后退档":
            self.NCU_3_msg_list[9] = 1
        else:
            # “空挡”
            self.NCU_3_msg_list[9] = 0

        # self.__NAVI_cmd_list[2]   # 修改提升器指令
        # 修改导航模式(0:手动模式、1：扭矩受导航请求控制、2：自动模式)
        self.NCU_1_msg_list[0] = Drive_Mode_Req
        self.NCU_EHC_msg_list[1] = Drive_Mode_Req
        # self.__NAVI_cmd_list[4]   # 修改点火熄火信号
        # self.__NAVI_cmd_list[5]   # 修改PTO指令

        # 前轮转角(后期读取前轮实际转角进行误差消除)
        self.NCU_1_msg_list[1] = Steering_Wheel_Angle  # 前轮目标转角

        # 发送修改的变量
        print('self.NCU_1_msg_list', self.NCU_1_msg_list)
        self.tractor.send_msg("NCU_1", self.NCU_1_msg_list)

        self.tractor.send_msg("NCU_2", self.NCU_2_msg_list)
        self.tractor.send_msg("NCU_3", self.NCU_3_msg_list)
        return "程序控制指令发送成功"

    def send_hoist_msg(self, hoist_cmd="rising"):  # debug 修改最高高度250 到220
        """
            控制提升器的高度、升降等。参数列表：提升器指令、耕地深度设定旋钮、限高旋钮、下降速度设定、模式设定。 (反馈值：提升器位置（高度）、提升器合力)
        :param hoist_cmd:"rising"、“falling”、“no_action”
        :param hoist_ploughing_depth_set:
        :param hoist_height_limit_set:
        :param hoist_drop_speed_set:
        :param hoist_model_set:
        :return:
        """
        # 提升器指令
        if hoist_cmd == "rising":
            self.NCU_EHC_msg_list[0] = 1
        elif hoist_cmd == "falling":
            self.NCU_EHC_msg_list[0]  = 2
        else:
            self.NCU_EHC_msg_list[0]  = 0  # 无动作



        # 发送命令
        self.tractor.send_msg("NCU_EH", self.NCU_EHC_msg_list)


    # def steering_wheel_fault_removal(self):
    #     """发送方向盘故障清除指令，这里应该有一个持续时间的问题，如果测试不行则多发几次"""
    #     # 改好发送再改过来
    #     self.__NAVI3_cmd_list[5] = 1
    #     self.tractor.send_msg("NAVI3", self.__NAVI3_cmd_list)
    #     self.__NAVI3_cmd_list[5] = 0

    def send_pto_msg(self, pto_connected=0):
        """控制PTO"""
        self.__NAVI_cmd_list[5] = pto_connected  # 修改PTO指令
        self.tractor.send_msg("NAVI", self.__NAVI_cmd_list)

    def send_hydraulic_msg(self):
        """控制液压输出"""


class SafetyGuarantee(object):
    """提供安全保障功能"""

    # 读取刹车状态、方向盘故障信息，然后切换成手动驾驶。使用多线程的方式实现。后期完成
    def __init__(self, tractor_recv):
        self.__tractor_recv = tractor_recv
        # self.brake_flag = brake_flag     # 默认为0不刹车
        global brake_flag

    def monitor_brake(self):
        # 暂时先这样
        while 1:
            recv = self.__tractor_recv.recv_msg_full_dbc_id()
            if recv[0] == 0x18FFA227 and recv[1]["刹车状态"] == 1:
                # 有刹车
                print("在刹车")
                brake_flag = 1
                # 执行速度为0、抬机具，δ时间后切换成空挡，判断切换成空挡后换成手动模式
                return brake_flag


if __name__ == "__main__":
    # 导航使能开关 0:自动驾驶使能关闭、1：开启、2：信号错误、3不使用
    # 前轮转向角命令
    NCU_1_msg_list = [1,0]
    #液压输出阀1/2/3/4动作控制  0：空档、4浮动、0x10：正向动作、0x40:反向动作
    NCU_2_msg_list = [0, 0, 0, 0]
    # 整机使能开关 0：关闭、1：开启、2信号错误、3：不使用 ；PTO使能开关；液压输出阀1使能开关、液压2、液压3、液压4；后PTO开关；远程差速锁指令；远程四驱指令
    # 远程方向档位控制指令：0：空档、1：后退档、2：前进档；远程目标车速指令：0-40km/h
    NCU_3_msg_list = [1, 1, 1, 1, 1, 1, 1,1,1,0, 0]
    # 摇臂开关（控制提升器动作）：0：停止、1：上升、2：下降、0x0E:fault、0x0f:not available
    # 外部使能开关：0：手动控制、1：导航控制、2：不使用、3：报错
    NCU_EHC_msg_list = [0, 1]

    # 20210705 更新:测试横向跟踪性能
    tractor = Tractor("send")
    vcu_cmd = VCUCmd(tractor)
    vcu_cmd.send_motion_ctrl_msg(1, "前进档", 0, 10, 0)
    # time.sleep(3)
    # tractor.send_msg("NAVI", NAVI_cmd_list)
    # NAVI3_cmd_list[4] = -40
    # tractor.send_msg("NAVI3", NAVI3_cmd_list)
    while True:
        time.sleep(1)
    # tractor.send_msg("NAVI3", NAVI3_cmd_list)
    # tractor = classOfDongFengSF2204.Tractor("send")
    # vcu_cmd = VCUCmd(tractor)

    # machine_operation_enabled=1
    # machine_operation_status="raise"
    # #machine_operation_status = "down"
    #
    # # 添加机具控制，及其使能
    # if machine_operation_enabled == 1:
    #     if machine_operation_status == "raise":
    #         # 机具提升命令发送
    #         # pto断开
    #         vcu_cmd.send_pto_msg(0)
    #         time.sleep(0.1)  # debug
    #         vcu_cmd.send_hoist_msg("rising", hoist_ploughing_depth_set=1000, hoist_height_limit_set=220)
    #         pass
    #     elif machine_operation_status == "down":
    #         # 机具下降命令发送
    #         vcu_cmd.send_hoist_msg("falling", hoist_ploughing_depth_set=760, hoist_height_limit_set=220)
    #         time.sleep(0.1)  # debug
    #         #vcu_cmd.send_pto_msg(1)
    #         pass

    # 单独测试转角
    # NAVI3_cmd_list[4] = 40
    # NAVI3_cmd_list[4] = -40
    # tractor.send_msg("NAVI3", NAVI3_cmd_list)
    # while True:
    #     time.sleep(1)


