"""进行云乐小车的类型声明"""
import time

# 小车要控制它，需要的信息如下：小车的属性和对它的操作，属性分别是五个，操作分别是发送和接收信息,发送和接收信息应该是接口
#   SCU_Drive_Mode_Req,SCU_ShiftLevel_Req,SCU_Steering_Wheel_Angle,SCU_Target_Speed,SCU_Brk_En五个参数在发送的时候需要

import cantools
import os

import can



# 全局变量，dbc文件解析和can0通信初始化
dbcPath = os.path.join(os.getcwd(), "lovol1204.dbc")
db = cantools.db.load_file(dbcPath)

can_bus = can.interface.Bus('can1', bustype='socketcan', bitrate=250000)



class Car:
    # 实例化节点的时候默认开启
    def __init__(self, Car_Switch):
        """
        :param Car_Switch: Car_Switch=1 #默认为1，开启小车控制功能
        """
        self.Car_Switch = Car_Switch
        #sendMsg用的task
        self.task = self.__init_sendMsg(1,1,1,1,1,0,0)

    def __msgInit(self,Shift_Control_Enable,PTO_Control_Enable,Brake_Control_Enable,Engine_Control_Enable,Shift_Control_Order,PTO_Control_Order,RPM_Order):
        """
            初始化消息处理：处理dbc文件
        :return:返回message，给send_period、modify作为其参数
        """
        nsc_message = db.get_message_by_name('NSC')

        nscData = {}
        for i in nsc_message.signal_tree:
            nscData[i] = 0
        nscData["Shift_Control_Enable"] = Shift_Control_Enable  ##挡位，D前：1; N空：2; R倒：3
        nscData["PTO_Control_Enable"] = PTO_Control_Enable
        nscData["Brake_Control_Enable"] = Brake_Control_Enable
        nscData["Engine_Control_Enable"] = Engine_Control_Enable
        nscData["Shift_Control_Order"] = Shift_Control_Order
        nscData["PTO_Control_Order"] = PTO_Control_Order
        nscData["RPM_Order"] = RPM_Order


        data = nsc_message.encode(nscData)
        # 得到发送的帧id
        print("=================================")
        print(nsc_message.frame_id)
        print(nsc_message.senders)
        # nsc_message.frame_id=0x51


        message = can.Message(arbitration_id=nsc_message.frame_id, data=data, is_extended_id=True)

        return message


    def __init_sendMsg(self,Shift_Control_Enable,PTO_Control_Enable,Brake_Control_Enable,Engine_Control_Enable,Shift_Control_Order,PTO_Control_Order,RPM_Order):
        """
            (调用modify函数实现)小车会以100ms周期一直发送该指令
        :param Shift_Control_Enable: 换挡控制使能
        :param PTO_Control_Enable:PTO控制使能
        :param Brake_Control_Enable:刹车控制使能
        :param Engine_Control_Enable:发动机控制使能
        :param Shift_Control_Order:换挡命令
        :param PTO_Control_Order:PTO命令
        :param RPM_Order：转速命令
        :return:返回task，方便调用后续调用modify
        """
        if self.Car_Switch == 0:
            print('当前节点为信息读取节点')
        else:

            # 转换成CAN帧，发送
            message = self.__msgInit(Shift_Control_Enable,PTO_Control_Enable,Brake_Control_Enable,Engine_Control_Enable,Shift_Control_Order,PTO_Control_Order,RPM_Order)

            print('发送的信息：', message)
            task = can_bus.send_periodic(message, period=0.02, duration=None, store_task=True)
            print('已开启sendperiod线程')
            # 假设task线程在本函数结束以后不停止
            return task
    def sendMsg(self,Shift_Control_Enable,PTO_Control_Enable,Brake_Control_Enable,Engine_Control_Enable,Shift_Control_Order,PTO_Control_Order,RPM_Order):
        """
            发送车辆控制指令
        :param Shift_Control_Enable: 换挡控制使能
        :param PTO_Control_Enable:PTO控制使能
        :param Brake_Control_Enable:刹车控制使能
        :param Engine_Control_Enable:发动机控制使能
        :param Shift_Control_Order:换挡命令
        :param PTO_Control_Order:PTO命令
        :param RPM_Order：转速命令
        :return:返回task，方便调用后续调用modify
        """


        message = self.__msgInit(Shift_Control_Enable,PTO_Control_Enable,Brake_Control_Enable,Engine_Control_Enable,Shift_Control_Order,PTO_Control_Order,RPM_Order)
        print('发送的信息：', message)
        self.task.modify_data(message)
        pass

if __name__ == "__main__":

    car = Car(1)
    #档位前进 转速1000 对应帧 55 01 00 00 40 1f 00 00
    # 55是给四个数据使能  01 是前进档 第五六个字节控制转速，物理数据1000r,报文数据得扩大8倍，所以报文数值大小是8000
    #转换为16进制，即是1f40 ,由于字节顺序是从底到高，所以第五个字节是40，第六个字节是1f
    while 1 :
      car.sendMsg(1,1,1,1,1,0,1000)



