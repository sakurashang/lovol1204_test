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


class SteerWheel:
    # 实例化节点的时候默认开启
    def __init__(self, Car_Switch):
        """
        :param Car_Switch: Car_Switch=1 #默认为1，开启小车控制功能
        """
        self.Car_Switch = Car_Switch
        #sendMsg用的task
        self.task = self.__init_sendMsg(1,250,0,64,0,0)

    def __msgInit(self,Id200_Motor_Enable,Id200_Motor_Speed,Id200_Steer_Angle,Id200_Maximum_Torque_Set,Id200_Parameter_Set,Id200_Parameter_Value):
        """
            初始化消息处理：处理dbc文件
        :return:返回message，给send_period、modify作为其参数
        """
        id_200_message = db.get_message_by_name('Message_id_200')

        id_200Data = {}
        for i in id_200_message.signal_tree:
            id_200Data[i] = 0
        id_200Data["Id200_Motor_Enable"] = Id200_Motor_Enable  ##挡位，D前：1; N空：2; R倒：3
        id_200Data["Id200_Motor_Speed"] = Id200_Motor_Speed
        id_200Data["Id200_Steer_Angle"] = Id200_Steer_Angle
        id_200Data["Id200_Maximum_Torque_Set"] = Id200_Maximum_Torque_Set
        id_200Data["Id200_Parameter_Set"] = Id200_Parameter_Set
        id_200Data["Id200_Parameter_Value"] = Id200_Parameter_Value



        data = id_200_message.encode(id_200Data)
        # 得到发送的帧id
        print("=================================")
        print(id_200_message.frame_id)
        print(id_200_message.senders)
        # id_200_message.frame_id=0x51


        message = can.Message(arbitration_id=id_200_message.frame_id, data=data, is_extended_id=False)

        return message


    def __init_sendMsg(self,Id200_Motor_Enable,Id200_Motor_Speed,Id200_Steer_Angle,Id200_Maximum_Torque_Set,Id200_Parameter_Set,Id200_Parameter_Value):
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
            message = self.__msgInit(Id200_Motor_Enable,Id200_Motor_Speed,Id200_Steer_Angle,Id200_Maximum_Torque_Set,Id200_Parameter_Set,Id200_Parameter_Value)

            print('发送的信息：', message)
            task = can_bus.send_periodic(message, period=0.02, duration=None, store_task=True)
            print('已开启sendperiod线程')
            # 假设task线程在本函数结束以后不停止
            return task
    def sendMsg(self,Id200_Motor_Enable,Id200_Motor_Speed,Id200_Steer_Angle,Id200_Maximum_Torque_Set,Id200_Parameter_Set,Id200_Parameter_Value):
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

        message = self.__msgInit(Id200_Motor_Enable,Id200_Motor_Speed,Id200_Steer_Angle,Id200_Maximum_Torque_Set,Id200_Parameter_Set,Id200_Parameter_Value)
        self.task.modify_data(message)
        pass

if __name__ == "__main__":

    steerWheel = SteerWheel(1)
    #档位前进 转速1000 对应帧 55 01 00 00 40 1f 00 00
    # 55是给四个数据使能  01 是前进档 第五六个字节控制转速，物理数据1000r,报文数据得扩大8倍，所以报文数值大小是8000
    #转换为16进制，即是1f40 ,由于字节顺序是从底到高，所以第五个字节是40，第六个字节是1f
    while 1:
        steerWheel.sendMsg(1,20,2,64,0,0)
        # time.sleep(0.3)
        # steerWheel.sendMsg(1, 0, 0, 64, 0, 0)
        # time.sleep(0.3)
        # steerWheel.sendMsg(1, 20, 2, 64, 0, 0)
        # time.sleep(0.3)
        # kp = 0
        # ki = 0
        # kd = 0
        # pid = classOfPID.PID(P=kp, I=ki, D=kd)
        # pid.setValue = 15  # set end  zz设定值
        # curValue = 10
        # outPID = pid.pidPosition(curValue)
        #
        # #控制量有两个，一个速度，一个方向；速度暂时可以不管，获得他的
        # steerWheel.sendMsg(1, 0, 2, 64, 0, 0)