from simple_pid import PID
import time

from steer import Car

import serial
from pymodbus.client.sync import ModbusSerialClient

# 配置串口参数
ser = serial.Serial(
    port='/dev/ttyUSB0',  # 串口设备
    baudrate=9600,         # 波特率
    bytesize=8,            # 数据位
    parity='N',            # 奇偶校验
    stopbits=1             # 停止位
)

# 创建MODBUS RTU客户端
client = ModbusSerialClient(method='rtu', port='/dev/ttyUSB0', baudrate=9600,stopbits=1)


# 初始化PID控制器
kp = 0.9
ki = 0.1
kd = 0.00
pid = PID(kp, ki, kd, setpoint=0)

address = 0x0100  # 输入寄存器的起始地址
count = 2  # 要读取的寄存器数量

# 模拟前轮抓角传感器，初始转角为0

steerWheel = Car(1)
# 模拟汽车前轮转向控制
# 记录开始时间
start_time = time.time()
while True:

    response = client.read_input_registers(address, count, unit=1)




    # if response.isError():
    #     print("读取输入寄存器时发生错误")
    # else:
    #     t=response.registers[0]
    #     i=t/65535*5
    #     if i>2.04226 and i<2.0438:
    #         steer_angle=0
    #     elif i <2.04226:
    #         steer_angle= (2.04226-i)/(2.04226-1.314)*28
    #     else :
    #         steer_angle= (2.0438-i)/(2.411-2.0438)*28
    #     print("成功读取输入寄存器的值：",t,"电压值:  ",i,"角度值：  ",steer_angle)
    if response.isError():
        print("读取输入寄存器时发生错误")
    else:
        t=response.registers[0]
        i=t/65535*5
        if i>2.051 and i<2.063:
            steer_angle=0
        elif i <2.051:
            steer_angle= (2.051-i)/(2.051-1.346)*28
        else :
            steer_angle= (2.063-i)/(2.4476-2.051)*28
        print("成功读取输入寄存器的值：",t,"电压值:  ",i,"角度值：  ",steer_angle)


    # 获取实时前轮转角数据，你需要替换这里的代码来获取传感器数据
    # 这里使用current_wheel_angle来代替传感器数据
    current_wheel_angle = steer_angle  # 假设前轮每次增加1度，实际情况根据传感器来获取
    pid.setpoint = -20
    # 在这里，你可以根据需要动态修改setpoint
    # 例如，每隔一段时间将setpoint更改为不同的目标角度
    # if -22 <=current_wheel_angle <= -20:
    #     pid.setpoint = -10  # 设置新的目标角度
    # elif -12 <= current_wheel_angle <= -10:
    #     pid.setpoint = 0  # 设置另一个目标角度
    # elif -2 <= current_wheel_angle <= 0:
    #     pid.setpoint = 10  # 设置另一个目标角度
    # elif 10 <= current_wheel_angle <= 12:
    #     pid.setpoint = 20  # 设置另一个目标角度
    # 方向盘的目标度数在-28～28

    # 计算PID控制器的输出
    # control_output输出为正值的时候，向左；输出为负值的时候，向右。
    error = current_wheel_angle - pid.setpoint
    control_output = pid(error)

    # 映射PID输出到方向盘扭矩（转速）和旋转方向
    steering_speed = abs(control_output)
    if control_output > 0:
        rotation_direction = 1
        steerWheel.sendMsg(steering_speed,rotation_direction,1,64)
    elif control_output < 0:
        rotation_direction = 2
        steerWheel.sendMsg(steering_speed,rotation_direction,1,64)
    if -0.5 <= error <= 0.5 :
        steerWheel.sendMsg(0, rotation_direction, 0, 60)
        # 记录结束时间
        end_time = time.time()

        # 计算执行时间
        execution_time = end_time - start_time
        print(f"结束时间:{execution_time}")
        break


    # 执行控制动作，控制方向盘转向
    # 这里你需要替换为真实的汽车控制代码
    print(f"当前前轮转角: {current_wheel_angle} 度，PID输出: {control_output}")
    print(f"方向盘扭矩: {steering_speed}，旋转方向: {rotation_direction}")
    # 模拟时间间隔，实际情况下可以根据需要调整
    time.sleep(0.1)