from simple_pid import PID
import time

from Steer_Wheel_Control import SteerWheel

# 初始化PID控制器
kp = 1.5
ki = 0.0 
kd = 0.01
pid = PID(kp, ki, kd, setpoint=0)

# 模拟前轮抓角传感器，初始转角为0
current_wheel_angle = 0
steerWheel = SteerWheel(1)
# 模拟汽车前轮转向控制
while True:
    # 获取实时前轮转角数据，你需要替换这里的代码来获取传感器数据
    # 这里使用current_wheel_angle来代替传感器数据
    #current_wheel_angle += control_output  # 假设前轮每次增加1度，实际情况根据传感器来获取

    # 在这里，你可以根据需要动态修改setpoint
    # 例如，每隔一段时间将setpoint更改为不同的目标角度
    # if current_wheel_angle == 10:
    #     pid.setpoint = 30  # 设置新的目标角度
    # elif current_wheel_angle == 20:
    #     pid.setpoint = 60  # 设置另一个目标角度
    # 方向盘的目标度数在-28～28
    pid.setpoint = 20
    # 计算PID控制器的输出
    # control_output输出为正值的时候，向左；输出为负值的时候，向右。
    control_output = pid(current_wheel_angle - pid.setpoint)

    # 映射PID输出到方向盘扭矩（转速）和旋转方向
    steering_speed = abs(control_output)
    if control_output > 0:
        rotation_direction = 1
        steerWheel.sendMsg(1,steering_speed,rotation_direction,64,0,0)
    else:
        rotation_direction = 0
        steerWheel.sendMsg(1,steering_speed,rotation_direction,64,0,0)

    # 执行控制动作，控制方向盘转向
    # 这里你需要替换为真实的汽车控制代码
    print(f"当前前轮转角: {current_wheel_angle} 度，PID输出: {control_output}")
    print(f"方向盘扭矩: {steering_speed}，旋转方向: {rotation_direction}")
    current_wheel_angle += control_output
    # 模拟时间间隔，实际情况下可以根据需要调整
    time.sleep(0.1)
