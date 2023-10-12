# 学习csv文件的读写
# 用一个类来实现，在循迹程序开始前打开文件，在循迹程序结束时关闭文件
import csv


class DataRecord:
    def __init__(self, csv_name):
        self.csvfile = open(csv_name, 'w')
        self.writer = csv.writer(self.csvfile, delimiter=',')
        self.writer.writerow(['GPSWeek', 'GPSTime', 'imu_lon', 'imu_lat', 'headingAngle', 'x', 'y', 'v',
                              'cmd_steering', 'cmd_v', 'imu_gps_state', 'imu_altitude', 'imu_satellite_num', 'imu_warning'])

    def __del__(self):
        self.csvfile.close()

    def data_record(self, msg):
        """
            需要记录的数据字段：经度，纬度，航向角，x,y,时间，小车发送的指令（转角，速度）...
            GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, cmd_steering, cmd_v
        :return:
        """
        # msg=[GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, cmd_steering, cmd_v]
        # 测试是否可以写数值型
        #self.writer.writerow([GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, cmd_steering, cmd_v])
        self.writer.writerow(msg)


if __name__ == "__main__":
    record = DataRecord()
    # 10个参数存入
    a=[]
    for i in range(10):
        print(i)
        a.append(i)

    for j in range(10):
        record.data_record(a)
    del record

