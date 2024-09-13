# coding=utf-8
# import serial

# # 配置串口参数
# port = 'COM4'  # 替换为你的串口号
# baudrate = 115200  # 波特率
# timeout = 1  # 读取超时设置（秒）

# # 创建串口对象
# ser = serial.Serial(port, baudrate, timeout=timeout)

# def parse_data(data):
#     """
#     解析从串口读取的数据
#     """
#     # 示例解析：假设数据是以逗号分隔的字符串
#     parsed_data = data.strip().split(',')
#     return parsed_data

# try:
#     while True:
#         if ser.in_waiting > 0:
#             # 从串口读取数据
#             raw_data = ser.readline().decode('utf-8').strip()
#             print(f"Raw Data: {raw_data}")

#             # 解析数据
#             parsed_data = parse_data(raw_data)
#             print(f"Parsed Data: {parsed_data}")

# except KeyboardInterrupt:
#     print("程序被用户中断")

# finally:
#     ser.close()
#     print("串口已关闭")

# import sys
# sys.path.append()

# 低字节在前，高字节在后
import serial
import struct
import numpy as np

def int16_to_float16(value):
    int16_value = np.int16(value)
        # 将 int16 转换为 16 位浮点数表示
    return np.float16(int16_value).astype(np.float32)

def float16_to_int16(value):
    float16_value = np.float16(value)
        # 将 float16 转换为 int16 表示
    return np.int16(float16_value)

def parsedata(data):
    data_sum = 0
    data_xor = 0
    # print("数据长度：" + str(len(data)))
    if len(data)==27:
        # 计算校验和与异或和
        data_slice=data[:25]
        for byte in data_slice:
            data_sum += byte
            data_xor ^= byte
        data_sum = data_sum&0xFF
        data_xor = data_xor&0xFF
        # print(data_xor,data_sum)

        if data_sum==data[26] and data_xor == data[25] and 238==data[0] and 144 == data[1]: 
            # 为了补齐字节 这里多增加了一个00
            insert_byte = b'\x00'
            insert_position = 3 #这里增加一个00 不然第3位都会被忽略

            part1 = data[:insert_position]
            part2 = data[insert_position:]
            data = part1+insert_byte+part2
           
            format_string = "3B8h8B"#B是字节 H是uint16的格式 h是int16的格式 2个字节拼
            unpacked_data = struct.unpack(format_string,data)

            sync1 = unpacked_data[0]
            sync2 = unpacked_data[1]
            fk_mode = unpacked_data[2]
            int16_values = unpacked_data[3:11]
            floats = [int16_to_float16(value) for value in int16_values]
            sum = unpacked_data[18]
            xor = unpacked_data[17]

            return{
                'sync1': sync1,
                'sync2': sync2,
                'FK_mode': fk_mode,
                'FK_yaw': floats[0]*0.01,
                'FK_pitch': floats[1]*0.01,
                'FK_roll': floats[2]*0.01,
                'Frame_Angle_Roll': floats[3]*0.01,
                'Frame_Angle_Pitch': floats[4]*0.01,
                'Frame_Angle_Azimuth': floats[5]*0.01,
                'OffTarget_x': floats[6]*0.05,
                'OffTarget_y': floats[7]*0.05,
                'xor':xor,
                'sum':sum
            }
        else:
            print('校验不通过') 


       
            # for byte in data:
            #     print(f"{byte:d}", end=' ')
            #     print()  # 换行
    else:
         print("字节数不够\n")
         

def pack_data(data):
    # 定义帧头
    FRAME_HEADER = b'\xEB\x16'
    data = float16_to_int16(100 * data) #放大100倍
    data = tuple(data)
    # 将 int16 值打包成二进制数据
    packed_data = struct.pack('<hhh', *data) 
    packed_data = packed_data
    packed_data = FRAME_HEADER + packed_data  
    zero_byte =  b'\x00\x00\x00\x00'
    packed_data = packed_data + zero_byte 
    # 计算校验和和异或校验
    data_sum = 0
    data_xor = 0
        # 计算校验和与异或和
    for byte in packed_data:
        # print(byte)
        data_sum += byte
        data_xor ^= byte
    data_sum = data_sum&0xFF
    data_xor = data_xor&0xFF
    # print(data_xor,data_sum)
    
    # 打包校验和到数据中
    sum_bytes = struct.pack('H', data_sum)
    sum_bytes = sum_bytes[0].to_bytes(1, byteorder='big')
    xor_bytes = struct.pack('H', data_xor)  
    xor_bytes = xor_bytes[0].to_bytes(1, byteorder='big')

    
    # 拼接数据、校验和、异或校验，作为帧尾
    final_packet = packed_data + xor_bytes + sum_bytes
    # final_packet = FRAME_HEADER
    return final_packet


sendDataTest = b'AAAAAA'
#获取串口所读取的数据
class getdata:
    def __init__(self,name,baudrate):
        self._initialized = False
        self.serial = name
        self.baudrate = baudrate
        self.ser = False

    def initialize(self):       # 初始化串口
        self.ser = serial.Serial(self.serial, self.baudrate, timeout=0.01)
        if self.ser:
             self._initialized = True

    def run(self):      # 接收并解析
        try:
            if self._initialized:
                data = self.ser.read(27)  # 读取一行数据
                # global sendDataTest
                # a = self.ser.write(sendDataTest)   
                # # print(sendDataTest)
                # print(a)
                # data = self.ser.write(27)       # 27个字节
                # for byte in data:
                #     print(f"{byte:d}", end=' ')
                # print()  # 换行

                par_data = parsedata(data)
                if par_data!=None:
                    return par_data
                else:
                    print('未收到数据')
                
                
        except KeyboardInterrupt:
            print("程序被中断")
         
    def send(self, sendData):     #发送数据
        # with serial.Serial(self.serial, self.baudrate, timeout=0.01) as ser:
        # # 发送数据
        #     if isinstance(packData, str):
        #         packData = packData.encode()  # 如果数据是字符串，转换为字节
        #     ser.write(packData)
        #     # print(ser.write(packData))
        #     print(f"发送数据: {packData}")
        try:
            if self._initialized:
                packData = pack_data(sendData)
                if isinstance(packData, str):
                    packData = packData
                # packData = b'eb169a02de006f00000000000bcf' 
                printRes = ' '.join(f'{byte:02x}' for byte in packData)
                # print(printRes)
                a = self.ser.write(packData)
                # print(f"发送数据的长度：{a}")
        except KeyboardInterrupt:
            print("程序被中断")
        except serial.SerialException as e:
            print(f"串口发送错误: {e}")
