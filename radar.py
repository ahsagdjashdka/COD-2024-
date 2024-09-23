import serial
import struct
import time
from threading import Thread
from threading import Lock
from contextlib import redirect_stdout
import datetime
# 常量定义
REFEREE_DATALENGTH = 200
REFEREE_BufferNum = 200
CRC8_INIT = 0xff
CRC16_INIT = 0xffff
HP_DATA_ID = 0x0003  # 飞镖发射站状态
RADAR_MARK_ID = 0x020C       # 雷达标记进度
Dart_info = 0x0105
seq = 0
index = 1
# CRC8查找表
CRC8_table = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
]

# CRC16查找表
CRC16_table = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
]


class RefereeInfo:
    """定义用于存储裁判系统信息的类"""
    def __init__(self):
        self.dart_info = {}
        self.HP_info = {}
        self.radar_mark_data = {}
        self.count = {}
        

def append_CRC8_check_sum(message):
    """计算并追加CRC8校验和至消息末尾"""
    crc8 = CRC8_INIT
    for byte in message[:-1]:
        crc8 = CRC8_table[crc8 ^ byte]
    message[-1] = crc8

def append_CRC16_check_sum(message):
    """计算并追加CRC16校验和至消息末尾"""
    crc16 = CRC16_INIT
    for byte in message[:-2]:
        crc16 = (crc16 >> 8) ^ CRC16_table[(crc16 ^ byte) & 0xFF]
    message[-2] = crc16 & 0xFF
    message[-1] = crc16 >> 8

class Referee:
    def __init__(self):
        self.ser = serial.Serial('COM3', 115200, timeout=None)  # 根据实际情况调整串口号
        self.referee_info = RefereeInfo()
        self.lock = Lock()  # 添加一个线程锁

    def log_print(self, *args, **kwargs):
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        message = f"{timestamp} - {' '.join(map(str, args))}"
        with open('output.log', 'a') as file:
            with redirect_stdout(file):
                print(message, **kwargs)

    def read_data(self):
        """不断读取串口数据"""
        while True:
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                if len(data) >= 8:
                    self.process_data(data)

    def process_data(self, data):
        """处理接收到的数据"""
        index = 0
        while index < len(data):
            if data[index] == 0xA5:
                if index + 3 > len(data) - 1:
                    break
                data_length = (data[index + 2] << 8) | data[index + 1] + 9
                if index + data_length > len(data):
                    break
                if self.verify_crc(data[index:index+data_length], data_length):
                    self.referee_info_update(data[index:index+data_length], self.referee_info)
                index += data_length
            else:
                index += 1

    def verify_crc(self, message, length):
        """验证消息的CRC校验和"""
        return self.verify_crc8(message[:5]) and self.verify_crc16(message, length)

    def verify_crc8(self, message):
        """验证CRC8校验和"""
        crc8 = CRC8_INIT
        for b in message[:-1]:
            crc8 = CRC8_table[crc8 ^ b]
        return crc8 == message[-1]

    def verify_crc16(self, message, length):
        """验证CRC16校验和"""
        crc16 = CRC16_INIT
        for b in message[:-2]:
            crc16 = (crc16 >> 8) ^ CRC16_table[(crc16 ^ b) & 0xFF]
        return crc16 == (message[-2] | (message[-1] << 8))
    
    def bit8_to_bit16(self, byte1, byte2):
        """将两个8位字节转换为16位整数"""
        return byte2 << 8 | byte1
    
    def referee_info_update(self, data, referee):
        """更新裁判系统信息"""
        cmd_id = struct.unpack('<H', data[5:7])[0]  # 解析命令ID
        
        if cmd_id == RADAR_MARK_ID:
            self.hero_now = referee.radar_mark_data['mark_hero'] = data[7]  #7
            self.stand = referee.radar_mark_data['mark_stand'] = data[9]  #7
            self.sentry_now = referee.radar_mark_data['mark_sentry'] = data[12]
            if (self.hero_now - self.hero_last) <= 0 and self.hero_now != 120:
                self.hero_target = not self.hero_target
            else:
                self.hero_target = self.hero_target
            if self.sentry_now - self.sentry_last <= 0 and self.sentry_now != 120:
                self.sentry_target = not self.sentry_target
            else:
                self.sentry_target = self.sentry_target
            self.log_print(self.hero_now,self.hero_last)
            print(self.hero_now,self.hero_last)
            print(self.stand)
            self.hero_last = self.hero_now
            self.sentry_last = self.sentry_now

        elif cmd_id == Dart_info:
            referee.dart_info['flag'] = self.bit8_to_bit16(data[8],data[9])
            flag = referee.dart_info['flag']
            self.result = (flag & 0b1100000) >> 5
            self.log_print("flag:",self.result)
            print("flag:",self.result)

        elif cmd_id == 0x020E:
            referee.count['time'] = data[7] & 0b11
            self.conut = referee.count['time']
            self.log_print("time",referee.count['time'])
            print("time",referee.count['time'])


    def send_data(self):
        """定期发送数据"""
        self.seq = 0
        change_flag = 0
        self.hero_target = False
        self.sentry_target = False
        self.hero_last = 0
        self.sentry_last = 0
        self.sentry_now = 0
        self.hero_now = 0
        while True:
            hero_bule_id = 101
            hero_red_id = 1 
            hero_blue_index = [[16.4, 22.75] , [   1,    2]]
            #hero_blue_index = [[0, 22.75] , [   0,    2]]
            hero_red_index =  [[18.7 , 11] , [6.9, 13.5]]
            # hero_red_index =  [[0 , 11] , [0, 13.1]]
            id = hero_red_id
            x = hero_red_index[0][self.hero_target]
            y = hero_red_index[1][self.hero_target]
            header = bytearray(struct.pack('<BHB', 0xA5, 0x000A, self.seq)) + bytearray(1)  # 为CRC8额外添加一个字节
            append_CRC8_check_sum(header)  # 此时header包括了CRC8校验和
            data = struct.pack('<H', 0x0305) + struct.pack('<Hff', id, x, y)
            message = header + bytearray(data) + bytearray(2)  # 额外2个字节用于CRC16
            append_CRC16_check_sum(message)  # 添加CRC16校验和
            hex_representation = ' '.join(format(byte, '02X') for byte in message)
            # print("Message in hex:", hex_representation)
            # print(f"Sent message {seq}")
            with self.lock:  # 使用锁保护串口写入
                self.ser.write(message)  # 发送消息
            self.seq += 1  # 序列号递增
            if self.seq == 256:
                self.seq = 0
            time.sleep(0.1)  # 延时

            sentry_bule_id = 107
            sentry_red_id = 7
            sentry_blue_index = [[22.5, 22.5] , [8.7 , 6.3]]
            sentry_red_index =  [[5.7 , 5.7]  , [8.7 , 6.3]]
            id = sentry_red_id
            x = sentry_red_index[0][self.sentry_target]
            y = sentry_red_index[1][self.sentry_target]
            header = bytearray(struct.pack('<BHB', 0xA5, 0x000A, self.seq)) + bytearray(1)  # 为CRC8额外添加一个字节
            append_CRC8_check_sum(header)  # 此时header包括了CRC8校验和
            data = struct.pack('<H', 0x0305) + struct.pack('<Hff', id, x, y)
            message = header + bytearray(data) + bytearray(2)  # 额外2个字节用于CRC16
            append_CRC16_check_sum(message)  # 添加CRC16校验和
            hex_representation = ' '.join(format(byte, '02X') for byte in message)
            # print("Message in hex:", hex_representation)
            # print(f"Sent message {seq}")
            with self.lock:  # 使用锁保护串口写入
                if self.seq%5 == 0:
                    self.ser.write(message)  # 发送消息
            self.seq += 1  # 序列号递增
            if self.seq == 256:
                self.seq = 0
            time.sleep(0.1)  # 延时

            stand_bule_id = 103
            stand_red_id = 3
            stand_blue_index = [[22.5, 22.5] , [8.7 , 6.3]]
            stand_red_index =  [[0 , 11.6]  , [0 , 14.4]]
            id = stand_red_id
            x = stand_red_index[0][1]
            y = stand_red_index[1][1]
            header = bytearray(struct.pack('<BHB', 0xA5, 0x000A, self.seq)) + bytearray(1)  # 为CRC8额外添加一个字节
            append_CRC8_check_sum(header)  # 此时header包括了CRC8校验和
            data = struct.pack('<H', 0x0305) + struct.pack('<Hff', id, x, y)
            message = header + bytearray(data) + bytearray(2)  # 额外2个字节用于CRC16
            append_CRC16_check_sum(message)  # 添加CRC16校验和
            hex_representation = ' '.join(format(byte, '02X') for byte in message)
            # print("Message in hex:", hex_representation)
            # print(f"Sent message {seq}")
            with self.lock:  # 使用锁保护串口写入
                # if  self.seq%5 == 0:
                    self.ser.write(message)  # 发送消息
                    # print("Message in hex111:", hex_representation)
            self.seq += 1  # 序列号递增
            if self.seq == 256:
                self.seq = 0
            time.sleep(0.1)  # 延时
            
    def send_double(self): 
        self.seq = 0
        self.result = 0
        send_count = 0
        while True:
                self.log_print(self.result)
                print(self.result)
                header = bytearray(struct.pack('<BHB', 0xA5, 0x0007,self.seq)) + bytearray(1)  # 为CRC8额外添加一个字节
                append_CRC8_check_sum(header)  # 此时header包括了CRC8校验和
                if self.result != 0 and self.conut != 0:
                    send_count = send_count+1
                    if send_count>= 2:
                        data = struct.pack('<H', 0x0301) + struct.pack('<HHHB', 0x0121, 109 ,0x8080 , 2) #L=Bule 109 Red 9
                    else:
                        data = struct.pack('<H', 0x0301) + struct.pack('<HHHB', 0x0121, 109 ,0x8080 , send_count) #L=Bule 109 Red 9
                else:
                    data = struct.pack('<H', 0x0301) + struct.pack('<HHHB', 0x0121, 109 ,0x8080 , 0) #L=Bule 109 Red 9
                message = header + bytearray(data) + bytearray(2)  # 额外2个字节用于CRC16
                append_CRC16_check_sum(message)  # 添加CRC16校验和
                hex_representation = ' '.join(format(byte, '02X') for byte in message)
                self.log_print("Message in hex:", hex_representation)
                self.log_print(f"Sent message {self.seq}")
                print("Message in hex:", hex_representation)
                print(f"Sent message {self.seq}")
                with self.lock:  # 使用锁保护串口写入
                    self.ser.write(message)  # 发送消息
                self.seq += 1  # 序列号递增\=
                if self.seq == 256:
                        self.seq = 0
                index = 0
                time.sleep(3)  # 延时

    def run(self):
        """并行执行数据读取和发送"""
        recv_thread = Thread(target=self.read_data)
        send_thread = Thread(target=self.send_data)
        send_doubule_thread = Thread(target=self.send_double)
        recv_thread.start()
        send_thread.start()
        send_doubule_thread.start()
        recv_thread.join()  # 确保主线程等待接收线程完成
        send_thread.join()  # 确保主线程等待发送线程完成
        send_doubule_thread.join()

if __name__ == "__main__":
    referee = Referee()
    referee.run()
