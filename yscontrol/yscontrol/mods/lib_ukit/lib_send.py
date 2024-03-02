# -*- coding: utf-8 -*-

from .lib_ukit import data_pb2
from .lib_ukit import header_pb2
import binascii
import re

'''本python库用于yanshee机器人控制uKit2.0主控广播消息格式的封装，遵循《优必选智能传感器协议》
   包括使用google protobuf封装格式和crc8校验等方法技术，欢迎使用。
'''
def crc8_data(msg):
    poly = 0x07
    msg_crc = 0x00
    xorout = 0x55
    msg_list = re.findall(r'.{2}', msg.decode('utf-8'))
    for key in msg_list:
        msg_crc ^= int(key,16)
        for bit in range(8):
            if (msg_crc&0x80):
                msg_crc=(msg_crc<<1)^poly
            else:
                msg_crc <<= 1
        msg_crc &= 0xFF
    msg_crc ^= xorout
    crc_data = hex(msg_crc)
    return crc_data


def make_send_data(header_str,header_crc,data_str,data_crc):
    frame_head = "db"
    len_basic = int(len(header_str)/2)
    header_length = str(hex(len_basic))[2:]
    if(0<len_basic)and(len_basic<16):
        header_length = "0" + header_length
    header_body = header_str
    header_check = header_crc
    data_body = data_str
    data_check = data_crc
    send_data = frame_head + header_length + header_body + header_check + data_body + data_check
    return send_data

def lib_send_data_to_uKit(msg):
    data_raw = data_pb2.d2c2700_intelligent_dev_wifi_send_boardcast_msg_ph()
    data_raw.name = "BMSG"
    data_raw.value = msg
    # 序列化
    data_pb = data_raw.SerializeToString()
    header_raw = header_pb2.Header()
    header_raw.dev = 2
    header_raw.cmd = 2700
    header_raw.id = 0
    header_raw.dataLen = len(data_pb)
    header_raw.attr = header_pb2.Header.PUSH
    # 序列化
    header_pb = header_raw.SerializeToString()
    header_hex = binascii.hexlify(header_pb)
    data_hex = binascii.hexlify(data_pb)
    header_crc = crc8_data(header_hex)
    data_crc = crc8_data(data_hex)
    header_crc = header_crc[2:]
    data_crc = data_crc[2:]
    data_msg = make_send_data(header_hex.decode(),header_crc,data_hex.decode(),data_crc)
    return data_msg

def lib_get_msg_from_uKit(msg):
    #数据预处理
    msg_list = re.findall(r'.{2}', msg)
    if(len(msg_list) > 0):
        if msg_list[0] == 'db'or msg_list[0] == 'DB':
            header_hex = ''.join(msg_list[2:11])
            header_crc = crc8_data(header_hex.encode())
            #检验帧头crc8是否正确
            if header_crc[2:] == msg_list[11]:
                data_hex = ''.join(msg_list[12:-1])
                data_crc = crc8_data(data_hex.encode())
                #检验数据部分crc8是否正确
                if data_crc[2:] == msg_list[-1]:
                    data_pb = binascii.unhexlify(data_hex.encode())
                    data_raw = data_pb2.d2c2700_intelligent_dev_wifi_send_boardcast_msg_ph()
                    # 反序列化
                    data_raw.ParseFromString(data_pb)
                    name = data_raw.name
                    #获取接收到的字符串
                    value = data_raw.value
                    return value
    return ""
