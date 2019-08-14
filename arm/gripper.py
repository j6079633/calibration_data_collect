import os
import sys
import serial
import binascii
import time
from PyCRC.CRC16 import CRC16
from functools import partial

class Gripper:
    def __init__(self, parent=None):
        self.gripper_setup()

    def gripper_setup(self):
        try:
            self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0.1, parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        except serial.serialutil.SerialException as e:
            print(e)
    def gripper_reset(self):
        # activate gripper
        print('Activate gripper:')
        print('Clear: 09 10 03 E8 00 03 06 00 00 00 00 00 00 73 30')
        self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30')
        data_raw = self.ser.readline()  # bytes
        data = binascii.hexlify(data_raw)
        print('Response:', data)

        print('Set: 09 10 03 E8 00 03 06 01 00 00 00 00 00 72 E1')
        self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00\x00\x72\xE1')
        # data_raw = self.ser.readline()  # bytes
        # data = binascii.hexlify(data_raw)
        # print('Response:', data)
        while True:
            self.ser.write(b'\x09\x03\x07\xD0\x00\x01\x85\xCF')
            data_raw = self.ser.readline()
            if data_raw == b'\x09\x03\x02\x11\x00\x55\xD5':
                print('Activate Not Complete')
            elif data_raw == b'\x09\x03\x02\x31\x00\x4C\x15':
                print('Activate Complete')
                break

    def initial_gripper_pose(self,wait_time=0):
        print('Close gripper')
        position = b'\x91'  # 7F
        time.sleep(wait_time) # calibrate:\x15
        speed = b'\xFF'  # 00:min;FF:max calibrate:\x15
        force = b'\xFF'  # 00:min;FF:max 15
        input = b''.join([b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00', position])
        input = b''.join([input, speed])
        input = b''.join([input, force])
        temp = self.mycrc(input)
        crc = (((temp << 8) | (temp >> 8)) & 0xFFFF).to_bytes(2, byteorder='big')
        write = b''.join([input,crc])

        self.ser.write(write)
        data_raw = self.ser.readline()  # bytes
        data = binascii.hexlify(data_raw)
        print('Response:', data)

    def gripper_on(self, wait_time=0):
        print('Open gripper')
        position = b'\x00' #open
        time.sleep(wait_time) # calibrate:\x15
        speed = b'\xFF' # 00:min;FF:max
        force = b'\xFF' # 00:min;FF:max

        input = b''.join([b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00', position])
        input = b''.join([input, speed])
        input = b''.join([input, force])

        temp = self.mycrc(input)
        crc = (((temp << 8) | (temp >> 8)) & 0xFFFF).to_bytes(2, byteorder='big')
        write = b''.join([input, crc])

        self.ser.write(write)
        data_raw = self.ser.readline()  # bytes
        data = binascii.hexlify(data_raw)
        print('Response:', data)

    def gripper_off(self, wait_time=0):
        print('Close gripper')
        position = b'\xFF'  # close
        time.sleep(wait_time) # calibrate:\x15
        speed = b'\x15'  # 00:min;FF:max calibrate:\x15
        force = b'\xFF'  # 00:min;FF:max 15
        input = b''.join([b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00', position])
        input = b''.join([input, speed])
        input = b''.join([input, force])
        temp = self.mycrc(input)
        crc = (((temp << 8) | (temp >> 8)) & 0xFFFF).to_bytes(2, byteorder='big')
        write = b''.join([input,crc])

        self.ser.write(write)
        data_raw = self.ser.readline()  # bytes
        data = binascii.hexlify(data_raw)
        print('Response:', data)

    def gripper_soft_off(self, wait_time=0):
        print('Close gripper')
        position = b'\xF0'  # close
        time.sleep(wait_time) # calibrate:\x15
        speed = b'\x15'  # 00:min;FF:max calibrate:\x15
        force = b'\x15'  # 00:min;FF:max 15
        input = b''.join([b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00', position])
        input = b''.join([input, speed])
        input = b''.join([input, force])
        temp = self.mycrc(input)
        crc = (((temp << 8) | (temp >> 8)) & 0xFFFF).to_bytes(2, byteorder='big')
        write = b''.join([input,crc])

        self.ser.write(write)
        data_raw = self.ser.readline()  # bytes
        data = binascii.hexlify(data_raw)
        print('Response:', data)

    def mycrc(self, input):
        crc = 0xffff
        # i = 0
        for byte in input:
            crc ^= byte
            for _ in range(8):
                if crc&0x0001:
                    crc = (crc>>1)^0xa001
                else:
                    crc = crc>>1
            # i+=1
            # print('---'+hex(crc))
        return crc