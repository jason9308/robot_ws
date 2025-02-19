import rclpy
from rclpy.node import Node

import os
import sys
import serial
import binascii
import time
from functools import partial
from sympy import true

from grpr2f85_ifaces.srv import Reset, \
                                GetGripperStatus, \
                                SetGripperState

class Gripper(Node):
    def __init__(self,usbPort=1,name= 'my_2f85gripper', parent=None):
        super().__init__(name)
        self.usbPort=usbPort
        if (self.setup(self.usbPort) & self.set_gripper_state()):
            self.srv_reset = self.create_service(Reset, 'reset', self.reset_callback)
            self.srv_set_gripper_state = self.create_service(SetGripperState,
                                                             'set_gripper_state',
                                                              self.set_gripper_state_callback)
            self.srv_get_gripper_status = self.create_service(GetGripperStatus,
                                                             'get_gripper_status',
                                                              self.get_gripper_status_callback)
            #self.gripper_status_publisher = self.create_publisher(String, 'gripper_status', 10)
            #self.timer = self.create_timer(1, self.gripper_status_callback)
            self.get_logger().info(f'Initial ok')                       
        else:
            self.get_logger().info(f'no gripper found on usbPort{self.usbPort}')
            raise ConnectionError(f'no gripper found on usbPort{self.usbPort}')
        
    def setup(self,usbPort):
        try:
            self.ser = serial.Serial(port=f'/dev/ttyUSB{usbPort}', baudrate=115200, 
                                     timeout=1, parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
            self.get_logger().info(f'Gripper setup on USB port {usbPort}')
            self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30')
            #print('setup send:', b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30')
            data_raw = self.ser.readline()
            #print('setup recieve:', data_raw)
            data = binascii.hexlify(data_raw, ' ') # bytes
            self.get_logger().info(f'Response: {data}')
            #print((data != b''))
            return (data != b'')
        except Exception as e:
            self.get_logger().info(f'{e}')
            print(e)
            pass

    def mycrc(self, input):
        crc = 0xffff
        # i = 0
        for byte in input:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xa001
                else:
                    crc = crc >> 1
            # i+=1
            # print('---'+hex(crc))
        crc = (((crc << 8) | (crc >> 8)) &
               0xFFFF).to_bytes(2, byteorder='big')
        return crc

    def get_gripper_status(self):
            #bool get 0
            #---
            #bool ok 0
            #uint8 status_code
            #string result ''
        self.get_logger().info(f'Read gripper status')
        #print('Read gripper status')
        self.ser.write(b'\x09\x03\x07\xD0\x00\x03\x04\x0E')
        data_raw = self.ser.readline()  # bytes too slow
        data_show = binascii.hexlify(data_raw, ' ')
        self.get_logger().info(f'Response: {data_show}')
        #print('Response:', data_show)
        # a = int.from_bytes(data, byteorder='big')
        # print(a)
        # b = int.from_bytes('\x00\x00\x00\xFF\x00\x00\x00\x00\x00\x00\x00', byteorder='big')
        # b = b'\x00\x00\x00\xFF\x00\x00\x00\x00\x00\x00\x00'
        gripper_status_mask = b'\xFF'
        gripper_status = bytes([data_raw[3] & gripper_status_mask[0]])
        # print('data[3]:',binascii.hexlify(bytes([data_raw[3]])))
        # print('gripper status:',binascii.hexlify(gripper_status))
        gripper_status = int.from_bytes(gripper_status, 'big')
        if gripper_status & 0b00001000 == 0b00001000 and\
                gripper_status & 0b11000000 == 0b00000000:
            self.get_logger().info(f'No Object Detect (gripper moving)')
            #print('No Object Detect (gripper moving)')
            status_code=0
            status_description='No Object Detect (gripper moving)'
        elif gripper_status & 0b11000000 == 0b01000000:
            self.get_logger().info(f'Object Detect (opening)')
            #print('Object Detect (opening)')
            status_code=1
            status_description='Object Detect (opening)'
        elif gripper_status & 0b11000000 == 0b10000000:
            self.get_logger().info(f'Object Detect (closing)')
            #print('Object Detect (closing)')
            status_code=2
            status_description='Object Detect (closing)'
        elif gripper_status & 0b11000000 == 0b11000000 or\
            (gripper_status & 0b00001000 == 0b00000000 and
             gripper_status & 0b11000000 == 0b00000000):
            self.get_logger().info(f'No Object Detect (gripper stop)')
            #print('No Object Detect (gripper stop)')
            status_code=3
            status_description='No Object Detect (gripper stop)'
        
        return True, status_code, status_description

    def set_gripper_state(self, position=0, speed=255, force=255, wait_time=0):
            #uint8 position 0~255, Open~Close
            #uint8 speed 0~255
            #uint8 force 0~255
            #uint16 wait_time 0
            #---
            #bool ok 0
        self.get_logger().info(f'Gripper position: {position}/255')
        position_byte = position.to_bytes(1,'big')  # full open
        speed_byte = speed.to_bytes(1,'big')  # 00:min;FF:max
        force_byte = force.to_bytes(1,'big')  # 00:min;FF:max
        time.sleep(wait_time)  # calibrate:\x15
        command = b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00' +\
                position_byte + speed_byte + force_byte
        command+=self.mycrc(command)
        print('set state:', command)
        self.ser.write(command)
        data_raw = self.ser.readline()
        data = binascii.hexlify(data_raw, ' ') # bytes            
        self.get_logger().info(f'Response: {data}')
        print('Response:', data)
        return True
    
    def reset_callback(self,request, response):
            #bool get 0
            #---
            #bool ok 0
        if request.get:
            self.get_logger().info(f'Gripper setup on USB pott {self.usbPort}')
            self.ser.write(
                b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30')
            data_raw = self.ser.readline()
            data = binascii.hexlify(data_raw, ' ') # bytes            
            self.get_logger().info(f'Response: {data}')
            #print('Response:', data)
            self.get_logger().info(f'Reset Gripper')
            #print('Reset: 09 10 03 E8 00 03 06 01 00 00 00 00 00 72 E1')
            self.ser.write(
                b'\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00\x00\x72\xE1')
            while True:
                self.ser.write(b'\x09\x03\x07\xD0\x00\x01\x85\xCF')
                data_raw = self.ser.readline()
                self.get_logger().info(f'{data_raw}')
                #print(data_raw)
                if data_raw == b'\x09\x03\x02\x11\x00\x55\xD5':
                    self.get_logger().info(f'Activate Not Complete')
                    #print('Activate Not Complete')
                elif data_raw == b'\x09\x03\x02\x31\x00\x4C\x15':
                    response.ok = True
                    self.get_logger().info(f'Activate Complete')
                    #print('Activate Complete')
                    break
        else:
            self.get_logger().info(f'Fatal Error')
            return response
        return response
      
    def get_gripper_status_callback(self,request, response):
            #bool get 0
            #---
            #bool ok 0
            #uint8 status_code
            #string result '' 
        if request.get: 
            response.ok, response.status_code, response.result = self.get_gripper_status()
        return response

    def set_gripper_state_callback(self, request, response):
            #uint8 position 0
            #uint8 speed 255
            #uint8 force 255
            #uint16 wait_time 0
            #---
            #bool ok 0
            #uint8 status_code
            #sting result ''
        response.ok = self.set_gripper_state(request.position, 
                                             request.speed, 
                                             request.force,
                                             request.wait_time)
        _, response.status_code , response.result = self.get_gripper_status()
        return response
    
def get_port_from_argv():
    valid_argv = [i for i in sys.argv if 'usb_port:=' in i]
    if len(valid_argv)>=1:
        return valid_argv[0][10:]
    else:
        print('Didn\'t find argument: usb_port:=xxx')
        return ''

def main(args=None):
    port = get_port_from_argv()
    rclpy.init(args=args)
    node = Gripper(usbPort=port)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
