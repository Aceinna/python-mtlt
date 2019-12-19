'''
version 1.0.0 in Aceinna
mtlt products(MTLT305D and 300RI included now) CAN Bus read and send message module
Requires PI3B and CAN_HAT(shopping link: https://m.tb.cn/h.eRtgNe2)
with H/L of CAN_HAT connected with H/L from sensor side
only store the angle, acc, rate to can_data.txt
@author: tejas_pashte
'''

# follow http://www.waveshare.net/wiki/RS485_CAN_HAT

import os
import sys
import can
import time
import threading
import binascii
import gps
import struct
import requests
import csv
import datetime
import gnsscal
import serial
from ubx import UbxStream
# import ubx

if sys.version_info[0] > 2:
    from queue import Queue
else:
    from Queue import Queue

#j1939 with extended id
class can_mtlt:
    def __init__(self, chn = 'can0', bus_type = 'socketcan'):
        os.system('sudo /sbin/ip link set can0 up type can bitrate 250000')  # run this command first with correct baud rate
        self.can0 = can.interface.Bus(channel = chn, bustype = bus_type)  # socketcan_native
        self.rows = []
        self.writer = ""

        fields = ["timeITOW","time","roll","pitch","xRate","yRate","zRate","xAccel","yAccel","zAccel"]
        with open('can_data.txt', 'w') as f:  # empty the can_data.txt
            # f.write("time,roll,pitch,xRate,yRate,zRate,xAccel,yAccel,zAccel,lat,long,gps_yaw,gps_pitch,gps_roll,cog,sog"+ "\n")
            f.write("time,rsvd1,rsvd2,xRate,yRate,zRate,xAccel,yAccel,zAccel,lat,long,gps_yaw,gps_pitch,gps_roll,cog,sog"+ "\n")
            print('')

        # with open('file.csv','w',newline='') as csvfile:
        #     writer = csv.DictWriter(csvfile, fieldnames = fields)
        #     writer.writeheader()

            # for row in reader:
            #     self.rows.append(row)

        self.msg_queue = Queue()
        self.fw_version_msg_queue = Queue()
        self.id_msg_queue = Queue()
        self.hw_bit_msg_queue = Queue()
        self.sw_bit_msg_queue = Queue()
        self.sensor_status_msg_queue = Queue()
        self.pdu_dict = {
            "time_stamp":0,
            "id":0x0000,
            "extended_id":False,
            "priority":0,
            "src":0,
            "dlc":0,
            "pgn":0,
            "payload":0
        }
        self.thread_put = threading.Thread(target=self.put_msg)
        self.thread_read = threading.Thread(target=self.read_msg)
    def print_slope(self,msg):   # unit: degree
        pitch_uint = msg.data[0] + 256 * msg.data[1] +  65536 * msg.data[2]
        roll_uint = msg.data[3] + 256 * msg.data[4] +  65536 * msg.data[5]
        pitch = pitch_uint * (1/32768) - 250.0
        roll = roll_uint * (1/32768) - 250.0

        print('Time: {2:18.6f} Roll: {0:6.2f} Pitch: {1:6.2f}'.format(roll,pitch,msg.timestamp))
        with open('can_data.txt', 'a') as f:
            # f.write('Roll: {0:6.2f} Pitch: {1:6.2f}'.format(roll,pitch) + "\n")
            # f.write('{0:6.2f},{1:6.2f},{2:6.2f},0,0,0,0,0,0,0,0,0,0,0,0,0'.format(msg.timestamp,roll,pitch) + "\n")
            f.write('{0:6.2f},0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'.format(msg.timestamp,roll,pitch) + "\n")

        # with open('file.csv','r') as csvfile:
        #     reader = csv.reader(csvfile)
        #     lines = list(reader)
        #     lines.append([msg.timestamp,msg.timestamp,roll,pitch,0,0,0,0,0,0])
        #
        # with open('file.csv','w') as file:
        #     writer = csv.writer(file)
        #     writer.writerows(lines)

    def print_accel(self,msg):   # unit: g

        ax_uint = msg.data[0] + 256 * msg.data[1]
        ay_uint = msg.data[2] + 256 * msg.data[3]
        az_uint =  msg.data[4] + 256 * msg.data[5]
        ax = ax_uint * (0.01) - 320.0
        ay = ay_uint * (0.01) - 320.0
        az = az_uint * (0.01) - 320.0

        # date = time.ctime(msg.timestamp)
        print('Time: {3:18.6f} AX  : {0:6.2f} AY   : {1:6.2f} AZ: {2:6.2f}'.format(ax,ay,az,msg.timestamp))
        with open('can_data.txt', 'a') as f:
            # f.write('AX  : {0:6.2f} AY   : {1:6.2f} AZ: {2:6.2f}'.format(ax,ay,az) + "\n")
            f.write('{0:6.2f},0,0,0,0,0,{1:6.2f},{2:6.2f},{3:6.2f},0,0,0,0,0,0,0'.format(msg.timestamp,ax,ay,az) + "\n")

        # with open('file.csv','r') as csvfile:
        #     reader = csv.reader(csvfile)
        #     lines = list(reader)
        #     lines.append([msg.timestamp,msg.timestamp,0,0,0,0,0,ax,ay,az])
        #
        # with open('file.csv','w') as file:
        #     writer = csv.writer(file)
        #     writer.writerows(lines)

    def parse_gps_packet_1(self,dlc,msg):

        empty = ""
        long_1 = 0
        lat_1= 0
        long_1 = [str(dlc[7]),str(dlc[6]),str(dlc[5]),str(dlc[4])]
        lat_1 = [str(dlc[3]),str(dlc[2]),str(dlc[1]),str(dlc[0])]

        long_1_join = empty.join(long_1)
        lat_1_join = empty.join(lat_1)

        long_1_new = int(long_1_join,16)
        lat_1_new = int(lat_1_join,16)



        long = (long_1_new * 0.0000001) - 210
        lat = (lat_1_new * 0.0000001) - 210

        print('lat,long')
        print(long, lat)

        with open('can_data.txt', 'a') as f:
            # f.write('WX  : {0:6.2f} WY   : {1:6.2f} WZ: {2:6.2f}'.format(wx,wy,wz) + "\n")
            f.write('{0:6.2f},0,0,0,0,0,0,0,0,{1:6.9f},{2:6.9f},0,0,0,0,0'.format(msg.timestamp,lat,long) + "\n")

        return [long,lat]

    def parse_gps_packet_3(self,dlc,msg):
        empty = ""
        yaw_1 = [str(dlc[2]),str(dlc[1])]
        pitch_1 = [str(dlc[4]),str(dlc[3])]
        roll_1 = [str(dlc[6]),str(dlc[5])]

        yaw_1_join = empty.join(yaw_1)
        pitch_1_join = empty.join(pitch_1)
        roll_1_join = empty.join(roll_1)

        yaw_1_new = int(yaw_1_join,16)
        pitch_1_new = int(pitch_1_join,16)
        roll_1_new = int(roll_1_join,16)

        yaw = (yaw_1_new * 0.0001) - 3.14159
        pitch = (pitch_1_new * 0.0001) - 3.14159
        roll = (roll_1_new * 0.0001) - 3.14159

        with open('can_data.txt', 'a') as f:
            print("yaw")
            print(yaw)
            print("pitch" + str(pitch))
            print("roll" + str(roll))
            # f.write('WX  : {0:6.2f} WY   : {1:6.2f} WZ: {2:6.2f}'.format(wx,wy,wz) + "\n")
            f.write('{0:6.2f},0,0,0,0,0,0,0,0,0,0,{1:6.4f},{2:6.4f},{3:6.4f},0,0'.format(msg.timestamp,yaw,pitch,roll) + "\n")

        return [yaw,pitch,roll]

    def parse_gps_packet_2(self,dlc,msg):
        empty = ""
        cog_1 = [str(dlc[3]),str(dlc[2])]
        sog_1 = [str(dlc[5]),str(dlc[4])]

        cog_1_join = empty.join(cog_1)
        sog_1_join = empty.join(sog_1)

        cog_new = int(cog_1_join,16)
        sog_new = int(sog_1_join,16)

        cog = (cog_new * 0.0001) - 3.14
        sog = (cog_new * 0.0001)
        # yaw_1_join = empty.join(dlc)
        # bits_dlc = bin(int(yaw_1_join, base=16)).lstrip('0b')
        #
        # cog = (int(bits_dlc[16:32],2) * 0.0001) - 3.14
        # print(int(bits_dlc[16:32],2))
        # sog = (int(bits_dlc[32:48],2) * 0.0001)
        #
        with open('can_data.txt', 'a') as f:
            # f.write('WX  : {0:6.2f} WY   : {1:6.2f} WZ: {2:6.2f}'.format(wx,wy,wz) + "\n")
            f.write('{0:6.2f},0,0,0,0,0,0,0,0,0,0,0,0,0,{1:6.2f},{2:6.2f}'.format(msg.timestamp,cog,sog) + "\n")

        return [cog,sog]


    def print_rate(self,msg):    # degree per second
        wx_uint = msg.data[0] + 256 * msg.data[1]
        wy_uint = msg.data[2] + 256 * msg.data[3]
        wz_uint =  msg.data[4] + 256 * msg.data[5]
        wx = wx_uint * (1/128.0) - 250.0
        wy = wy_uint * (1/128.0) - 250.0
        wz = wz_uint * (1/128.0) - 250.0

        print(msg.timestamp)
        print('WX  : {0:6.2f} WY   : {1:6.2f} WZ: {2:6.2f}'.format(wx,wy,wz))
        with open('can_data.txt', 'a') as f:
            # f.write('WX  : {0:6.2f} WY   : {1:6.2f} WZ: {2:6.2f}'.format(wx,wy,wz) + "\n")
            f.write('{0:6.2f},0,0,{1:6.2f},{2:6.2f},{3:6.2f},0,0,0,0,0,0,0,0,0,0'.format(msg.timestamp,wx,wy,wz) + "\n")

        # with open('file.csv','r') as csvfile:
        #     reader = csv.reader(csvfile)
        #     lines = list(reader)
        #     lines.append([msg.timestamp,msg.timestamp,0,0,wx,wy,wz,0,0,0])
        #
        # with open('file.csv','w') as file:
        #     writer = csv.writer(file)
        #     writer.writerows(lines)

 #to be finished
    # def gps_position(self,msg):
    #     session = gps.gps("localhost", "2947")
    #     session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
    #     rep = session.next()
    #
    #     if (rep["class"] == "TPV") :
    #         self.gps_position['lat'] = rep.lat
    #         self.gps_position['long'] = rep.long

    def start_record(self):
        self.thread_put.start()
        self.thread_read.start()

    def read_msg(self):    # print all messages
        while (True):
            if (self.msg_queue.not_empty):
                msg_read = self.msg_queue.get()
            else:
                msg_read = self.can0.recv()

            self.get_pdu_list(msg = msg_read)

            # print(self.pdu_dict)


            #print to screen and can_data.txt file
            # if self.pdu_dict["pgn"] == 65267:
            #     # with open('can_data.txt', 'a') as f:  #store the raw message to file
            #         # f.write(msg_read.__str__() + "  ")
            #
            #     self.print_gps(msg_read)
            #     print('gps detected')
            if self.pdu_dict["pgn"] == 61481:
                # with open('can_data.txt', 'a') as f:  #store the raw message to file
                    # f.write(msg_read.__str__() + "  ")
                self.print_slope(msg_read)


            elif self.pdu_dict["pgn"] == 63489:
                print("long","lat")
                self.parse_gps_packet_1(self.pdu_dict["dlc"],msg_read)

            elif self.pdu_dict["pgn"] == 63490:
                print("cog sog")
                self.parse_gps_packet_2(self.pdu_dict["dlc"],msg_read)

            elif self.pdu_dict["pgn"] == 61721:
                print("yaw","pitch","roll")
                self.parse_gps_packet_3(self.pdu_dict["dlc"],msg_read)

            # elif  self.pdu_dict["pgn"] == 63490:

            elif self.pdu_dict["pgn"] == 61482:
                # with open('can_data.txt', 'a') as f:  #store the raw message to file
                    # f.write(msg_read.__str__() + "  ")
                self.print_rate(msg_read)
                # pass
            elif self.pdu_dict["pgn"] == 61485:
                # with open('can_data.txt', 'a') as f:  #store the raw message to file
                    # f.write(msg_read.__str__() + "  ")
                self.print_accel(msg_read)
                # pass
            elif self.pdu_dict["pgn"] == 61183:
                pass
                print("Time:", self.pdu_dict["time_stamp"], self.pdu_dict["pgn"], self.pdu_dict["payload"], "------Address_Claiming")     #only print to screen, no write to file
            elif self.pdu_dict["pgn"] == 65242:
                self.fw_version_msg_queue.put(self.pdu_dict)
                print(self.pdu_dict)
            elif self.pdu_dict["pgn"] == 64965:
                self.id_msg_queue.put(self.pdu_dict)
                print(self.pdu_dict)
            elif self.pdu_dict["pgn"] == 65362:
                self.hw_bit_msg_queue.put(self.pdu_dict)
                print(self.pdu_dict)
            elif self.pdu_dict["pgn"] == 65362:
                self.hw_bit_msg_queue.put(self.pdu_dict)
                print(self.pdu_dict)
            elif self.pdu_dict["pgn"] == 65363:
                self.sw_bit_msg_queue.put(self.pdu_dict)
                print(self.pdu_dict)
            elif self.pdu_dict["pgn"] == 65364:
                self.sensor_status_msg_queue.put(self.pdu_dict)
                print(self.pdu_dict)
            else:
                # print( "\n")
                print(msg_read, "\n")

    def put_msg(self):
        while (True):
            msg_save = self.can0.recv()
            self.msg_queue.put(msg_save)

    def send_msg(self, id_int, data_list):  # id_int = 0x18FF5500, data_list =[128, 1, 0, 0, 0, 0, 0, 0] set ODR is 100hz
        send_msg = can.Message(arbitration_id=id_int, data=data_list, is_extended_id=True)
        self.can0.send(send_msg)

    def get_fw_version(self):
        data = [0, 254, 218]
        my_can.send_msg(0x18EAFF00,data)
        print("fw version:", self.fw_version_msg_queue.get())
        # return(self.fw_version_msg_queue.get())
    def get_id(self):
        data = [0, 253, 197]
        my_can.send_msg(0x18EAFF01,data)
        print("id:", self.id_msg_queue.get())
    def get_hw_status(self):
        self.start_record()
        data = [0, 255, 82]
        my_can.send_msg(0x18EAFF02,data)
        print("hw bit:", self.hw_bit_msg_queue.get())
    def get_sw_status(self):
        self.start_record()
        data = [0, 255, 83]
        my_can.send_msg(0x18EAFF03,data)
        print("sw bit:", self.sw_bit_msg_queue.get())
    def get_sensor_status(self):
        self.start_record()
        data = [0, 255, 84]
        my_can.send_msg(0x18EAFF04,data)
        print("sensor status:", self.sensor_status_msg_queue.get())

    def save_configuration(self):
        data = [0, 128]
        my_can.send_msg(0x18FF5100,data)
        print(data)
    def reset_algorithm(self):
        data = [0, 128]
        my_can.send_msg(0x18FF5000,data)
        print(data)
    def set_odr(self,odr_int):
        data = [128, odr_int]
        my_can.send_msg(0x18FF5500,data)
        print("set odr:", data)
    def set_pkt_type(self,type_int):
        data = [128, type_int]
        my_can.send_msg(0x18FF5600,data)
        print(data)

    def set_lpf_filter(self,rate_int,acc_int):
        data = [128, rate_int, acc_int]
        my_can.send_msg(0x18FF5700,data)
        msg_read = self.can0.recv()
        print (msg_read)
        msg_list = list(str(msg_read).split(" "))     # the list include: time, id, priority, DLC, data0,data1,...,data7
        msg_list = [x for x in msg_list if x != '']
        self.pdu_dict["pgn"] = (0x00FFFF00 & int(msg_list[1],16)) >> 8
        print (self.pdu_dict)
        print(data)

    def set_orientation(self,value_int):
        value_msb = 0xFF00 & value_int #get the msb value of certain number
        value_msb = value_msb >> 8
        value_lsb = 0x00FF & value_int
        data = [128, value_msb, value_lsb]
        my_can.send_msg(0x18FF5800,data)
        print(data)

    def get_pdu_list(self, msg = None, msg_dict = None):
        if msg == None:
            msg = self.can0.recv()
        msg_list = list(str(msg).split(" "))     # the list include: time, id, priority, DLC, data0,data1,...,data7
        msg_list = [x for x in msg_list if x != '']       # delete the empty items

        if msg_list[3] == '19f80180'  or msg_list[3] == '19f80280' or msg_list[3] == '19f11980':
            self.pdu_dict["time_stamp"] = float(msg_list[1])
            self.pdu_dict["id"] = int(msg_list[3],16)
            # self.pdu_dict["extended_id"] = msg.id_type
            # self.pdu_dict["priority"] = int(msg_list[2],2)
            # self.pdu_dict["src"] = 0x000000FF & int(msg_list[1],16)
            # self.pdu_dict["dlc"] = int(msg_list[6:15],10)
            self.pdu_dict["dlc"] = msg_list[7:15]
            self.pdu_dict["pgn"] = (0x00FFFF00 & int(msg_list[3],16)) >> 8
            self.pdu_dict["payload"] = msg_list[7:15]
            msg_dict = self.pdu_dict
            return msg_dict
        else:
            # print(msg)
            pgn = (0x00FFFF00 & int(msg_list[3],16)) >> 8
            # print(pgn)
            self.pdu_dict["time_stamp"] = float(msg_list[1])
            self.pdu_dict["id"] = int(msg_list[3],16)
            # self.pdu_dict["extended_id"] = msg.id_type
            # self.pdu_dict["priority"] = int(msg_list[2],2)
            # self.pdu_dict["src"] = 0x000000FF & int(msg_list[1],16)
            self.pdu_dict["dlc"] = msg_list[7:15]
            self.pdu_dict["pgn"] = (0x00FFFF00 & int(msg_list[3],16)) >> 8
            self.pdu_dict["payload"] = ''.join(msg_list[7:15])
            # msg_dict = self.pdu_dict
            # return msg_dict




    def read_lianshi(self):
       # pgn 62256
        x = UbxStream(serial.Serial(port="/dev/ttyUSB0",baudrate=9600,timeout =1))
        while True:
            x.enable_message(1,7)
            time.sleep(0.3)
            nav_pvt = x.read()

            time.sleep(0.2)

            #packet 1 params
            pre_nav_speed = nav_pvt.gSpeed
            nav_speed = ((pre_nav_speed * 3600) / 1000000)/256

            speed= self.int2Hex(int(lat),0)
            packet speed = speed + [0,0,0,0]
            my_can.send_msg(0x18F33080,speed)




if __name__ == "__main__":
    my_can = can_mtlt('can0', 'socketcan')


    #set messages
    # my_can.set_odr(1)

    my_can.read_lianshi()
    # time.sleep(0.5)

    #print and save messages of slope, acc, rate
    # my_can.start_record()
