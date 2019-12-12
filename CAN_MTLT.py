'''
version 1.0.0 in Aceinna
mtlt products(MTLT305D and 300RI included now) CAN Bus read and send message module
Requires PI3B and CAN_HAT(shopping link: https://m.tb.cn/h.eRtgNe2)
with H/L of CAN_HAT connected with H/L from sensor side
only store the angle, acc, rate to can_data.txt
@author: cek
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

        print(long_1_new)
        print(lat_1_new)

        long = (long_1_new * 0.0000001) - 210
        lat = (lat_1_new * 0.0000001) - 210

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


            #to be finished
            # if self.gps_position["lat"] > 0:
            #     data = [elf.gps_position["lat"],elf.gps_position["long"]]
            #     my_can.send_msg(0x18FEF300,data)

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

    def mag_alignment_start(self):
        status = self.mag_alignment_calculate(1)
        while status != 13:
            status = self.mag_alignment_status()

        if status == 13:
            self.mag_alignment_save()

    def mag_alignment_status(self):
            status = self.mag_alignment_calculate(0)
            return status

    def mag_alignment_save(self):
        sys.stdout.write('Accept values? Type y/n: ')
        choice = input().lower()
        if choice  == 'y':
            self.mag_alignment_calculate(5,True)
            print('values saved')
        elif choice == 'n':
            print('values rejected')
            return

    def mag_alignment_calculate(self,payload,calc = False):
        data = [128,payload]
        self.can0.flush_tx_buffer()
        my_can.send_msg(0x18FF5E00,data)
        msg_read = self.can0.recv()
        msg_list = list(str(msg_read).split(" "))
        msg_list = [x for x in msg_list if x != '']
        status = int(msg_list[5],16)
        if calc == True:
            status = msg_list[5:]
            # bitList = f msg
        return status

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

    def int2Hex(self,num,n):
        OFFSET = 1 << 32
        MASK = OFFSET - 1
        hex = '%08x' % (num + OFFSET & MASK)
        bytes = []

        for i in range(n, 4):
            temp = '0x' + hex[i * 2: i * 2 + 2]
            bytes.insert(0,int(temp,16))
        return bytes[::]

    def year_four(self,year):
        if year < 80:
            return year + 2000
        elif year < 1900 and year > 80:
            return year + 1900
        else:
            return year

    def is_skip_year(self,year):

        if (int(self.year_four(year)% 4) == 0 and int(self.year_four(year)% 100) != 0  or int(self.year_four(year)) % 400 == 0):
            return True
        else:
            return False

    def day_of_year(self,year,mon,day):
        totalDay = day;
        mon_index = 0
        day_per_month = [ 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 ]
        day_per_month_trunc = day_per_month[0:mon-1]

        sum_total_days = sum(day_per_month_trunc)

        if mon > 2 and self.is_skip_year(year) == True:
            sum_total_days = sum_total_days + 1

        return sum_total_days + day

    def convert_utc_gps(self,year,month,day,hour,minute,second):
        week_num = 0
        week_sec = 0
        doy = 0
        new_totalDay = 0
        if year < 1981:
            totalDay = 0
        else:
            totalDay = 360

        yearIndex = 1981

        for x in range(yearIndex,year):
            totalDay += 365
            if self.is_skip_year(x) == True:
                totalDay +=  1

        doy = self.day_of_year(year,month,day)


        new_totalDay = totalDay + doy

        week_num = int(new_totalDay/7)
        week_sec = ((new_totalDay-week_num * 7)*24*3600) + (hour*3600) + (minute*60) + (second)

        total_sec = 604800 * week_num + week_sec

        return total_sec



    # def read_gps(self):
    #     session = gps.gps("localhost", "2947")
    #     session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
    #     # print(self.convert_utc_gps(2019,11,20,13,30,20))
    #
    #
    #     while True:
    #         rep = session.next()
    #
    #         try :
    #             sat_num = 0
    #             #params for 64502 packet
    #             if (rep["class"] == "TPV") :
    #
    #                 # if sat_num > 0 :
    #
    #                 #params for 65390
    #                 year = int(rep.time[0:4])
    #                 month = int(rep.time[5:7])
    #                 day = int(rep.time[8:10])
    #                 hour = int(rep.time[11:13])
    #                 minute = int(rep.time[14:16])
    #                 second = int(rep.time[17:19])
    #
    #                 timestamp = self.convert_utc_gps(year,month,day,hour,minute,second)
    #                 ground_speed = int(rep.speed)
    #                 strTimestamp = self.int2Hex(timestamp,0)
    #                 strSpeed = self.int2Hex(ground_speed,0)
    #
    #                 # params for 65257 packet
    #                 lat = int((rep.lat + 210) * 1000000)
    #                 lon = int((rep.lon + 210) * 1000000)
    #                 strLat = self.int2Hex(lat,0)
    #                 strLong = self.int2Hex(lon,0)
    #
    #                 # params for 65391
    #                 mode = rep.mode
    #                 alt = int(rep.alt * 1000)
    #                 strAlt = self.int2Hex(alt,0)
    #
    #                 #params for 65392
    #
    #                 if hasattr(rep,'eph') == True:
    #                     eph = int(rep.eph * 1000)
    #                 else:
    #                     eph = 0
    #
    #                 strEph = self.int2Hex(eph,2)
    #
    #                 epv = int(rep.epv) * 1000
    #                 strEpv = self.int2Hex(epv,2)
    #
    #                 heading = int(((rep.track) * 100000)/ 182)
    #                 strHeading = self.int2Hex(heading,2)
    #
    #                 vehicle_gps = strLat + strLong
    #                 vehicle_speed = strTimestamp + strSpeed
    #
    #                 my_can.send_msg(0x18FEF380,vehicle_gps)
    #                 my_can.send_msg(0x18FF6E80,vehicle_speed)
    #
    #
    #                 ######### J1939 format: available for referrence ######
    #
    #                 # #params for 65256 packet
    #                 # #conversions according to j1939 excel
    #                 # alt = int((rep.alt + 2500) / 0.125)
    #                 # pitch = 0
    #                 # speed = int(rep.speed * 256)
    #                 # heading = int(rep.track * 128)
    #                 #
    #                 # strHeading = self.int2Hex(heading,2)
    #                 # strSpeed = self.int2Hex(speed,2)
    #                 # strPitch = self.int2Hex(pitch,2)
    #                 # strAlt = self.int2Hex(alt,2)
    #                 #
    #                 # vehicle_pos = strHeading + strSpeed + strPitch + strAlt
    #                 #
    #
    #                 # data = strLong + strLat
    #
    #                 #     hDop = int(rep.hdop * 10)
    #                 #     vDop = int(rep.vdop * 10)
    #                 #     pDop = int(rep.pdop * 10)
    #                 #     tDop = int(rep.tdop * 10)
    #                 #     rsvd1 = 0
    #                 #     rsvd2 = 0
    #                 #     rsdv3= 0
    #                 #     sat_num = len(rep.satellites)
    #                 #
    #                 #     dilution_val = [sat_num,hDop,vDop,pDop,tDop]
    #                 #     print("pgn 64502",dilution_val)
    #                 #
    #                 #        my_can.send_msg(0x18FBF680,dilution_val)
    #
    #             if (rep["class"] == "SKY"):
    #                 # params for 65391
    #                 sat_num = len(rep.satellites)
    #                 flags = 1
    #                 valid = 1
    #
    #                 strAlt.insert(0,valid)
    #                 strAlt.insert(0,flags)
    #                 strAlt.insert(0,sat_num)
    #
    #                 # packet_3 =  int(sat_num) + flags + valid + int(mode) + int(strAlt)
    #
    #                 #params for 65392
    #                 pDop = int(rep.pdop * 10)
    #                 strPdop = self.int2Hex(heading,2)
    #                 packet_2 = strEph + strEpv + strHeading + strPdop
    #
    #                 # print(packet_2)
    #                 # print(strAlt)
    #                 my_can.send_msg(0x18FF7080,packet_2)
    #                 my_can.send_msg(0x18FF6F80,strAlt)
    #
    #
    #         except Exception as e :
    #             print("Got exception " + str(e))

    def read_gps_2(self):
        x = UbxStream(serial.Serial(port="/dev/ttyUSB0",baudrate=9600,timeout =1))
        while True:
            x.enable_message(1,7)
            time.sleep(0.3)
            nav_pvt = x.read()
            print(nav_pvt)
            time.sleep(0.2)

            #packet 1 params
            prelat = nav_pvt.lat
            prelon = nav_pvt.lon

            # lat = int(((prelat / 10000000) - 210) * 1000000)
            # lon =int(((prelon / 10000000) - 210) * 1000000)
            #
            lat = ((prelat / 10000000) + 210 ) * 1000000
            lon = ((prelon / 10000000) + 210 ) * 1000000

            # print(lat/10,lon/10)

            #packet 2 params
            hAcc = (nav_pvt.hAcc / 1000) + 0.5
            vAcc = (nav_pvt.vAcc / 1000 ) + 0.5
            pDop = nav_pvt.pDOP
            headMot = nav_pvt.headMot / 1000



            #packet 3 params
            numSV = nav_pvt.numSV
            flags = nav_pvt.flags
            valid = nav_pvt.valid
            fixType = nav_pvt.fixType
            height = nav_pvt.height




            #packet 4 params
            iTOW = nav_pvt.iTOW
            gSpeed = nav_pvt.gSpeed

            print(iTOW)
            print(gSpeed)


            strLat = self.int2Hex(int(lat),0)
            strLong = self.int2Hex(int(lon),0)

            vehicle_gps = strLat + strLong
            # print(vehicle_gps)
            # my_can.send_msg(0x18FEF380,vehicle_gps)

            strHeading = self.int2Hex(int(headMot),2)
            strHAcc = self.int2Hex(int(hAcc),2)
            strVAcc = self.int2Hex(int(vAcc),2)
            strPdop = self.int2Hex(pDop,2)

            vehicle_acc = strHAcc + strVAcc + strHeading + strPdop
            # print(vehicle_acc)
            # my_can.send_msg(0x18FF7080,vehicle_acc)


            strAlt = self.int2Hex(height,0)
            strAlt.insert(0,fixType)
            strAlt.insert(0,valid)
            strAlt.insert(0,flags)
            strAlt.insert(0,numSV)
            # my_can.send_msg(0x18FF6F80,strAlt)

            strTOW = self.int2Hex(iTOW,0)
            strGspeed = self.int2Hex(gSpeed,0)
            vehicle_speed = strTOW + strGspeed
            # print(vehicle_speed)
            my_can.send_msg(0x18FF6E80,vehicle_speed)




        # nav_pvt.lat
        # print(x.read())
        # print(nav_pvt.lat)



if __name__ == "__main__":
    my_can = can_mtlt('can0', 'socketcan')

    # get messages
    # my_can.get_fw_version()
    # my_can.get_id()
    # my_can.get_hw_status()
    # my_can.get_sw_status()
    # my_can.get_sensor_status()

    #set messages
    # my_can.save_configuration()
    # my_can.reset_algorithm()
    my_can.set_odr(1)
    # my_can.mag_alignment_start()
    # my.set_pkt_type(7)
    # my_can.set_lpf_filter(25,5)
    # my_can.set_orientation(9)
    # my_can.read_gps()
    #my_can.read_gps_2()
    # time.sleep(0.5)

    #print and save messages of slope, acc, rate
    my_can.start_record()
