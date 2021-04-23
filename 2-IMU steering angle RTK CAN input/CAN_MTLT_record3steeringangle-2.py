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
import subprocess
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
import subprocess, signal
from ubx import UbxStream

from azure.storage.blob import AppendBlobService
from azure.storage.blob import ContentSettings
from azure.storage.blob import BlockBlobService

from pymongo import MongoClient
from dotenv import load_dotenv
load_dotenv()
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

        self.filename = 'OpenIMU330BI-'
        self.filename += time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()) + '.txt'

        self.fname = './data/'
        self.fname += self.filename
        fmode = 'wb'
        #with open('can_data.txt', 'w') as f:  # empty the can_data.txt
        with open(self.fname, 'w') as f:  # empty the can_data.txt
            # f.write("time,roll,pitch,xRate,yRate,zRate,xAccel,yAccel,zAccel,lat,long,gps_yaw,gps_pitch,gps_roll,cog,sog"+ "\n")
            # f.write("time,roll,pitch,wx,wy,wz,lat,long,cog,sog,yaw_gps,pitch_gps,roll_gps,ax,ay,az"+ "\n")
            f.write("time, steering_angle(deg), lat, lon"+ "\n")
            print('')
        self.msg_queue = Queue()
        self.fw_version_msg_queue = Queue()
        self.id_msg_queue = Queue()
        self.hw_bit_msg_queue = Queue()
        self.sw_bit_msg_queue = Queue()
        self.sensor_status_msg_queue = Queue()
        self.test_queue = {
            "timeStamp": 0,
            "pgn_dict": {}
            }
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

        self.lat2 = 0 
        self.long2 = 0 
        self.steering_angle = 0
    def print_slope(self,msg):   # unit: degree
        pitch_uint = msg.data[0] + 256 * msg.data[1] +  65536 * msg.data[2]
        roll_uint = msg.data[3] + 256 * msg.data[4] +  65536 * msg.data[5]
        pitch = pitch_uint * (1/32768) - 250.0
        roll = roll_uint * (1/32768) - 250.0

        print('Time: {2:18.6f} Roll: {0:6.2f} Pitch: {1:6.2f}'.format(roll,pitch,msg.timestamp))

        return [roll,pitch]

    def print_accel(self,msg):   # unit: g
        ax_ay_az = struct.unpack('<HHHH', msg.data)
        ax = ax_ay_az[0] * (0.01) - 320.0
        ay = ax_ay_az[1] * (0.01) - 320.0
        az = ax_ay_az[2] * (0.01) - 320.0
        return [ax,ay,az]
    
    def print_steering_angle(self, msg):   # unit: deg
        # print(msg.data.hex())  # struct.unpack(strformat, bytes.fromhex(strhex))  
        angle = struct.unpack('<H', bytes.fromhex(msg[0:4]))
        val_angle = angle[0] * (1/256) - 125
        return [val_angle]

    def parse_gps_packet_1(self, msg):
        # deg  
        lat_long = struct.unpack('<II',  bytes.fromhex(msg))
        lat = (lat_long[0] * 0.0000001) - 210
        long = (lat_long[1] * 0.0000001) - 210
        return [lat,long]

    def parse_gps_packet_3(self,dlc): 
        sida_yaw_pitch_roll_rsvd = struct.unpack('<BHHHB', dlc.data) 
        yaw = (sida_yaw_pitch_roll_rsvd[1] * 0.0001) - 3.14159
        pitch = (sida_yaw_pitch_roll_rsvd[2] * 0.0001) - 3.14159
        roll = (sida_yaw_pitch_roll_rsvd[3] * 0.0001) - 3.14159

        return [yaw,pitch,roll]

    def parse_gps_packet_2(self,dlc):
        # empty = ""
        # cog_1 = [str(dlc.data[3]),str(dlc.data[2])]
        # sog_1 = [str(dlc.data[5]),str(dlc.data[4])]

        # cog_1_join = empty.join(cog_1)
        # sog_1_join = empty.join(sog_1)

        # cog_new = int(cog_1_join,16)
        # sog_new = int(sog_1_join,16)

        rapid_course = struct.unpack('<HHHH', dlc.data) 

        cog = (rapid_course[1] * 0.0001) - 3.14
        sog = (rapid_course[2] * 0.0001)

        return [cog,sog]

    def print_rate(self,msg):    # degree per second
        wx_wy_wz = struct.unpack('<HHHH', msg.data)
        wx = wx_wy_wz[0] * (1/128.0) - 250.0
        wy = wx_wy_wz[1] * (1/128.0) - 250.0
        wz = wx_wy_wz[2] * (1/128.0) - 250.0

        return [wx,wy,wz]

    def start_record(self):
        self.thread_put.start()
        self.thread_read.start()
        # sys.stdout.write('\nPress q and enter to upload file to the azzure.. ')
        choice = input().lower()
        if choice == 'q':
        # if True:
            self.upload_drive_test_azzure()
            sys.stdout.write('Now press Cntrl + C to terminate')

    def read_msg(self):    # print all messages
        while (True):
            if (self.msg_queue.not_empty):
                msg_read = self.msg_queue.get()
            else:
                msg_read = self.can0.recv()

            self.get_pdu_list(msg = msg_read)

            # self.test_queue["pgn_dict"].update({ self.pdu_dict["pgn"] : msg_read})
            # print(msg_read, self.pdu_dict)
            self.write_msg(self.pdu_dict)

            # if int(self.pdu_dict["time_stamp"]) != self.test_queue["timeStamp"]:
            #     self.write_msg(self.test_queue)
            #     self.test_queue["timeStamp"] = int(self.pdu_dict["time_stamp"])
            #     self.test_queue["pgn_dict"] = {}
            # else:
            #     if self.pdu_dict == 63489 or self.pdu_dict == 63490 or self.pdu_dict == 61721:
            #         self.test_queue["pgn_dict"].update({ self.pdu_dict["pgn"] : self.pdu_dict["dlc"]})
            #     else:
            #         self.test_queue["pgn_dict"].update({ self.pdu_dict["pgn"] : msg_read})

            # print(self.test_queue)


            # if self.pdu_dict["pgn"] == 61481:
            #     self.print_slope(msg_read)
            # elif self.pdu_dict["pgn"] == 63489:
            #     print("long","lat")
            #     self.parse_gps_packet_1(self.pdu_dict["dlc"],msg_read)
            # elif self.pdu_dict["pgn"] == 63490:
            #     print("cog sog")
            #     self.parse_gps_packet_2(self.pdu_dict["dlc"],msg_read)
            # elif self.pdu_dict["pgn"] == 61721:
            #     print("yaw","pitch","roll")
            #     self.parse_gps_packet_3(self.pdu_dict["dlc"],msg_read)
            # elif self.pdu_dict["pgn"] == 61482:
            #     self.print_rate(msg_read)
            # elif self.pdu_dict["pgn"] == 61485:
            #     self.print_accel(msg_read)
            # elif self.pdu_dict["pgn"] == 61183:
            #     pass
            #     print("Time:", self.pdu_dict["time_stamp"], self.pdu_dict["pgn"], self.pdu_dict["payload"], "------Address_Claiming")     #only print to screen, no write to file
            # elif self.pdu_dict["pgn"] == 65242:
            #     self.fw_version_msg_queue.put(self.pdu_dict)
            #     print(self.pdu_dict)
            # elif self.pdu_dict["pgn"] == 64965:
            #     self.id_msg_queue.put(self.pdu_dict)
            #     print(self.pdu_dict)
            # elif self.pdu_dict["pgn"] == 65362:
            #     self.hw_bit_msg_queue.put(self.pdu_dict)
            #     print(self.pdu_dict)
            # elif self.pdu_dict["pgn"] == 65362:
            #     self.hw_bit_msg_queue.put(self.pdu_dict)
            #     print(self.pdu_dict)
            # elif self.pdu_dict["pgn"] == 65363:
            #     self.sw_bit_msg_queue.put(self.pdu_dict)
            #     print(self.pdu_dict)
            # elif self.pdu_dict["pgn"] == 65364:
            #     self.sensor_status_msg_queue.put(self.pdu_dict)
            #     print(self.pdu_dict)
            # else:
            #     # print( "\n")
            #     print(msg_read, "\n")


    def write_msg(self,queue):
        self.roll = 0
        self.pitch = 0
        self.wx = 0
        self.wy = 0
        self.wz = 0
        self.lat = 0
        self.long = 0
        self.cog = 0
        self.sog = 0
        self.yaw_gps = 0
        self.pitch_gps = 0
        self.roll_gps = 0
        self.ax = 0
        self.ay = 0
        self.az = 0
        # self.lat2 = 0 if self.lat2 == 0 else self.lat2
        # self.long2 = 0 if self.long2 == 0 else self.long2
        # self.steering_angle = 0

        tm = queue.get('time_stamp')
        pgn = queue.get('pgn')
        py_load = queue.get('payload')
        print(pgn, py_load, end='\r')
        
        if pgn == 65267:  #  FEF3 from PI3 Raspberry                  
            parsed_val = self.parse_gps_packet_1(py_load)
            self.lat2 = parsed_val[0] 
            self.long2 = parsed_val[1]

        elif pgn == 61451:  # F00B
            parsed_val = self.print_steering_angle(py_load)
            self.steering_angle = parsed_val[0]

        with open(self.fname, 'a') as f:
            # f.write('{0:6.4f},{1:6.4f},{2:6.4f},{3:6.4f},{4:6.4f},{5:6.4f},{6:10.7f},{7:10.7f},{8:6.4f},{9:6.4f},{10:6.4f},{11:6.4f},{12:6.4f},{13:6.4f},{14:6.4f},{15:6.4f}'.format(queue["timeStamp"],\
            #     self.roll, self.pitch, self.wx, self.wy, self.wz, self.lat, self.long, self.cog, self.sog, self.yaw_gps, \
            #     self.pitch_gps, self.roll_gps, self.ax, self.ay, self.az) + "\n")
            f.write('{0:6.4f},{1:6.4f},{2:10.7f},{3:10.7f}'.format(tm, self.steering_angle, self.lat2, self.long2) + "\n")

    def put_msg(self):
        while (True):
            msg_save = self.can0.recv()
            self.msg_queue.put(msg_save)

    def send_msg(self, id_int, data_list):  # id_int = 0x18FF5500, data_list =[128, 1, 0, 0, 0, 0, 0, 0] set ODR is 100hz
        send_msg = can.Message(arbitration_id=id_int, data=data_list, is_extended_id=True)
        cmd = 'sudo /sbin/ip -details link show can0'
        res = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
        feedback = res.stdout.read().decode('utf-8')
        # print(feedback)
        if 'BUS-OFF' not in feedback:
            self.can0.send(send_msg)
        else:
            print('bus-off now')
            cmd = 'sudo /sbin/ip link set can0 type can restart-ms 100'
            res = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

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
        # print('msg list',msg_list)
        if msg_list[3] == '19f80180'  or msg_list[3] == '19f80280' or msg_list[3] == '19f11980':
            self.pdu_dict["time_stamp"] = float(msg_list[1])
            self.pdu_dict["id"] = int(msg_list[3],16)
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
            self.pdu_dict["payload"] = ''.join(msg_list[8:16])
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

    def read_gps_2(self):
        x = UbxStream(serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout =1))
        while True:
            x.enable_message(1,7)
            time.sleep(0.3)
            nav_pvt = x.read()
            print(nav_pvt)
            time.sleep(0.2)

            #packet 1 params
            prelat = nav_pvt.lat
            prelon = nav_pvt.lon

            print(prelat)
            print(prelon)

            lat = ((prelat / 10000000) + 210 ) * 1000000
            lon = ((prelon / 10000000) + 210 ) * 1000000

            #packet 2 params
            hAcc = (nav_pvt.hAcc / 1000) + 0.5
            vAcc = (nav_pvt.vAcc / 1000 ) + 0.5
            pDop = nav_pvt.pDOP
            print (pDop)
            print('pdop')
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

            strLat = self.int2Hex(int(lat),0)
            strLong = self.int2Hex(int(lon),0)

            vehicle_gps = strLat + strLong
            print("lat:{0} long:{1} strlat:{2} strlong:{3}".format(lat, lon, strLat, strLong))
            my_can.send_msg(0x18FEF380,vehicle_gps)

            strHeading = self.int2Hex(int(headMot),2)
            strHAcc = self.int2Hex(int(hAcc),2)
            strVAcc = self.int2Hex(int(vAcc),2)
            strPdop = self.int2Hex(pDop,2)

            vehicle_acc = strHAcc + strVAcc + strPdop +strHeading
            my_can.send_msg(0x18FF7080,vehicle_acc)


            strAlt = self.int2Hex(height,0)
            strAlt.insert(0,fixType)
            strAlt.insert(0,valid)
            strAlt.insert(0,flags)
            strAlt.insert(0,numSV)
            my_can.send_msg(0x18FF6F80,strAlt)

            strTOW = self.int2Hex(iTOW,0)
            strGspeed = self.int2Hex(gSpeed,0)
            vehicle_speed = strTOW + strGspeed
            my_can.send_msg(0x18FF6E80,vehicle_speed)

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
            print(pre_nav_speed)
            nav_speed = (( pre_nav_speed * 3600) / 1000000) * 256
            print(nav_speed)

            speed= self.int2Hex(int(nav_speed),2)
            packet_speed = [0,0] + speed + [0,0] + [0,0]
            my_can.send_msg(0x18FEE880,packet_speed)

    def upload_drive_test_db(self):
        name = self.filename.replace(".txt","")
        data = {"name":name,"reference_primary":self.filename,"reference_sec":"N/A","device_primary":"OpenIMU300RI","device_secondary":"N/A" ,"graphs": False}


        db_name = os.getenv("DB_NAME")
        host = os.getenv("HOST")
        port = os.getenv("PORT")
        username = os.getenv("USERNAME")
        password = os.getenv("PASSWORD")
        # args = "ssl=true&retrywrites=false&ssl_cert_reqs=CERT_NONE"

        connection_uri = os.getenv("CONNECTION_URI")

        client = MongoClient(connection_uri)

        db = client[db_name]
        user_collection = db['drive']
        user_collection.insert_one(data)
        # for user in user_collection.find():
        #     print(user)
        # demo server
        # host_address='http://40.118.233.18:3000/'

        # local
        # host_address= 'http://localhost:3000/'

        # production
        # host_address='https://api.aceinna.com/'

        # url = host_address + "api/Drive/post"
        # data_json = json.dumps(data)
        # headers = {'Content-type': 'application/json', 'Authorization' : self.user['access_token'] }
        # response = requests.post(url, data=data_json, headers=headers)
        # response = response.json()

    def upload_drive_test_azzure(self):
        self.upload_drive_test_db()
        f = open(self.fname, "r")
        text = f.read()

        self.block_blob_service = BlockBlobService(account_name='navview',
                                                    # sas_token=self.sas_token, # account_key
                                                    account_key=os.getenv("ACCOUNT_KEY"), # account_key to be encripted
                                                    protocol='http')

        self.block_blob_service.create_blob_from_text(container_name='data-test',
                                                    blob_name=self.filename,
                                                    text=text,
                                                    content_settings=ContentSettings(content_type='text/plain'))


if __name__ == "__main__":
    my_can = can_mtlt()

    # get messages
    # my_can.get_fw_version()
    # my_can.get_id()
    # my_can.get_hw_status()
    # my_can.get_sw_status()
    # my_can.get_sensor_status()

    #set messages
    # my_can.save_configuration()
    # my_can.reset_algorithm()
    # my_can.set_odr(10)
    # my_can.mag_alignment_start()
    # my.set_pkt_type(7)
    # my_can.set_lpf_filter(25,5)
    # my_can.set_orientation(9)
    # my_can.read_gps()
    # my_can.read_gps_2()
    # my_can.read_lianshi()
    # time.sleep(0.5)

    #print and save messages of slope, acc, rate
    my_can.start_record()
