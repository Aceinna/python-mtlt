'''
version 2.0.0 in Aceinna
mtlt products(MTLT305D and 300RI included now) CAN Bus read and send message module
Requires PI3B and CAN_HAT(shopping link: https://m.tb.cn/h.eRtgNe2)
with H/L of CAN_HAT connected with H/L from sensor side
only store the angle, acc, rate to can_data.txt
@author: cek and tejas pashte
'''

# follow http://www.waveshare.net/wiki/RS485_CAN_HAT

import os
import subprocess
import sys
import can
import time
import threading
import binascii
import struct
import requests
import csv
import datetime
import serial
import subprocess, signal
from ubx import UbxStream
import glob

from azure.storage.blob import AppendBlobService
from azure.storage.blob import ContentSettings
from azure.storage.blob import BlockBlobService

from pymongo import MongoClient
from dotenv import load_dotenv
load_dotenv()

if sys.version_info[0] > 2:
    from queue import Queue
else:
    from Queue import Queue

#j1939 with extended id
class can_mtlt:
    def __init__(self, chn = 'can0', bus_type = 'socketcan'):
        """define global values and creating txt file """
        # run this command first with correct baud rate to set open the can port 
        os.system('sudo /sbin/ip link set can0 up type can bitrate 250000')  
        self.can0 = can.interface.Bus(channel = chn, bustype = bus_type)  # socketcan_native
        self.rows = []
        self.writer = ""

        # creating folder data if does not exists 
        path = './data'
        try:
            os.mkdir(path)
        except OSError:
            print ("Creation of the directory %s failed" % path)
            print ("%s already exists or storage is full" % path)
        else:
            print ("Successfully created the directory %s " % path)

        #filename with unique idefiener as date
        self.filename = 'OpenIMURI-'
        self.filename += time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()) + '.txt'

        self.fname = './data/'
        self.fname += self.filename
        fmode = 'wb'
        
        with open(self.fname, 'w') as f:  # empty the can_data.txt
            f.write("time,roll,pitch,wx,wy,wz,lat,long,cog,sog,yaw_gps,pitch_gps,roll_gps,ax,ay,az"+ "\n")
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

        # Threads to read raw data and forward gps
        self.thread_put = threading.Thread(target=self.put_msg)
        self.thread_read = threading.Thread(target=self.read_msg)
        self.thread_read_gps = threading.Thread(target=self.read_gps_2)

        

    def print_slope(self,msg):
        """ return roll and pitch for  61481 pgn """   
        pitch_uint = msg.data[0] + 256 * msg.data[1] +  65536 * msg.data[2]
        roll_uint = msg.data[3] + 256 * msg.data[4] +  65536 * msg.data[5]
        pitch = pitch_uint * (1/32768) - 250.0
        roll = roll_uint * (1/32768) - 250.0

        print('Time: {2:18.6f} Roll: {0:6.2f} Pitch: {1:6.2f}'.format(roll,pitch,msg.timestamp))

        return [roll,pitch]

    def print_accel(self,msg):   
        """ return roll and pitch for  61481 pgn """   
        ax_ay_az = struct.unpack('<HHHH', msg.data)
        ax = ax_ay_az[0] * (0.01) - 320.0
        ay = ax_ay_az[1] * (0.01) - 320.0
        az = ax_ay_az[2] * (0.01) - 320.0
        return [ax,ay,az]

    def parse_gps_packet_1(self,dlc):
        """ return lattitude and longitude for  63489 pgn """   
        lat_long = struct.unpack('<II',dlc.data)
        long = (lat_long[1] * 0.0000001) - 210
        lat = (lat_long[0] * 0.0000001) - 210
        return [lat,long]

    def parse_gps_packet_3(self,dlc):
        """ return yaw,roll and pitch for  61721 pgn """   
        sida_yaw_pitch_roll_rsvd = struct.unpack('<BHHHB', dlc.data) 
        yaw = (sida_yaw_pitch_roll_rsvd[1] * 0.0001) - 3.14159
        pitch = (sida_yaw_pitch_roll_rsvd[2] * 0.0001) - 3.14159
        roll = (sida_yaw_pitch_roll_rsvd[3] * 0.0001) - 3.14159

        return [yaw,pitch,roll]

    def parse_gps_packet_2(self,dlc):
        """ return cog and sog for  63490 pgn """   
        rapid_course = struct.unpack('<HHHH', dlc.data) 

        cog = (rapid_course[1] * 0.0001) - 3.14
        sog = (rapid_course[2] * 0.0001)

        return [cog,sog]

    def print_rate(self,msg):   
        """ return xrate,yrate,zrate for 61482 pgn """   
        wx_wy_wz = struct.unpack('<HHHH', msg.data)
        wx = wx_wy_wz[0] * (1/128.0) - 250.0
        wy = wx_wy_wz[1] * (1/128.0) - 250.0
        wz = wx_wy_wz[2] * (1/128.0) - 250.0

        return [wx,wy,wz]

    def start_record(self):
        """ 
        start put, read threads
        uploads file to azure 
        start forward data thread
        """
        self.thread_put.start()
        self.thread_read.start()

        if '-F' in sys.argv or '-f' in sys.argv:
            self.thread_read_gps.start()
        
        if  '-U' in sys.argv or '-u' in sys.argv:
            sys.stdout.write('Press q and enter to upload file to the azzure.. \n')
            sys.stdout.write('Press Cntrl + C to terminate \n')
            choice = input().lower()
            if choice == 'q':
                try:
                    self.upload_drive_test_azzure()
                    sys.stdout.write('Now press Cntrl + C to terminate')
                except:
                    print('Upload unsuccessful! Please check your network connection and start the script again.')
        else:
            print('Recording file on local. \n')   
            print('Press Cntrl + C to abort. \n')              


    def read_msg(self):
        """ returns pgn/message detail list """    
        while (True):
            if (self.msg_queue.not_empty):
                msg_read = self.msg_queue.get()
            else:
                msg_read = self.can0.recv()


            self.get_pdu_list(msg = msg_read)

            if int(self.pdu_dict["time_stamp"]) != self.test_queue["timeStamp"]:
                self.write_msg(self.test_queue)
                self.test_queue["timeStamp"] = int(self.pdu_dict["time_stamp"])
                self.test_queue["pgn_dict"] = {}
            else:
                if self.pdu_dict == 63489 or self.pdu_dict == 63490 or self.pdu_dict == 61721:
                    self.test_queue["pgn_dict"].update({ self.pdu_dict["pgn"] : self.pdu_dict["dlc"] })
                else:
                    self.test_queue["pgn_dict"].update({ self.pdu_dict["pgn"] : msg_read })

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
        """ logs message into txt file """
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

        if queue["timeStamp"] > 0 :

            for key in queue["pgn_dict"]:               
                if key == 61481:
                    parsed_val = self.print_slope(queue["pgn_dict"][key])
                    self.roll = parsed_val[0]
                    self.pitch = parsed_val[1]

                elif key == 61482:
                    parsed_val = self.print_rate(queue["pgn_dict"][key])
                    self.wx = parsed_val[0]
                    self.wy = parsed_val[1]
                    self.wz = parsed_val[2]

                elif key == 63489:                                                 
                    parsed_val = self.parse_gps_packet_1(queue["pgn_dict"][key])
                    self.lat = parsed_val[0]
                    self.long = parsed_val[1]

                elif key == 63490:
                    parsed_val = self.parse_gps_packet_2(queue["pgn_dict"][key])
                    self.cog = parsed_val[0]
                    self.sog = parsed_val[1]

                elif key == 61721:
                    parsed_val = self.parse_gps_packet_3(queue["pgn_dict"][key])
                    self.yaw_gps = parsed_val[0]
                    self.pitch_gps = parsed_val[1]
                    self.roll_gps = parsed_val[2]

                elif key == 61485:
                    parsed_val = self.print_accel(queue["pgn_dict"][key])
                    self.ax = parsed_val[0]
                    self.ay = parsed_val[1]
                    self.az = parsed_val[2]

        with open(self.fname, 'a') as f:
            f.write('{0:6.4f},{1:6.4f},{2:6.4f},{3:6.4f},{4:6.4f},{5:6.4f},{6:10.7f},{7:10.7f},{8:6.4f},{9:6.4f},{10:6.4f},{11:6.4f},{12:6.4f},{13:6.4f},{14:6.4f},{15:6.4f}'.format(queue["timeStamp"],\
                self.roll, self.pitch, self.wx, self.wy, self.wz, self.lat, self.long, self.cog, self.sog, self.yaw_gps, \
                self.pitch_gps, self.roll_gps, self.ax, self.ay, self.az) + "\n")

    def put_msg(self):
        """ reads raw message from can0 and save in the golbal list"""
        while (True):
            msg_save = self.can0.recv()
            self.msg_queue.put(msg_save)

    def send_msg(self, id_int, data_list):  # id_int = 0x18FF5500, data_list =[128, 1, 0, 0, 0, 0, 0, 0] set ODR is 100hz
        """ sends message to the unit using CAN0 interface"""
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
        """ returns firmware version"""
        data = [0, 254, 218]
        my_can.send_msg(0x18EAFF00,data)
        print("fw version:", self.fw_version_msg_queue.get())
        # return(self.fw_version_msg_queue.get())
    def get_id(self):
        """ returns unit it version"""
        data = [0, 253, 197]
        my_can.send_msg(0x18EAFF01,data)
        print("id:", self.id_msg_queue.get())

    def get_hw_status(self):
        """ returns hardware status"""
        self.start_record()
        data = [0, 255, 82]
        my_can.send_msg(0x18EAFF02,data)
        print("hw bit:", self.hw_bit_msg_queue.get())

    def get_sw_status(self):
        """ returns software status"""
        self.start_record()
        data = [0, 255, 83]
        my_can.send_msg(0x18EAFF03,data)
        print("sw bit:", self.sw_bit_msg_queue.get())

    def get_sensor_status(self):
        """ returns sensor status"""
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
        """ sets data rate and returns the set value"""
        data = [128, odr_int]
        my_can.send_msg(0x18FF5500,data)
        print("set odr:", data)

    def set_pkt_type(self,type_int):
        """ sets packet type and returns current packet type """
        data = [128, type_int]
        my_can.send_msg(0x18FF5600,data)
        print(data)

    def mag_alignment_start(self):
        """ returns magnetic alignment status """
        status = self.mag_alignment_calculate(1)
        while status != 13:
            status = self.mag_alignment_status()

        if status == 13:
            self.mag_alignment_save()

    def mag_alignment_status(self):
            status = self.mag_alignment_calculate(0)
            return status

    def mag_alignment_save(self):
        """ save value into unit for magalignment """
        sys.stdout.write('Accept values? Type y/n: ')
        choice = input().lower()
        if choice  == 'y':
            self.mag_alignment_calculate(5,True)
            print('values saved')
        elif choice == 'n':
            print('values rejected')
            return

    def mag_alignment_calculate(self,payload,calc = False):
        """ return current mag alignemnt status from the unit """
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
        """ returns lpf filter settings """
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
        """ returns new orientation"""
        value_msb = 0xFF00 & value_int #get the msb value of certain number
        value_msb = value_msb >> 8
        value_lsb = 0x00FF & value_int
        data = [128, value_msb, value_lsb]
        my_can.send_msg(0x18FF5800,data)
        print(data)

    def get_pdu_list(self, msg = None, msg_dict = None):
        """ return pgn detail list"""
        if msg == None:
            msg = self.can0.recv()
        msg_list = list(str(msg).split(" "))     # the list include: time, id, priority, DLC, data0,data1,...,data7
        msg_list = [x for x in msg_list if x != '']       # delete the empty items

        if msg_list[3] == '19f80180'  or msg_list[3] == '19f80280' or msg_list[3] == '19f11980':
            self.pdu_dict["time_stamp"] = float(msg_list[1])
            self.pdu_dict["id"] = int(msg_list[3],16)
            self.pdu_dict["dlc"] = msg_list[7:15]
            self.pdu_dict["pgn"] = (0x00FFFF00 & int(msg_list[3],16)) >> 8
            self.pdu_dict["payload"] = msg_list[7:15]
            msg_dict = self.pdu_dict
            return msg_dict
        else:
            pgn = (0x00FFFF00 & int(msg_list[3],16)) >> 8    
            self.pdu_dict["time_stamp"] = float(msg_list[1])
            self.pdu_dict["id"] = int(msg_list[3],16)
            self.pdu_dict["dlc"] = msg_list[7:15]
            self.pdu_dict["pgn"] = (0x00FFFF00 & int(msg_list[3],16)) >> 8
            self.pdu_dict["payload"] = ''.join(msg_list[7:15])

    # convert unix time to GPS time     
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

    def find_ports(self):
        """ returns connected USB ports """
        print('Looking for port with GPS reciever. Please do not disconnect..')
        # if sys.platform.startswith('win'):
        #     ports = ['COM%s' % (i + 1) for i in range(256)]
        if sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                if port.find('USB') == -1:
                    continue
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result   

    def read_gps_2(self):
        """ returns packet and forwards data for unit"""
        # port = self.find_ports()
        if sys.argv[1].find('USB') == -1:
            print('Please enter the correct port')
            return
        
        ### Use ubx stream to forward data using user defined port and baudrate       
       
        x = UbxStream(serial.Serial(port=sys.argv[1],baudrate=sys.argv[2],timeout =1))
        while True:
            print('sending data to the unit ...')
            x.enable_message(1,7)
            time.sleep(0.3)
            nav_pvt = x.read()
            time.sleep(0.2)

            #packet 1 params
            prelat = nav_pvt.lat
            prelon = nav_pvt.lon

            lat = ((prelat / 10000000) + 210 ) * 1000000
            lon = ((prelon / 10000000) + 210 ) * 1000000

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

            strLat = self.int2Hex(int(lat),0)
            strLong = self.int2Hex(int(lon),0)

            vehicle_gps = strLat + strLong
            print("lat:{0} long:{1} strlat:{2} strlong:{3}".format(lat, lon, strLat, strLong))
            my_can.send_msg(0x18FEF380,vehicle_gps)

            # convert hex value to hex  and send data for the PGN    
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

    # This is custom packet for lianshi
    def read_lianshi(self):
       
<<<<<<< HEAD
        x = UbxStream(serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout =1))
=======
        x = UbxStream(serial.Serial(port="/dev/ttyUSB0",baudrate=9600,timeout =1))
>>>>>>> 1368039a5b258cf9cf968c37cbedb6ca14444fef
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
        """ uploads data to the database """
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
    

    def upload_drive_test_azzure(self):
        """ uploads file to the bolbs """
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
    # my_can.set_odr(10)
    # if sys.argv[1] == 'record':
    #     my_can.start_record()
    # elif sys.argv[1] == 'send':
    #     my_can.read_gps_2()        
    
    # get messages
    # my_can.get_fw_version()
    # my_can.get_id()
    # my_can.get_hw_status()
    # my_can.get_sw_status()
    # my_can.get_sensor_status()

    #set messages
    # my_can.save_configuration()
    # my_can.reset_algorithm()
    my_can.set_odr(10)
    # my_can.mag_alignment_start()
    # my.set_pkt_type(7)
    # my_can.set_lpf_filter(25,5)
    # my_can.set_orientation(9)
    # my_can.read_gps()
    # my_can.read_gps_2()
    # my_can.read_lianshi()
    # time.sleep(0.5)

    #print and save messages of slope, acc, rate
<<<<<<< HEAD
    my_can.start_record()
=======
    my_can.start_record()
>>>>>>> 1368039a5b258cf9cf968c37cbedb6ca14444fef
