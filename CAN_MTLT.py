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
import requests
if sys.version_info[0] > 2:
    from queue import Queue
else:
    from Queue import Queue

#j1939 with extended id
class can_mtlt:
    def __init__(self, chn = 'can0', bus_type = 'socketcan_ctypes'):
        os.system('sudo /sbin/ip link set can0 up type can bitrate 250000')  # run this command first with correct baud rate
        self.can0 = can.interface.Bus(channel = chn, bustype = bus_type)  # socketcan_native
        with open('can_data.txt', 'w') as f:  # empty the can_data.txt
            print("")
        self.msg_queue = Queue()
        self.fw_version_msg_queue = Queue()
        self.id_msg_queue = Queue()
        self.hw_bit_msg_queue = Queue()
        self.sw_bit_msg_queue = Queue()
        self.sensor_status_msg_queue = Queue()

        # self.gps_direction = {
        #     "compassBearing":0,
        #     "navSpeed":0,
        #     "pitch":0,
        #     "altitude":0
        # }
        # self.gps_wheel_speed = {
        #     "frontAxleSpeed":0,
        #     "relSpeedFrontAxleLeftWheel":0,
        #     "relSpeedFrontAxleRightWheel":0,
        #     "relSpeedRearAxle1LeftWheel":0,
        #     "relSpeedRearAxle1RigthWheel":0,
        #     "relSpeedRearAxle2LeftWheel":0,
        #     "relSpeedRearAxle2RigthWheel":0
        # }
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

        #to be done
        # self.gps_direction["pitch"] = pitch
        # self.gps_direction["altitude"] = roll
        # still need to figure out how to get compass bearing and navspeed

        print('Time: {2:18.6f} Roll: {0:6.2f} Pitch: {1:6.2f}'.format(roll,pitch,msg.timestamp))
        with open('can_data.txt', 'a') as f:
            f.write('Roll: {0:6.2f} Pitch: {1:6.2f}'.format(roll,pitch) + "\n")
    def print_accel(self,msg):   # unit: g
        ax_uint = msg.data[0] + 256 * msg.data[1]
        ay_uint = msg.data[2] + 256 * msg.data[3]
        az_uint =  msg.data[4] + 256 * msg.data[5]
        ax = ax_uint * (0.01) - 320.0
        ay = ay_uint * (0.01) - 320.0
        az = az_uint * (0.01) - 320.0
        print('Time: {3:18.6f} AX  : {0:6.2f} AY   : {1:6.2f} AZ: {2:6.2f}'.format(ax,ay,az,msg.timestamp))
        with open('can_data.txt', 'a') as f:
            f.write('AX  : {0:6.2f} AY   : {1:6.2f} AZ: {2:6.2f}'.format(ax,ay,az) + "\n")
    def print_rate(self,msg):    # degree per second
        wx_uint = msg.data[0] + 256 * msg.data[1]
        wy_uint = msg.data[2] + 256 * msg.data[3]
        wz_uint =  msg.data[4] + 256 * msg.data[5]
        wx = wx_uint * (1/128.0) - 250.0
        wy = wy_uint * (1/128.0) - 250.0
        wz = wz_uint * (1/128.0) - 250.0
        print('Time: {3:18.6f} WX  : {0:6.2f} WY   : {1:6.2f} WZ: {2:6.2f}'.format(wx,wy,wz,msg.timestamp))
        with open('can_data.txt', 'a') as f:
            f.write('WX  : {0:6.2f} WY   : {1:6.2f} WZ: {2:6.2f}'.format(wx,wy,wz) + "\n")

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
            #print to screen and can_data.txt file
            if self.pdu_dict["pgn"] == 65267:
                with open('can_data.txt', 'a') as f:  #store the raw message to file
                    f.write(msg_read.__str__() + "  ")
                self.print_gps(msg_read)
                print('gps detected')
            elif self.pdu_dict["pgn"] == 61481:
                with open('can_data.txt', 'a') as f:  #store the raw message to file
                    f.write(msg_read.__str__() + "  ")
                self.print_slope(msg_read)
            elif self.pdu_dict["pgn"] == 61482:
                with open('can_data.txt', 'a') as f:  #store the raw message to file
                    f.write(msg_read.__str__() + "  ")
                self.print_rate(msg_read)
                # pass
            elif self.pdu_dict["pgn"] == 61485:
                with open('can_data.txt', 'a') as f:  #store the raw message to file
                    f.write(msg_read.__str__() + "  ")
                self.print_accel(msg_read)
                # pass
            elif self.pdu_dict["pgn"] == 61183:
                # pass
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
        send_msg = can.Message(arbitration_id=id_int, data=data_list, extended_id=True)
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
            list = msg_list[5:]
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
        self.pdu_dict["time_stamp"] = float(msg_list[0])
        self.pdu_dict["id"] = int(msg_list[1],16)
        self.pdu_dict["extended_id"] = msg.id_type
        self.pdu_dict["priority"] = int(msg_list[2],2)
        self.pdu_dict["src"] = 0x000000FF & int(msg_list[1],16)
        self.pdu_dict["dlc"] = int(msg_list[3],10)
        self.pdu_dict["pgn"] = (0x00FFFF00 & int(msg_list[1],16)) >> 8
        self.pdu_dict["payload"] = msg_list[4:]
        msg_dict = self.pdu_dict
        return msg_dict

    def read_gps(self):
        session = gps.gps("localhost", "2947")
        session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

        while True:
            rep = session.next()
            # print(test_gps)
            try :
                if (rep["class"] == "TPV") :
                    # print(rep)
                    lat = int(rep.lat + 210)
                    lon = int(rep.lon + 210)
                    data = [lat, lon]
                    my_can.send_msg(0x18FEF300,data)
                    msg_read = self.can0.recv()

                 	# print(str(rep.lat) + "," + str(rep.lon))

            except Exception as e :
                print("Got exception " + str(e))


if __name__ == "__main__":
    my_can = can_mtlt('can0', 'socketcan_ctypes')

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
    my_can.read_gps()
    # time.sleep(0.5)

    #print and save messages of slope, acc, rate
    my_can.start_record()
