'''
version 1.1.0 in Aceinna
mtlt products(MTLT305D and 300RI included now) CAN Bus read and send message module
Requires PI3B and CAN_HAT(shopping link: https://m.tb.cn/h.eRtgNe2)
with H/L of CAN_HAT connected with H/L from sensor side
follow http://www.waveshare.net/wiki/RS485_CAN_HAT 
or follow blog at http://skpang.co.uk/blog/archives/1220
only store the angle, acc, rate to can_data.txt
running in hardbyte-python-can foler, downloaded from https://bitbucket.org/hardbyte/python-can/get/4085cffd2519.zip
@author: cek
'''

import os
import sys
import can
import time
import threading

from queue import Queue #onlu python3 supported now 


#j1939 with extended id
class can_mtlt: 
    def __init__(self, chn = 'can0', bus_type = 'socketcan_ctypes'):   
        if os.sys.platform.startswith('lin'):    
            os.system('sudo /sbin/ip link set can0 up type can bitrate 250000')  # run this command first with correct baud rate
        else:
            print('not linux system, pls running on linux system')
        self.can0 = can.interface.Bus(channel = chn, bustype = bus_type)  # socketcan_native
         
        self.msg_queue = Queue(1000)  
        self.slopedata = Queue(1000)
        self.acceldata = Queue(1000)
        self.ratedate = Queue(1000)   
        self.addressclaim = Queue(1000) 

        self.fw_version_msg_queue = Queue(1)
        self.id_msg_queue = Queue(1)
        self.hw_bit_msg_queue = Queue(1)
        self.sw_bit_msg_queue = Queue(1)
        self.sensor_status_msg_queue = Queue(1) 

        self.pdu_dict = {}   

        self.thread_put = threading.Thread(target=self.put_msg)
        self.thread_read = threading.Thread(target=self.parse_msg)
    def calc_slope(self,msg):   
        '''
        unit: degree
        '''
        pitch_uint = msg.data[0] + 256 * msg.data[1] +  65536 * msg.data[2]
        roll_uint = msg.data[3] + 256 * msg.data[4] +  65536 * msg.data[5]
        pitch = pitch_uint * (1/32768) - 250.0
        roll = roll_uint * (1/32768) - 250.0
        # put parsed data to queue
        if self.slopedata.qsize() == 1000:
            self.slopedata.queue.clear()
        self.slopedata.put('Time: {2:18.6f} Roll: {0:6.2f} Pitch: {1:6.2f}'.format(roll,pitch,msg.timestamp))
        
    def calc_accel(self,msg):   
        '''
        unit: g
        '''    
        ax_uint = msg.data[0] + 256 * msg.data[1] 
        ay_uint = msg.data[2] + 256 * msg.data[3] 
        az_uint =  msg.data[4] + 256 * msg.data[5] 
        ax = ax_uint * (0.01) - 320.0
        ay = ay_uint * (0.01) - 320.0
        az = az_uint * (0.01) - 320.0        
        # put parsed data to queue
        if self.acceldata.qsize() == 1000:
            self.acceldata.queue.clear()
        self.acceldata.put('Time: {3:18.6f} AX  : {0:6.2f} AY   : {1:6.2f} AZ: {2:6.2f}'.format(ax,ay,az,msg.timestamp))
    def calc_rate(self,msg):     
        '''
        unit: deg/s
        '''   
        wx_uint = msg.data[0] + 256 * msg.data[1] 
        wy_uint = msg.data[2] + 256 * msg.data[3] 
        wz_uint =  msg.data[4] + 256 * msg.data[5] 
        wx = wx_uint * (1/128.0) - 250.0
        wy = wy_uint * (1/128.0) - 250.0
        wz = wz_uint * (1/128.0) - 250.0        

        # put parsed data to queue
        if self.ratedate.qsize() == 1000:
            self.ratedate.queue.clear()
        self.ratedate.put('Time: {3:18.6f} WX  : {0:6.2f} WY   : {1:6.2f} WZ: {2:6.2f}'.format(wx,wy,wz,msg.timestamp))             
    def start_record(self):        
        self.thread_put.start()
        self.thread_read.start()   
     
    def parse_msg(self):    
        while (True):
            if (self.msg_queue.not_empty):
                msg_read = self.msg_queue.get()
            else:
                msg_read = self.can0.recv()                          
            self.get_pdu_list(msg = msg_read)
            
            if self.pdu_dict["pgn"] == 61481:                
                self.calc_slope(msg_read)
            elif self.pdu_dict["pgn"] == 61482:                
                self.calc_rate(msg_read)
                
            elif self.pdu_dict["pgn"] == 61485:                
                self.calc_accel(msg_read)
                
            elif self.pdu_dict["pgn"] == 61183:   
                if self.slopedata.qsize() == 1000:
                    self.slopedata.queue.clear()                             
                self.addressclaim.put('Time: {0:18.6f} PGN:{1:>8d} Payload:{2}'.format(self.pdu_dict["time_stamp"], self.pdu_dict["pgn"], self.pdu_dict["payload"]))
                                
            elif self.pdu_dict["pgn"] == 65242:
                self.fw_version_msg_queue.put(self.pdu_dict)                
            elif self.pdu_dict["pgn"] == 64965:
                self.id_msg_queue.put(self.pdu_dict)                
            elif self.pdu_dict["pgn"] == 65362:
                self.hw_bit_msg_queue.put(self.pdu_dict)                
            elif self.pdu_dict["pgn"] == 65362:
                self.hw_bit_msg_queue.put(self.pdu_dict)                
            elif self.pdu_dict["pgn"] == 65363:
                self.sw_bit_msg_queue.put(self.pdu_dict)                
            elif self.pdu_dict["pgn"] == 65364:
                self.sensor_status_msg_queue.put(self.pdu_dict)                
            else:
                print(msg_read, "\n")      
                       
    def put_msg(self):
        while (True):    
            msg_save = self.can0.recv()
            self.msg_queue.put(msg_save)           

    def send_msg(self, id_int, data_list):  # id_int = 0x18FF5500, data_list =[128, 1, 0, 0, 0, 0, 0, 0] set ODR is 100hz
        send_msg = can.Message(arbitration_id=id_int, data=data_list, extended_id=True)
        self.can0.send(send_msg)

    def get_fw_version(self): 
        self.fw_version_msg_queue.queue.clear() 
        data = [0, 254, 218]    
        my_can.send_msg(0x18EAFF00,data)       
        return(self.fw_version_msg_queue.get())                        
    def get_id(self):
        self.id_msg_queue.queue.clear() 
        data = [0, 253, 197]    
        my_can.send_msg(0x18EAFF01,data) 
        return self.id_msg_queue.get()         
    def get_hw_status(self):   
        self.hw_bit_msg_queue.queue.clear()      
        self.start_record()
        data = [0, 255, 82]    
        my_can.send_msg(0x18EAFF02,data) 
        return self.hw_bit_msg_queue.get()       
    def get_sw_status(self):  
        self.sw_bit_msg_queue.queue.clear()       
        self.start_record()
        data = [0, 255, 83]    
        my_can.send_msg(0x18EAFF03,data)  
        return self.sw_bit_msg_queue.get()  
    def get_sensor_status(self):  
        self.sensor_status_msg_queue.queue.clear()       
        self.start_record()
        data = [0, 255, 84]    
        my_can.send_msg(0x18EAFF04,data)  
        return self.sensor_status_msg_queue.get()      
     
    def save_configuration(self):
        data = [0, 128]    
        my_can.send_msg(0x18FF5100,data)
        
    def reset_algorithm(self):
        data = [0, 128]    
        my_can.send_msg(0x18FF5000,data) 
        
    def set_odr(self,odr_int):
        data = [128, odr_int]    
        my_can.send_msg(0x18FF5500,data)     
         
    def set_pkt_type(self,type_int):
        data = [128, type_int]    
        my_can.send_msg(0x18FF5600,data)  
         
    def set_lpf_filter(self,rate_int,acc_int):
        data = [128, rate_int, acc_int]    
        my_can.send_msg(0x18FF5700,data) 
        
    def set_orientation(self,value_int):
        value_msb = 0xFF00 & value_int #get the msb value of certain number
        value_msb = value_msb >> 8
        value_lsb = 0x00FF & value_int
        data = [128, value_msb, value_lsb]    
        my_can.send_msg(0x18FF5800,data)  
        
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
        self.pdu_dict["payload"] = ''.join(msg_list[4:])        
        msg_dict = self.pdu_dict
        return msg_dict                      

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
    my_can.set_odr(1)
    # my.set_pkt_type(7)
    # my_can.set_lpf_filter(25,5)
    # my_can.set_orientation(9)

    # time.sleep(0.5)

    #print and save messages of slope, acc, rate    
    my_can.start_record() #start put msgs to queue and parse the msgs

    with open('can_data.txt', 'w') as f:  # empty the can_data.txt if it exist
        pass

    while True:
        slope_data,accel_data,rate_data = my_can.slopedata.get(),my_can.acceldata.get(),my_can.ratedate.get()  
        print('{0}\n{1}\n{2}'.format(slope_data,accel_data,rate_data))
        with open('can_data.txt', 'a') as f:  
            f.write(slope_data + '\n' + accel_data + '\n' + rate_data + '\n')
        if not my_can.addressclaim.empty():
            addressclaim = my_can.addressclaim.get()
            print(addressclaim)
            with open('can_data.txt', 'a') as f:  
                f.write(addressclaim + '\n')

   

    



    



      

