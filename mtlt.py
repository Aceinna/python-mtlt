'''
MTLT Can Bus Verification Module
Requires PI2 and Cooperhill Tech PiCAN board
@author: m5horton
'''

# follow blog at http://skpang.co.uk/blog/archives/1220
# run this command first with correct baud rate as last paramater:
# sudo /sbin/ip link set can0 up type can bitrate 250000

import can
import threading
import time
from can.protocols import j1939

class Grab380CANData:

    def __init__(self):
        self.accel_pkt_cnt = 0
        self.gyro_pkt_cnt = 0
        self.slope_pkt_cnt = 0
        # set up Bus
        self.bus = j1939.Bus(channel='can0', bustype='socketcan_native')
        # set up Notifier
        self.notifier = can.Notifier(self.bus, [self.general_message])

    def increment_odr(self,msg):
        if msg.pgn == 61481:
            self.slope_pkt_cnt += 1
        elif msg.pgn == 61482:
            self.gyro_pkt_cnt += 1
        elif msg.pgn == 61485:
            self.accel_pkt_cnt += 1
    
    def general_message(self,msg):
        # print(msg)
        if msg.pgn == 65365:
            print('Packet Rate Divider Received')
            print(msg)
        elif msg.pgn == 65242:
            print('Version Command')
            print(msg)
        elif msg.pgn == 64965:
            print('ECU Command')
            print(msg)
        elif msg.pgn == 65362:
            print('Test HW')
            print(msg)
        elif msg.pgn == 65363:
            print('Test SW')
            print(msg)
        elif msg.pgn == 65364:
            print('Test Status')
            print(msg)
        elif msg.pgn == 65366:
            print('Data Packet Type')
            print(msg)
        elif msg.pgn == 65361:
            print('Save Settings')
            print(msg)
        elif msg.pgn == 65368:
            print('Set Orientation')
            print(msg)

    def print_slope(self,msg):
        if msg.pgn == 61481:
            pitch_uint = msg.data[0] + 256 * msg.data[1] +  65536 * msg.data[2]
            roll_uint = msg.data[3] + 256 * msg.data[4] +  65536 * msg.data[5]
            pitch = pitch_uint * (1/32768) - 250.0
            roll = roll_uint * (1/32768) - 250.0
            print('Roll: {0:3.2f} Pitch: {1:3.2f}'.format(roll,pitch))

    def print_accel(self,msg):
        if msg.pgn == 61485:
            ax_uint = msg.data[0] + 256 * msg.data[1] 
            ay_uint = msg.data[2] + 256 * msg.data[3] 
            az_uint =  msg.data[4] + 256 * msg.data[5] 
            ax = ax_uint * (0.01) - 320.0
            ay = ay_uint * (0.01) - 320.0
            az = az_uint * (0.01) - 320.0
            print('AX: {0:3.2f} AY: {1:3.2f} AZ: {2:3.2f}'.format(ax,ay,az))

    def print_rate(self,msg):
         if msg.pgn == 61482:
            wx_uint = msg.data[0] + 256 * msg.data[1] 
            wy_uint = msg.data[2] + 256 * msg.data[3] 
            wz_uint =  msg.data[4] + 256 * msg.data[5] 
            wx = wx_uint * (1/128.0) - 250.0
            wy = wy_uint * (1/128.0) - 250.0
            wz = wz_uint * (1/128.0) - 250.0
            print('WX: {0:3.2f} WY: {1:3.2f} WZ: {2:3.2f}'.format(wx,wy,wz))

    def print_packet_type(self,msg):
        if msg.pgn == 61481:
            print('Slope PKT Received')
        elif msg.pgn == 61482:
            print('Gyro PKT Received')
        elif msg.pgn == 61485:
            print('Accel PKT Received')

    def test_odr(self):
        threading.Thread(target=self.compute_odr).start()
        self.notifier.listeners.append(self.increment_odr)

    def compute_odr(self):
        time.sleep(10)
        self.bus.shutdown()
        accel_odr = self.accel_pkt_cnt / 10
        gyro_odr = self.gyro_pkt_cnt / 10
        slope_odr = self.slope_pkt_cnt / 10
        print('Accel Odr {0:3.1f}'.format(accel_odr))
        print('Gyro Odr {0:3.1f}'.format(gyro_odr))
        print('Slope Odr {0:3.1f}'.format(slope_odr))

    def set_odr(self,odr_setting):
        #arbitration_id = j1939.ArbitrationID(priority=6, pgn=65365, source_address=128)
        arbitration_id = j1939.ArbitrationID(priority=6, pgn=65365)
        m = j1939.PDU(arbitration_id=arbitration_id, data=[128, odr_setting])
        self.bus.send(m)

    def set_odr_mod(self,odr_setting):
        #arbitration_id = j1939.ArbitrationID(priority=6, pgn=65365, source_address=128)
        arbitration_id = j1939.ArbitrationID(priority=6, pgn=65385)
        m = j1939.PDU(arbitration_id=arbitration_id, data=[128, odr_setting])
        self.bus.send(m)

    def mod_bank0(self):
        # algorithm reset, save config, hardware bits, software bits, status, packet rate, packet type, digital filter]
        # original array
        # pdus_array = [ 80, 81, 82, 83, 84, 85, 86, 87]
        pdus_array = [ 100, 101, 102, 103, 104, 105, 106, 107]
        arbitration_id = j1939.ArbitrationID(priority=6, pgn=65520)
        m = j1939.PDU(arbitration_id=arbitration_id, data=pdus_array)
        self.bus.send(m)

    def save_settings(self):
        arbitration_id = j1939.ArbitrationID(priority=6, pgn=65361)
        m = j1939.PDU(arbitration_id=arbitration_id, data=[0, 128])
        print(m)
        self.bus.send(m)

    def get_config(self, pgn):
        pdu_format = 0xFF00 & pgn
        pdu_format = pdu_format >> 8
        pdu_specific = 0x00FF & pgn
        print(pdu_format)
        print(pdu_specific)
        arbitration_id = j1939.ArbitrationID(priority=6, pgn=60159)
        m = j1939.PDU(arbitration_id=arbitration_id, data=[24,pdu_format,pdu_specific])
        self.bus.send(m)

    def stream_attitude(self):
        self.notifier.listeners.append(self.print_slope)

    def stream_acceleration(self):
        self.notifier.listeners.append(self.print_accel)

    def stream_rate(self):
        self.notifier.listeners.append(self.print_rate)
        
    def stream_packet_types(self):
        self.notifier.listeners.append(self.print_packet_type)

    def set_orientation(self,value):
        arbitration_id = j1939.ArbitrationID(priority=6, pgn=65368)
        value_msb = 0xFF00 & value
        value_msb = value_msb >> 8
        value_lsb = 0x00FF & value
        m = j1939.PDU(arbitration_id=arbitration_id, data=[128, value_msb, value_lsb])
        print(m)
        self.bus.send(m)

    def set_packet_types(self,value):
        '''value bit 1 - slope, bit 2 - angular rate, bit 3 - accelerometer'''
        arbitration_id = j1939.ArbitrationID(priority=6, pgn=65366, source_address=128)
        m = j1939.PDU(arbitration_id=arbitration_id, data=[128, value])
        self.bus.send(m)

    def set_lpf_filter(self, gyro_hz, accel_hz):
        '''value bit 1 - slope, bit 2 - angular rate, bit 3 - accelerometer'''
        arbitration_id = j1939.ArbitrationID(priority=6, pgn=65367, source_address=128)
        m = j1939.PDU(arbitration_id=arbitration_id, data=[128, gyro_hz, accel_hz])
        self.bus.send(m)

    def get_config_loop(self,pgn):
        for i in range(0,5):
            grab.get_config(pgn)
            time.sleep(0.05)


if __name__ == "__main__":
    grab = Grab380CANData()
    #grab.set_packet_types(6)
    #grab.set_odr(2)
    #time.sleep(0.05)
    #grab.set_odr(0)
    #grab.set_lpf_filter(50,25)
    #grab.test_odr()
    #grab.set_orientation(9)
    #grab.save_settings()
    #grab.mod_bank0()
    #time.sleep(.05)
    #grab.set_odr_mod(2)
    #grab.set_odr(1)
    #grab.get_config_loop(pgn=65242)
    #grab.test_odr()
    #grab.stream_packet_types()
    grab.stream_attitude()
    #grab.stream_acceleration()
    
