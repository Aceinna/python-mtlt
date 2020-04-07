## Python-Mtlt
CAN bus Drivers for dynamic tilt MTLT and OpenIMU300RI (J1939 app) inertial measurement unit, using PyCan, PiCAN2 board and Pi3/Pi4

## Dependencies
Python-can (https://github.com/hardbyte/python-can)
PiCan2 board(https://www.elektor.com/pican-2-can-bus-board-for-raspberry-pi)
Ublox M8 (https://www.u-blox.com/en/product/neolea-m8t-series)

## Tech
 < Python 3.6
Currently not all the functions are compatible with python 2.7

## Hardware Installations
- For GPS through RS-232
1. Connect Ublox through RS-232 to OpenIMU300RI
2. Mount PiCAN board on Pi3/4
3. Connect PiCAN board to  OpenIMU300RI using CANBUS port.
4. Make sure to JUMP JP3 on PiCAN board.
5. Follow soldering instruction as per the following doc for CANBUS - ( http://skpang.co.uk/catalog/images/raspberrypi/pi_2/PICAN2UG13.pdf)
6. After all the connection make sure to restart the Pi

- For GPS through CANBUS
1. Connect Ublox through USB to Pi(Use DB9 to usb converter, if micro to usb is not available)
2. Mount PiCAN board on Pi3/4
3. Connect PiCAN board to OpenIMU300RI using CANBUS port.
4. Make sure to JUMP JP3 on PiCAN board.
5. Follow soldering instruction as per the following doc for CANBUS - ( http://skpang.co.uk/catalog/images/raspberrypi/pi_2/PICAN2UG13.pdf)
6. After all the connection make sure to restart the Pi

## Software Installations
1. Run to open the can0 port on Pi.
```
sudo /sbin/ip link set can0 up type can bitrate 250000

```
2. Place CAN_MTLT.py and ubx.py inside pi-can folder cloned from above mentioned directory.
3. In some of the raspberry pi queue length for message and might give error while parsing the gps messages. In that scenario run:
```
sudo ifconfig can0 txqueuelen 1000

```
4. Download .env file and place into root folder.
5. File can be uploaded to cloud by pressing "q" and "enter", anytime in the operation. User may have to terminate the file again after sending it to the cloud as it will be still recording the data on local.

- For GPS through CANBUS
1. Uncomment my_can.read_gps_2() function and comment the rest and the script by
```
python3 CAN_MTLT.py
```
2. In new terminal tab, uncomment my_can.set_odr(1) and my_can.start_record() to log the data in can_data.txt, after that run
```
python3 CAN_MTLT.py

3. Uncomment  my_can.read_lianshi()  for PGN 65256
```
- For GPS through RS-232
1. Uncomment my_can.set_odr(1) and my_can.start_record() to log the data in can_data.txt, after that run
```
python3 CAN_MTLT.py
```

All the data will be stored in CAN_DATA.txt.
