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
4. To install all the requried dependencies
```
pip install requirement.txt

```
4. Create .env in the root foler
    - Using terminal
    ```
    touch .env

    ```
6. Open the .env file and copy paste the azure credentials
    - You can use your own account's credential or the one provided by Aceinna.

5. If user intend to forward GPS data to the unit    
     ```
    python3 CAN_MTLT_2.py [port name] [baurate] -f 

    ```
    port name = USB port of the GPS receiver(eg. /dev/ttyUSB)
    baudrate = Baudrate of the unit (eg. 115200)
    -f  = indicator to forward the data. 

6. To upload the file to the cloud
    ```
    python3 CAN_MTLT_2.py -u
        
    ```
    File can be uploaded to cloud by pressing "q" and "enter", anytime in the operation. User may have to terminate the process again after sending it to the cloud as it will be still recording the data on local.

7. To just log data into the file on local
    ```
    python3 CAN_MTLT_2.py 

    ```
8. To foward data and upload the data to cloud in same process, open new terminal
    ```
    python3 CAN_MTLT_2.py [port name] [baurate] -f -u

    ```    
7. Debug steps to confirm if GPS receiver is sending data to the unit.
    - Install GPSD
    - Check  /dev/ttyS0  for data
    - ``` cgps ``` to confirm the incoming data
    - Please following the instruction in following link for detailed installation
    - https://medium.com/@DefCon_007/using-a-gps-module-neo-7m-with-raspberry-pi-3-45100bc0bb41
