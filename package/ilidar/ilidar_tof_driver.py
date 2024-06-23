##################################################################
# iLidar-ToF data retrieval via UDP
# Author: Youngbin Son, Jiho Ryoo
# Date: 2024.06.19
# Description: DCS Image logging driver
#              Triggering LiDAR is removed, and receiving time stamp of sampling time is added.
#              Sensor @ 192.168.5.x
#              Host   @ 192.168.5.2
#
#   EXP mode data packet structure [1298 B] 
# - STX	                                                [ 2 B] 	(0)
# - DCSnum (2 bit) | frame (6 bit) | row (8 bit)		[ 2 B]	(1)
# - timestamp_us (64 bit)								[ 8 B]	(2~5)
# - empty (6 bit)  | modfreq (10 bit)					[ 2 B]	(6)
# - empty (6 bit)  | shutter (10 bit)	                [ 2 B]	(7)
# - DATA												[640 x 2 = 1280 B]	(8~647)
# - ETX													[ 2 B]	(648)
#
##################################################################
# iLidar-ToF data retrieval via UDP
import socket	# for data retrieval
import struct	# for bytes -> int conversion
import numpy as np
import time

# If main is executed, run the following code
# from matplotlib import pyplot as plt
# import cv2

class iLidarDriver():
    def __init__(self):
        #### Parameters
        # Image information
        self.img_width   = 320
        self.img_height  = 160
        self.packet_size = 1298
        self.row_unpack_format = '<' + 'H' * self.img_width # '<': little endian, 'H': unsigned short
        
        # UDP connection
        self.ip_host   = '192.168.5.2'
        self.ip_sensor = '192.168.5.73'
        self.port_dat_host = 7256
        self.port_cmd_host = 7257
        self.port_cmd_snsr = 4906
        self.data_size = 2000

        self.MOD_FREQ = 0x07FF
        self.DCS_NUM  = 0xC000
        self.SHUTTER  = 0x07FF
        self.FRAME    = 0x3F00
        self.ROW      = 0x00FF

        #### Initialization
        # Create data socket
        self.sockDat = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_data_address = (self.ip_host, self.port_dat_host)
        self.sockDat.bind(recv_data_address)
        # self.sockDat.settimeout(1.0)
        self.sockDat.settimeout(0.1)    # For video

        # Create command socket
        self.sockCmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_cmd_address = (self.ip_host, self.port_cmd_host)
        self.sockCmd.bind(recv_cmd_address)
        self.sockCmd.settimeout(0.1)

    def __del__(self):
        try:
            self.sockDat.close()
            self.sockCmd.close()
        except:
            pass

    def iLidar_cmd(self, command):
        # Comm Packet:c
        # [0xAA][0xBB][0xCC][0xDD][0xEE][0xFF][0x00][Data]
        cmd = bytearray([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00])
        cmd += bytes([command])
        self.sockCmd.sendto(cmd, (self.ip_sensor, self.port_cmd_snsr))


    # Socket: Receive a packet ################################################################
    def iLidar_getImg(self) -> tuple: # tuple[numpy.uint16, numpy.uint16, int, int, int]
        # Initialize image buffers
        dcs0 = np.zeros((self.img_height, self.img_width), dtype=np.uint16)
        dcs1 = np.zeros((self.img_height, self.img_width), dtype=np.uint16)
        dcs2 = np.zeros((self.img_height, self.img_width), dtype=np.uint16)
        dcs3 = np.zeros((self.img_height, self.img_width), dtype=np.uint16)
        dcs  = [dcs0, dcs1, dcs2, dcs3]

        lv = np.array([0, 1, 2, 2, 3, 3, 3, 3])

        isFailFlg = 0
        failFlg = 0

        # Socket: Receive a packet ################################################################
        while 1:
            # Accept only if the packet is appropriately sized
            try:
                data, sender = self.sockDat.recvfrom(self.data_size)
                isFailFlg = 1
            except:
                # Connection Error
                if isFailFlg == 0:
                    return dcs, 1
                # Not enough packet is received
                else:
                    return dcs, 1
                
            
            # If received packet is data packet, ignore
            if len(data) == 10:
                pass
            
            # If received packet is image packet,
            if len(data) == self.packet_size:
                ## Information packet
                stx = data[0:2]
                packet_id = struct.unpack('<H', data[2:4])[0]
                timestamp_us = struct.unpack('<Q', data[4:12])[0]
                modfreq_shutter = struct.unpack('<H', data[12:14])[0]
                frame_row = struct.unpack('<H', data[14:16])[0]
                data_payload = data[16:1296]
                etx = data[1296:1298]
                
                modfreq = modfreq_shutter & self.MOD_FREQ
                dcs_num = (packet_id & self.DCS_NUM) >> 14
                shutter = modfreq_shutter & self.SHUTTER
                frame = (packet_id & self.FRAME) >> 8
                cur_row = frame_row & self.ROW

                print(f"dcs_num: {dcs_num}, frame: {frame}, cur_row: {cur_row}, timestamp: {timestamp_us}")

                # Unpack the data payload
                for y in range(2):
                    start_index = y * self.img_width * 2
                    end_index = start_index + self.img_width * 2
                    pixels_row = struct.unpack(self.row_unpack_format, data_payload[start_index:end_index])
                    dcs[dcs_num][cur_row + y] = np.array(pixels_row, dtype=np.uint16)

                return dcs, failFlg
                    
        


if __name__ == '__main__':
    ilidar = iLidarDriver()
    dcs, failFlg = ilidar.iLidar_getImg()
    # time.sleep(0.1)
    dcs, failFlg = ilidar.iLidar_getImg()
    # time.sleep(0.1)
    dcs, failFlg = ilidar.iLidar_getImg()
