# Dynamixel Driving Package
# Author: Jiho Ryoo
import os       
import sys
import tty
import termios
from package.dynamixel.dynamixel_sdk import *
from package.dynamixel.dynamixel_driver_def import *

class DynamixelDriver():
    def __init__(self):
        # Initialize getch
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

        # Initialize Port Handler and Packet Handler
        self.portHandler   = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        if self.portHandler.openPort():
            print("Port Open Success!")
        else:
            print("Port Open Failed!")
            print("Press any key to exit...")
            self.getch()
            quit()
        
        ## Dynamixel Connect
        # DXL1 (Pitch)
        # dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_YAW_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_YAW_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel1 Connect Success!")
        # DXL2 (Yaw)
        # dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_PIT_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_PIT_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel2 Connect Success!")

    def getch(self):
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        return ch
    
    def dxl_torque_enable(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.dxl_chk_err(dxl_comm_result, dxl_error)
    
    def dxl_torque_disable(self, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.dxl_chk_err(dxl_comm_result, dxl_error)

    def dxl_chk_err(self, dxl_comm_result, dxl_error):
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def dxl_get_pos(self, dxl_id):
        dxl_cur_pos, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, ADDR_MX_PRESENT_POSITION)
        self.dxl_chk_err(dxl_comm_result, dxl_error)
        return dxl_cur_pos
    
    def dxl_get_spd(self, dxl_id):
        dxl_cur_spd, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, ADDR_MX_PRESENT_SPEED)
        self.dxl_chk_err(dxl_comm_result, dxl_error)
        return dxl_cur_spd
    
    def dxl_set_pos(self, dxl_id, tar_stp, tar_spd):
        self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_MX_GOAL_SPEED, tar_spd)
        self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_MX_GOAL_POSITION, tar_stp)

    def dxl_multi_get_pos(self):
        dxl_pit_cur_pos = self.dxl_get_pos(DXL_PIT_ID)
        dxl_yaw_cur_pos = self.dxl_get_pos(DXL_YAW_ID)
        return dxl_pit_cur_pos, dxl_yaw_cur_pos

    def dxl_multi_get_spd(self, sign):
        dxl_pit_cur_spd = self.dxl_get_spd(DXL_PIT_ID)
        dxl_yaw_cur_spd = self.dxl_get_spd(DXL_YAW_ID)

        # Unit Conversion: dynamixelStep => deg/s
        dxl_pit_spd = (dxl_pit_cur_spd & 0x3FF) * RPMCONSTANT
        dxl_yaw_spd = (dxl_yaw_cur_spd & 0x3FF) * RPMCONSTANT

        # Direction
        if sign["pit"] == '-':
            dxl_pit_spd = -dxl_pit_spd
        if sign["yaw"] == '-':
            dxl_yaw_spd = -dxl_yaw_spd

        return dxl_pit_spd, dxl_yaw_spd
    
    def dxl_multi_get_spd_rev2(self):
        dxl_pit_cur_spd = self.dxl_get_spd(DXL_PIT_ID)
        dxl_yaw_cur_spd = self.dxl_get_spd(DXL_YAW_ID)

        # Unit Conversion: dynamixelStep => deg/s
        dxl_pit_spd = (dxl_pit_cur_spd & 0x3FF) * RPMCONSTANT
        dxl_yaw_spd = (dxl_yaw_cur_spd & 0x3FF) * RPMCONSTANT

        # Direction
        if (dxl_pit_cur_spd & 0x400) == 0:
            dxl_pit_spd = -dxl_pit_spd
        if (dxl_yaw_cur_spd & 0x400) == 0:
            dxl_yaw_spd = -dxl_yaw_spd

        return dxl_pit_spd, dxl_yaw_spd

    def dxl_minmax(self):
        flg = 0
        dxl_yaw_pos, dxl_pit_pos = self.dxl_multi_get_pos()
        if (dxl_yaw_pos < MIN_YAW_POSITION) or (dxl_yaw_pos > MAX_YAW_POSITION):
            flg = DXL_YAW_ID
        if (dxl_pit_pos < MIN_PIT_POSITION) or (dxl_pit_pos > MAX_PIT_POSITION):
            flg = DXL_PIT_ID
        return flg

    def dxl_stp2deg(self, stp_pit, stp_yaw):
        deg_pit = (stp_pit - 2048)*(360/4096)
        deg_yaw = (stp_yaw - 1024)*(360/4096)
        return deg_pit, deg_yaw

    def dxl_deg2stp(self, deg_pit, deg_yaw):
        stp_pit = int(deg_pit*4096/360 + 2048)
        stp_yaw = int(deg_yaw*4096/360 + 1024)
        return stp_pit, stp_yaw
    
    # Degree/s dynamixel input
    def dxl_degPs2dxlIn(self, degPs):
        if degPs < 0:
            degPs = abs(degPs)
        return int(degPs / RPMCONSTANT)

    def dxl_print_pose(self, dxl_pit_pos, dxl_yaw_pos):
        pit_deg, yaw_deg = self.dxl_stp2deg(dxl_pit_pos, dxl_yaw_pos)
        os.system('clear')
        if self.dxl_minmax() == DXL_YAW_ID:
            print("Yaw   - CurPos: OUT OF RANGE!")
        else:
            print("Yaw   - CurPos:%03f" % yaw_deg)
        if self.dxl_minmax() == DXL_PIT_ID:
            print("Pitch - CurPos: OUT OF RANGE!")
        else:
            print("Pitch - CurPos:%03f" % pit_deg)