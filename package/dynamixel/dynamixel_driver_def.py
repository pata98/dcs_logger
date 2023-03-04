# DynamixelDriverDef.py
# Author: JihoRyoo
#

#### Dynamixel Control Variables ##########################################
# Control Table Address
ADDR_MX_TORQUE_ENABLE       = 24
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_GOAL_SPEED          = 32
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_PRESENT_SPEED       = 38

# Protocol Version
PROTOCOL_VERSION            = 1.0

# Dynamixel Communication Setting
DXL_PIT_ID  = 1
DXL_YAW_ID  = 2
BAUDRATE    = 57600
# DEVICENAME  = '/dev/tty.usbserial-FT763JHN'    # Only for Mac
DEVICENAME  = '/dev/ttyUSB0'    # For Linux - Check using ls /dev/tty*

# Dynamixel State Command Setting
TORQUE_ENABLE   = 1
TORQUE_DISABLE  = 0

# Dynamixel Constants
RPMCONSTANT     = 0.684     # deg/s to dxlStep => 0.114 * 6

# Dynamixel Boundary Conditions
MAX_YAW_POSITION        = 3072
MIN_YAW_POSITION        = 1024
MAX_PIT_POSITION        = 3072
MIN_PIT_POSITION        = 1024