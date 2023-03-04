##################################################################
# iLidar ToF Motion Blur DCS Dataset Aquisition Gimbal Driver
# Author: Jiho Ryoo
# Date: 2024.06.19
# Description: DCS Images are captured, not depth and intensity.
#              Angular velocity of IMU is also recorded (MPU9250)
# 
##################################################################
import os
import time
import numpy as np
import matplotlib.pyplot as plt
# from tqdm import tqdm
# Dynamixel
from package.dynamixel.dynamixel_driver import *
# Dynamixel
from package.mpu9250.mpu9250 import *
# Save Data as JSON
import datetime
from pytz import timezone
from package.custom_dataset.dataset_io import *
# iLidar
from package.ilidar.ilidar_tof_driver import *


def main():
    #### OPERATION MODE SETTING VALUES ###############################
    # Operation Mode
    V   = 0
    H   = 1
    VH  = 2
    ALL = 3

    ## SELECT OPERATION MODE AND VELOCITY HERE
    print('Operation Mode Selection:')
    print('Chose Operation Mode: V, H, VH, ALL')
    cmd_in = input()
    if cmd_in == 'H' or cmd_in == 'h':
        operation_mode = H
    elif cmd_in == 'V' or cmd_in == 'v':
        operation_mode = V
    elif cmd_in == 'VH' or cmd_in == 'vh':
        operation_mode = VH
    elif cmd_in == 'all' or cmd_in == 'ALL':
        operation_mode = ALL
        
    print('Dynamixel Velocity Selection:')
    print('Type dynamixel velocity: (0,40]')
    print('Pitch: ', end="")
    pit_spd = int(input())
    print('Yaw: ', end="")
    yaw_spd = int(input())

    if yaw_spd > 0:
        SWEEPBACK_YAW   = 20
    else:
        SWEEPBACK_YAW   = -20
    if pit_spd > 0:
        SWEEPBACK_PIT   = 20
    else:
        SWEEPBACK_PIT   = -20

    print("Where is dataset captured?")
    place = input()
    # Json
    path = "/home/pata/dcs_logger/dataset/"
    information = dict()
    information["Capture_time"] = datetime.datetime.now(timezone('Asia/Seoul')).strftime("%Y.%m.%d-%H:%M:%S")
    information["place"] = place
    pose = dict()
    pose["dx"] = 0  # Translation velocity vector
    pose["dy"] = 0
    pose["dz"] = 0
    pose["dp"] = 0  # Angular velocity vector
    pose["dt"] = 0
    pose["dr"] = 0

    # Dataset index set
    dataset_idx = 0
    if operation_mode == V:
        file_path = path + 'V/GT/'
    elif operation_mode == H:
        file_path = path + 'H/GT/'
    elif operation_mode == VH:
        file_path = path + 'VH/GT/'
            
    file_list = os.listdir(file_path)
    file_count = len(file_list)
    if '.DS_Store' in file_list:
        dataset_idx = file_count - 1
    else:
        dataset_idx = file_count
    
    # File index
    print('Continue saving files from previous log? Current initial index is {} - y/n'.format(dataset_idx))
    cmd = input()
    if cmd == 'y':
        pass
    elif cmd == 'n':
        print('Type initial file index: ', end="")
        dataset_idx = int(input())
        print('File index start from {}'.format(dataset_idx))

        
    # Initialize Dynamixel Connection
    os.system("sudo chmod 666 /dev/ttyUSB0")
    dxlDrive = DynamixelDriver()
    dxl_yaw_tar_spd = dxlDrive.dxl_degPs2dxlIn(yaw_spd)
    dxl_pit_tar_spd = dxlDrive.dxl_degPs2dxlIn(pit_spd)

    # Initialize iLidar Connection
    ilidar = iLidarDriver()

    ##################################################################
    ## Start moving
    # Auto Data Aquisition Mode
    # Dataset Aquisition Sequence:
    # 1. Goto target position
    # 2. Get GT image at target position
    # 3. Get MB image at target position
    # 3.1 Sweep back from target angle
    # 3.2 Move with assigned angular velocity
    # 3.3 Get image when target position is reached
    #
    ##################################################################
    img_num = 0
    print('+=================================================+')
    print('|=             MB Dataset Collecter              =|')
    print('|=================================================|')
    print('+  Num  + Yaw + Pit + Mode +       State          |')
    print('| {img_num:^2}/{img_total:^2} | {Yaw:^3} | {Pit:^3} | {mode:^4} | {state:^20} |'\
          .format(img_num=img_num+1, img_total=len(range(-20, 40, 20))*len(range(20, 160, 20)), Yaw=dxl_yaw_tar_deg, Pit=dxl_pit_tar_deg, mode="", state=' '), end="")
    dxl_pit_tar_deg = 0
    dxl_yaw_tar_deg = 0
    #### Get MB Image
    if 1:
        capFin = 0
        while 1:
            # Sweep-Back Sensor
            # Sweep back a little to ensure the LiDAR is at target angular velocity
            if operation_mode == H:
                dxl_pit_tar_stp, dxl_yaw_tar_stp = dxlDrive.dxl_deg2stp(dxl_pit_tar_deg, dxl_yaw_tar_deg - SWEEPBACK_YAW)
            elif operation_mode == V:
                dxl_pit_tar_stp, dxl_yaw_tar_stp = dxlDrive.dxl_deg2stp(dxl_pit_tar_deg - SWEEPBACK_PIT, dxl_yaw_tar_deg)
            elif operation_mode == VH:
                dxl_pit_tar_stp, dxl_yaw_tar_stp = dxlDrive.dxl_deg2stp(dxl_pit_tar_deg - SWEEPBACK_PIT, dxl_yaw_tar_deg - SWEEPBACK_YAW)
            print('\r| {img_num:^2}/{img_total:^2} | {Yaw:^3} | {Pit:^3} | {mode:^4} | {state:^20} |'\
                  .format(img_num=img_num+1, img_total=len(range(-20, 40, 20))*len(range(20, 160, 20)), Yaw=dxl_yaw_tar_deg, Pit=dxl_pit_tar_deg, mode='MB', state='GoTo SweepBack'), end="")
            dxlDrive.dxl_set_pos(DXL_YAW_ID, dxl_yaw_tar_stp, 0)
            dxlDrive.dxl_set_pos(DXL_PIT_ID, dxl_pit_tar_stp, 0)
            while 1:
                tmp_pit_cur_pos_stp, tmp_yaw_cur_pos_stp = dxlDrive.dxl_multi_get_pos()
                # Reached Target Position - Sweep-Back angle
                if (abs(tmp_pit_cur_pos_stp - dxl_pit_tar_stp) < 8) and (abs(tmp_yaw_cur_pos_stp - dxl_yaw_tar_stp) < 8):
                    # print("Reached Sweep-Back Position")
                    break
            print('\r| {img_num:^2}/{img_total:^2} | {Yaw:^3} | {Pit:^3} | {mode:^4} | {state:^20} |'\
                  .format(img_num=img_num+1, img_total=len(range(-20, 40, 20))*len(range(20, 160, 20)), Yaw=dxl_yaw_tar_deg, Pit=dxl_pit_tar_deg, mode='MB', state='SweepBack Fin'), end="")
            
            # Go to target position, and take image when it reaches the position
            dxl_pit_shutter_stp, dxl_yaw_shutter_stp = dxlDrive.dxl_deg2stp(dxl_pit_tar_deg, dxl_yaw_tar_deg)
            if operation_mode == H:
                dxl_pit_tar_stp, dxl_yaw_tar_stp = dxlDrive.dxl_deg2stp(dxl_pit_tar_deg, dxl_yaw_tar_deg + SWEEPBACK_YAW)
            elif operation_mode == V:
                dxl_pit_tar_stp, dxl_yaw_tar_stp = dxlDrive.dxl_deg2stp(dxl_pit_tar_deg + SWEEPBACK_PIT, dxl_yaw_tar_deg)
            elif operation_mode == VH:
                dxl_pit_tar_stp, dxl_yaw_tar_stp = dxlDrive.dxl_deg2stp(dxl_pit_tar_deg + SWEEPBACK_PIT, dxl_yaw_tar_deg + SWEEPBACK_YAW)
            dxlDrive.dxl_set_pos(DXL_YAW_ID, dxl_yaw_tar_stp, dxl_yaw_tar_spd)
            dxlDrive.dxl_set_pos(DXL_PIT_ID, dxl_pit_tar_stp, dxl_pit_tar_spd)

            # Reached Target Position - Sweep-Back angle
            while 1:
                dxl_pit_cur_stp, dxl_yaw_cur_stp = dxlDrive.dxl_multi_get_pos()
                if (abs(dxl_pit_cur_stp - dxl_pit_shutter_stp) < 8) or (abs(dxl_yaw_cur_stp - dxl_yaw_shutter_stp) < 8):
                    img_depth, img_inten, width, height, isFail = ilidar.iLidar_getImg()
                    # Image is not received - Rewind and Get Image Again
                    if isFail != 0:
                        print('\r| {img_num:^2}/{img_total:^2} | {Yaw:^3} | {Pit:^3} | {mode:^4} | {state:^20} |'.format(img_num=img_num+1, img_total=len(range(-20, 40, 20))*len(range(20, 160, 20)), Yaw=dxl_yaw_tar_deg, Pit=dxl_pit_tar_deg, mode='MB', state='ImgGet Fail'), end="")
                    
                    # Get Image Success
                    else:
                        depth = np.ndarray.tolist(img_depth)
                        inten = np.ndarray.tolist(img_inten)

                        # Get angular velocity
                        pose["dp"], pose["dyw"] = dxlDrive.dxl_multi_get_spd()
                        print('\r| {img_num:^2}/{img_total:^2} | {Yaw:^3} | {Pit:^3} | {mode:^4} | {state:^20} |'.format(img_num=img_num+1, img_total=len(range(-20, 40, 20))*len(range(20, 160, 20)), Yaw=dxl_yaw_tar_deg, Pit=dxl_pit_tar_deg, mode='MB', state='Image Received!'))

                        # Pose/position label
                        pose["dx"] = 0  # Translation velocity vector
                        pose["dz"] = 0
                        pose["dy"] = 0
                        pose["r"] = 0   # Pose angle
                        pose["p"], pose["yw"] = dxlDrive.dxl_stp2deg(dxl_pit_cur_stp, dxl_yaw_cur_stp)
                        # pose["p"], pose["yw"] = dxlDrive.dxl_stp2deg(dxl_pit_shutter_stp, dxl_yaw_shutter_stp)
                        pose["dr"] = 0  # Pose angular velocity vector
                        
                        # Save
                        if operation_mode == V:
                            file_path = path + '/V/MB/V_MB_' + str(dataset_idx) + '.json'
                        elif operation_mode == H:
                            file_path = path + '/H/MB/H_MB_' + str(dataset_idx) + '.json'
                        elif operation_mode == VH:
                            file_path = path + '/VH/MB/VH_MB_' + str(dataset_idx) + '.json'  
                        DatasetIO.JSON_write(file_path, width, height, pose, depth, inten, information)
                        capFin = 1
                    break
                elif (abs(dxl_pit_cur_stp - dxl_pit_tar_stp) < 8) and (abs(dxl_yaw_cur_stp - dxl_yaw_tar_stp) < 8):
                    break

            if capFin == 1:
                break
    # Show img
    if 1:
        plt.imshow(depth, cmap='gray')
        plt.xlabel('MB - Depth')
        plt.show(block=False)
        plt.pause(0.5)
        plt.close()

    
    #### Get GT Image
    if 1:
        print('| {img_num:^2}/{img_total:^2} | {Yaw:^3} | {Pit:^3} | {mode:^4} | {state:^20} |'.format(img_num=img_num+1, img_total=len(range(-20, 40, 20))*len(range(20, 160, 20)), Yaw=dxl_yaw_tar_deg, Pit=dxl_pit_tar_deg, mode='GT', state=''), end="")
        # Set Target Angle
        dxl_pit_tar_stp = dxl_pit_cur_stp
        dxl_yaw_tar_stp = dxl_yaw_cur_stp
        # dxl_pit_tar_stp = dxl_pit_shutter_stp
        # dxl_yaw_tar_stp = dxl_yaw_shutter_stp
        print('\r| {img_num:^2}/{img_total:^2} | {Yaw:^3} | {Pit:^3} | {mode:^4} | {state:^20} |'.format(img_num=img_num+1, img_total=len(range(-20, 40, 20))*len(range(20, 160, 20)), Yaw=dxl_yaw_tar_deg, Pit=dxl_pit_tar_deg, mode='GT', state='GoTo SweepBack'), end="")
        dxlDrive.dxl_set_pos(DXL_YAW_ID, dxl_yaw_tar_stp, 0)
        dxlDrive.dxl_set_pos(DXL_PIT_ID, dxl_pit_tar_stp, 0)
        
        while 1:
            # If Current position/pose is target position/pose, send shutter command
            dxl_pit_cur_stp, dxl_yaw_cur_stp = dxlDrive.dxl_multi_get_pos()
            if (abs(dxl_pit_cur_stp - dxl_pit_tar_stp) < 10) and (abs(dxl_yaw_cur_stp - dxl_yaw_tar_stp) < 10):
                print('\r| {img_num:^2}/{img_total:^2} | {Yaw:^3} | {Pit:^3} | {mode:^4} | {state:^20} |'.format(img_num=img_num+1, img_total=len(range(-20, 40, 20))*len(range(20, 160, 20)), Yaw=dxl_yaw_tar_deg, Pit=dxl_pit_tar_deg, mode='GT', state='TarPos Fin'), end="")
                time.sleep(0.5)

                # Get Image
                img_depth, img_inten, width, height, isFail = ilidar.iLidar_getImg()
                while isFail != 0:
                    print('\r| {img_num:^2}/{img_total:^2} | {Yaw:^3} | {Pit:^3} | {mode:^4} | {state:^20} |'.format(img_num=img_num+1, img_total=len(range(-20, 40, 20))*len(range(20, 160, 20)), Yaw=dxl_yaw_tar_deg, Pit=dxl_pit_tar_deg, mode='GT', state='ImgGet Fail'), end="")
                    img_depth, img_inten, width, height, isFail = ilidar.iLidar_getImg()
                print('\r| {img_num:^2}/{img_total:^2} | {Yaw:^3} | {Pit:^3} | {mode:^4} | {state:^20} |'.format(img_num=img_num+1, img_total=len(range(-20, 40, 20))*len(range(20, 160, 20)), Yaw=dxl_yaw_tar_deg, Pit=dxl_pit_tar_deg, mode='GT', state='Image Received!'))
                
                # Pose/position label
                pose["dx"] = 0  # Translation velocity vector
                pose["dy"] = 0
                pose["dz"] = 0
                pose["r"] = 0   # Pose angle
                # pose["p"] = dxl_pit_tar_deg
                # pose["yw"] = dxl_yaw_tar_deg
                pose["p"], pose["yw"] = dxlDrive.dxl_stp2deg(dxl_pit_tar_stp, dxl_yaw_tar_stp)
                pose["dr"] = 0  # Pose angular velocity vector
                pose["dp"] = 0
                pose["dyw"] = 0

                depth = np.ndarray.tolist(img_depth)
                inten = np.ndarray.tolist(img_inten)
                
                # Save
                if operation_mode == V:
                    file_path = path + '/V/GT/V_GT_' + str(dataset_idx) + '.json'
                elif operation_mode == H:
                    file_path = path + '/H/GT/H_GT_' + str(dataset_idx) + '.json'
                elif operation_mode == VH:
                    file_path = path + '/VH/GT/VH_GT_' + str(dataset_idx) + '.json'    
                DatasetIO.JSON_write(file_path, width, height, pose, depth, inten, information)
                break
    # Show img
    if 1:
        plt.imshow(depth, cmap='gray')
        plt.xlabel('GT - Depth')
        plt.show(block=False)
        plt.pause(0.5)
        plt.close()
    
    # Display
    dataset_idx = dataset_idx + 1
    img_num += 1  
    
    # Save file to USB
    # usb_get()



def usb_get():
    try:
        # Mount USB
        os.system("sudo fdisk -l")
        os.system("sudo mount -t ntfs /dev/sda1 usb")
        # Copy Files
        os.system("cp -r H usb")
        print('Files Saved at USB!')
        # Eject USB
        os.system("sudo eject /dev/sda1")
        print('Ready to remove USB')
    except:
        print('Upload Failed!')
    

if __name__ == '__main__':
    main()