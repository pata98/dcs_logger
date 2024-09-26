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
import time
# Motor
from package.mg4005e_can import *
# IMU
from package.mpu9250.mpu9250 import *
# Save Data as JSON
import datetime
from pytz import timezone
from package.custom_dataset.dataset_io import *
# iLidar
from package.ilidar.ilidar_tof_driver import *


def main():
    #### OPERATION MODE INPUT ########################################
    ## Target angular velocity
    ANG_VEL_MIN = -10
    ANG_VEL_MAX = 10
    ANG_VEL_MAX += 1
    ANG_VEL_INT = 5
    # Image size
    WIDTH  = 320
    HEIGHT = 160
    # CONST
    SWEEPBACK_TIME  = 1
    SWEEPBACK_SPEED = 20
    SHUTTER_ANGLE_THRESHOLD = 0.5

    ## Initialize iLidar Connection
    # ilidar = iLidarDriver()

    ## Check Scene
    while True:
        # dcs, isFail = ilidar.iLidar_getImg()
        isFail = 0

        print("Is the scene ok? (y/n): ", end="")
        cmd = input()
        if cmd == 'y':
            break
        elif cmd == 'n':
            print("Please check the scene and press enter to continue")
    
    ## DATASET CAPTURE PLACE
    print("Where is dataset captured?: ", end="")
    place = input()

    ## Initial file index
    path = "/home/pata/Documents/dcs_logger/dataset/tmp"
    while True:
        file_list = os.listdir(path + '/')
        file_count = len(file_list)
        if '.DS_Store' in file_list:
            dataset_idx = file_count - 1
        else:
            dataset_idx = file_count
        print('Continue saving files from previous log? Current initial index is {} - y/n: '.format(dataset_idx), end="")
        cmd = input()
        if cmd == 'y':
            break
        elif cmd == 'n':
            print('Type initial file index: ', end="")
            dataset_idx = int(input())
            print('File index start from {}'.format(dataset_idx))
            break
        else:
            print("Wrong Input!")


    #### INIRTIALIZE #################################################
    ## Save data format - JSON
    information = dict()
    information["time"] = datetime.datetime.now(timezone('Asia/Seoul')).strftime("%Y.%m.%d-%H:%M:%S")
    information["place"] = place
    pose = dict()
    pose["dx"] = 0  # Translation velocity vector
    pose["dy"] = 0
    pose["dz"] = 0
    pose["dp"] = 0  # Angular velocity vector
    pose["dt"] = 0
    pose["dr"] = 0

    ## Initialize Motor Connection
    canDrive = MotorDriver()
    canDrive.motor_on(canDrive.ID_MOTOR_PAN)
    canDrive.motor_on(canDrive.ID_MOTOR_TILT)


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
    dxl_pan_tar_deg = 0
    dxl_tlt_tar_deg = 0
    print('+=======================================================+')
    print('|=                MB Dataset Collecter                 =|')
    print('|=======================================================|')
    print('+  Num   + Pan rate + Tilt rate +        State          |',end="")
    for pan_vel_tar in range(ANG_VEL_MIN, ANG_VEL_MAX, ANG_VEL_INT):
        for tlt_vel_tar in range(ANG_VEL_MIN, ANG_VEL_MAX, ANG_VEL_INT):
            #### Print status on terminal
            print('\n| {img_idx:^2}/{img_total:^3} | {Pan:^3}deg/s | {Tilt:^3}deg/s  | {state:^20}  |'\
                .format(img_idx=img_num+1, img_total=len(range(ANG_VEL_MIN, ANG_VEL_MAX, ANG_VEL_INT))**2, Pan=pan_vel_tar, Tilt=tlt_vel_tar, state=' '), end="")
            
            #### Get MB Image
            if 1:
                captureFinished = 0
                while 1:
                    #### Sweep-Back Sensor - Sweep back a little to ensure the LiDAR is at target angular velocity
                    ## Print status on terminal
                    print('\r| {img_idx:^2}/{img_total:^3} | {Pan:^3}deg/s | {Tilt:^3}deg/s  | {state:^20}  |'\
                        .format(img_idx=img_num+1, img_total=len(range(ANG_VEL_MIN, ANG_VEL_MAX, ANG_VEL_INT))**2,\
                                 Pan=pan_vel_tar, Tilt=tlt_vel_tar, state='GoTo SweepBack'), end="")
                    
                    ## Go to sweep back position
                    tlt_tar_angle = -tlt_vel_tar * SWEEPBACK_TIME
                    pan_tar_angle = -pan_vel_tar * SWEEPBACK_TIME
                    canDrive.motor_move(canDrive.ID_MOTOR_TILT, tlt_tar_angle, SWEEPBACK_SPEED, stay=True)
                    canDrive.motor_move(canDrive.ID_MOTOR_PAN,  pan_tar_angle, SWEEPBACK_SPEED, stay=True)
                    print('\r| {img_idx:^2}/{img_total:^3} | {Pan:^3}deg/s | {Tilt:^3}deg/s  | {state:^20}  |'\
                            .format(img_idx=img_num+1, img_total=len(range(ANG_VEL_MIN, ANG_VEL_MAX, ANG_VEL_INT))**2, \
                                    Pan=pan_vel_tar, Tilt=tlt_vel_tar, state='SweepBack Fin'), end="")
                    
                    
                    #### Go to target position + sweep back angle. Take image when reached target position.
                    ## Set final position. (target positon + sweep back)
                    if tlt_vel_tar == 0:
                        tlt_vel_tar = 5
                        canDrive.motor_move(canDrive.ID_MOTOR_TILT, -tlt_tar_angle, abs(tlt_vel_tar))
                        tlt_vel_tar = 0
                    else:
                        canDrive.motor_move(canDrive.ID_MOTOR_TILT, -tlt_tar_angle, abs(tlt_vel_tar))

                    if pan_vel_tar == 0:
                        pan_vel_tar = 5
                        canDrive.motor_move(canDrive.ID_MOTOR_PAN,  -pan_tar_angle, abs(pan_vel_tar))
                        pan_vel_tar = 0
                    else:
                        canDrive.motor_move(canDrive.ID_MOTOR_PAN,  -pan_tar_angle, abs(pan_vel_tar))
                    
                    while 1:
                        ## Read current position (in step)
                        tlt_cur_respond = canDrive.read_motor_state(canDrive.ID_MOTOR_TILT)
                        pan_cur_respond = canDrive.read_motor_state(canDrive.ID_MOTOR_PAN)

                        ## Take image while when reached target position
                        if (abs(tlt_cur_respond.get('angle')) < SHUTTER_ANGLE_THRESHOLD) or \
                            (abs(pan_cur_respond.get('angle')) < SHUTTER_ANGLE_THRESHOLD):
                            
                            ## Take DCS Image
                            # dcs, isFail = ilidar.iLidar_getImg()
                            k1 = np.zeros((320,160))
                            k2 = np.zeros((320,160))
                            k3 = np.zeros((320,160))
                            k4 = np.zeros((320,160))
                            dcs=[k1,k2,k3,k4]
                            isFail = 0

                            ## Image is not received - Rewind and Get Image Again
                            if isFail != 0:
                                print('\r| {img_idx:^2}/{img_total:^3} | {Pan:^3}deg/s | {Tilt:^3}deg/s  | {state:^20}  |'\
                                    .format(img_idx=img_num+1, img_total=len(range(ANG_VEL_MIN, ANG_VEL_MAX, ANG_VEL_INT))**2, \
                                        Pan=pan_vel_tar, Tilt=tlt_vel_tar, state='ImgGet Fail'), end="")
                                
                            ## Get Image Success
                            else:
                                ## Pose/position label
                                # Angular velocity vector
                                pose["dt"] = tlt_cur_respond.get('speed')
                                pose["dp"] = pan_cur_respond.get('speed')
                                pose["dr"] = 0
                                # Translation velocity vector
                                pose["dx"] = 0  
                                pose["dy"] = 0
                                pose["dz"] = 0

                                ## Image Data
                                dcs0 = np.ndarray.tolist(dcs[0])
                                dcs1 = np.ndarray.tolist(dcs[1])
                                dcs2 = np.ndarray.tolist(dcs[2])
                                dcs3 = np.ndarray.tolist(dcs[3])

                                ## Print status on terminal
                                print('\r| {img_idx:^2}/{img_total:^3} | {Pan:^3}deg/s | {Tilt:^3}deg/s  | {state:^20}  |'\
                                    .format(img_idx=img_num+1, img_total=len(range(ANG_VEL_MIN, ANG_VEL_MAX, ANG_VEL_INT))**2, \
                                        Pan=pan_vel_tar, Tilt=tlt_vel_tar, state='Image Received!'), end="")
                                
                                ## Save
                                file_path = path + '/img_' + str(dataset_idx) + '.json'  
                                DatasetIO.JSON_write(file_path, WIDTH, HEIGHT, pose, dcs0, dcs1, dcs2, dcs3, information)
                                captureFinished = 1
                            break
                        
                        ## Escape while when reached target position
                        elif (abs(tlt_cur_respond.get('angle') + tlt_tar_angle) < SHUTTER_ANGLE_THRESHOLD) and \
                            (abs(pan_cur_respond.get('angle') + pan_tar_angle) < SHUTTER_ANGLE_THRESHOLD):
                            break

                    if captureFinished == 1:
                        break

            ## Goto origin
            canDrive.motor_move(canDrive.ID_MOTOR_TILT, 0, SWEEPBACK_SPEED)
            canDrive.motor_move(canDrive.ID_MOTOR_PAN,  0, SWEEPBACK_SPEED)

            # Show img
            if 0:
                show_img(dcs, 1)

            """
            #### Get GT Image
            if 0:
                print('| {img_idx:^2}/{img_total:^3} | {Pan:^3}deg/s | {Tilt:^3}deg/s  | {state:^20}  |'\
                    .format(img_idx=img_num+1, img_total=len(range(ANG_VEL_MIN, ANG_VEL_MAX, ANG_VEL_INT))**2, \
                            Pan=pan_vel_tar, Tilt=tlt_vel_tar, state='GT!'), end="")
                # Set Target Angle
                dxl_pit_tar_stp = dxl_tlt_cur_stp
                dxl_yaw_tar_stp = dxl_pan_cur_stp
                print('\r| {img_idx:^2}/{img_total:^3} | {Pan:^3}deg/s | {Tilt:^3}deg/s  | {state:^20}  |'\
                        .format(img_idx=img_num+1, img_total=len(range(ANG_VEL_MIN, ANG_VEL_MAX, ANG_VEL_INT))**2,\
                                 Pan=pan_vel_tar, Tilt=tlt_vel_tar, state='GoTo TarPos'), end="")
                dxlDrive.dxl_set_pos(DXL_PAN_ID, dxl_yaw_tar_stp, 0)
                dxlDrive.dxl_set_pos(DXL_TLT_ID, dxl_pit_tar_stp, 0)
                
                while 1:
                    # If Current position/pose is target position/pose, send shutter command
                    dxl_tlt_cur_stp, dxl_pan_cur_stp = dxlDrive.dxl_multi_get_pos()
                    if (abs(dxl_tlt_cur_stp - dxl_tlt_tar_stp) < 8) and (abs(dxl_pan_cur_stp - dxl_pan_tar_stp) < 8):
                        print('\r| {img_idx:^2}/{img_total:^3} | {Pan:^3}deg/s | {Tilt:^3}deg/s  | {state:^20}  |'\
                            .format(img_idx=img_num+1, img_total=len(range(ANG_VEL_MIN, ANG_VEL_MAX, ANG_VEL_INT))**2, \
                                    Pan=pan_vel_tar, Tilt=tlt_vel_tar, state='TarPos Fin'), end="")
                        time.sleep(0.5)

                        ## Take DCS Image
                        # dcs, isFail = ilidar.iLidar_getImg()
                        isFail = 0
                        ## Image is not received - Rewind and Get Image Again
                        while isFail != 0:
                            print('\r| {img_idx:^2}/{img_total:^3} | {Pan:^3}deg/s | {Tilt:^3}deg/s  | {state:^20}  |'\
                                .format(img_idx=img_num+1, img_total=len(range(ANG_VEL_MIN, ANG_VEL_MAX, ANG_VEL_INT))**2, \
                                    Pan=pan_vel_tar, Tilt=tlt_vel_tar, state='ImgGet Fail'), end="")
                            # dcs, isFail = ilidar.iLidar_getImg()
                            isFail = 0
                        print('\r| {img_idx:^2}/{img_total:^3} | {Pan:^3}deg/s | {Tilt:^3}deg/s  | {state:^20}  |'\
                            .format(img_idx=img_num+1, img_total=len(range(ANG_VEL_MIN, ANG_VEL_MAX, ANG_VEL_INT))**2, \
                                    Pan=pan_vel_tar, Tilt=tlt_vel_tar, state='Image Received!'), end="")

                        ## Pose/position label
                        # Angular velocity vector
                        pose["dt"], pose["dp"] = dxlDrive.dxl_multi_get_spd()
                        pose["dr"] = 0
                        # Translation velocity vector
                        pose["dx"] = 0  
                        pose["dy"] = 0
                        pose["dz"] = 0

                        ## Image Data
                        dcs0 = np.ndarray.tolist(dcs[0])
                        dcs1 = np.ndarray.tolist(dcs[1])
                        dcs2 = np.ndarray.tolist(dcs[2])
                        dcs3 = np.ndarray.tolist(dcs[3])
                        
                        ## Save
                        file_path = path + '/tmp/gt_img_' + str(dataset_idx) + '.json'  
                        DatasetIO.JSON_write(file_path, WIDTH, HEIGHT, pose, dcs0, dcs1, dcs2, dcs3, information)
                        break
            """
                
            # Show img
            if 0:
                show_img(dcs, 1)
            
            # Display
            dataset_idx += 1
            img_num += 1  
            

    ## Finished!
    print('\n+=======================================================+')
    print('|=                     Finished!                       =|')
    print('|=======================================================|')
    canDrive.motor_off(canDrive.ID_MOTOR_PAN)
    canDrive.motor_off(canDrive.ID_MOTOR_TILT)


def show_img(dcs, show_period=0):
    x = dcs[2] - dcs[0]
    y = dcs[3] - dcs[1]
    intensity = np.sqrt((x/20)**2 + (y/20)**2)

    plt.imshow(intensity, cmap='gray')
    plt.xlabel('Intensity')
    plt.show(block=False)
    if show_period != 0:
        plt.pause(show_period)
    plt.close()


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