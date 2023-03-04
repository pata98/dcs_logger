import os
import shutil

def dataset_delete():
    # print('!!Dataset Erase!!')
    # print('All data will be lost. Enter yes to confirm delete.')
    # cmd = input()
    # if cmd == 'yes':
    #     path = "/home/pata/Documents/MBPoseDataset/MBPose_Dataset/"
    #     try:
    #         shutil.rmtree(path + 'H')
    #         shutil.rmtree(path + 'V')
    #         shutil.rmtree(path + 'VH')
    #     except:
    #         pass

    #     os.mkdir(path + 'H')
    #     os.mkdir(path + 'H/GT')
    #     os.mkdir(path + 'H/MB')
    #     os.mkdir(path + 'V')
    #     os.mkdir(path + 'V/GT')
    #     os.mkdir(path + 'V/MB')
    #     os.mkdir(path + 'VH')
    #     os.mkdir(path + 'VH/GT')
    #     os.mkdir(path + 'VH/MB')
    #     print('Dataset Erase Successful!')
    # else:
    #     print('Dataset Not Erased!')
    
    path = "/home/pata/Documents/MBPoseDataset/MBPose_Dataset/"
    try:
        shutil.rmtree(path + 'GT')
        shutil.rmtree(path + 'MB')
    except:
        pass

    os.mkdir(path + 'GT')
    os.mkdir(path + 'MB')
    print('Dataset Erase Successful!')

if __name__ == '__main__':
    dataset_delete()