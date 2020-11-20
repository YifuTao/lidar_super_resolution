from pypcd import pypcd
from PIL import Image

import math
import numpy as np
from os.path import join
import os
import sys
import time

image_rows_full = 64
image_cols = 1024

def save_grey_img(Images,path,name):
    scale = 255/(Images.max()-Images.min())
    
    # save_video(Images,path,name)

    for i in range(0,Images.shape[0]):
        im = Image.fromarray(Images[i,:,:,0]*scale)
        grey = im.convert("L")
        grey.save(join(path,name,'%s_%d.png'%(name,i)))

def xyz2range(data):
    return np.sqrt(data[0]**2 + data[1]**2 + data[2]**2)

def offset_range_img(pcdata,):

    # define offset
    offset = [0 for x in range(64)]
    for i in range(0,64):
        offset[i]= (i%4) * 6
    
    # offset the range image
    range_image = np.empty([1, image_rows_full, image_cols, 1], dtype=np.float32)
    for i in range(0,64):
        for j in range(0,1024):
            _j = (j+offset[i]) % 1024
            index = _j * 64 + i
            depth = np.sqrt(pcdata[index,0]**2 + pcdata[index,1]**2 + pcdata[index,2]**2)
            range_image[0,i,j] =depth
    return range_image

def reverse_offset_range_img(pcdata):
    # define offset
    offset = [0 for x in range(64)]
    for i in range(0,64):
        offset[i]= (i%4) * 6
    
    pcdata_reverse = np.empty([64*1024,3], dtype=np.float32)
    for i in range(0,64):
        for j in range(0,1024):
            _j = (j+offset[i]) % 1024
            index = _j * 64 + i
            pcdata_reverse[index] = pcdata[i,j]
    return pcdata_reverse.reshape(64,1024,3) 



def pc2range(pcdata):
    range_image = np.empty([1, image_rows_full, image_cols, 1], dtype=np.float32)
    for i in range(0,64):
        for j in range(0,1024):
            #_j = (j+offset[i]) % 1024
            index = i * 64 + j
            depth = np.sqrt(pcdata[index,0]**2 + pcdata[index,1]**2 + pcdata[index,2]**2)
            range_image[0,i,j] =depth
    return range_image

def create_4dir(path):
    if not os.path.exists(path):
        os.mkdir(join(path))
        os.mkdir(join(path,'gt'))
        os.mkdir(join(path,'input'))
        os.mkdir(join(path,'prd'))
        os.mkdir(join(path,'prd_clean'))

def progressbar(count,total):
        num = int(float(count)/total*100/5)
        percent = float(count)/total*100
        # sys.stdout.write("\r %d Processing %dth /%d" % ( int(percent),count, total))
        sys.stdout.write("\rProcessing %dth/%d[%-20s] %d%%  " % (count,total,'='*num, int(percent)))
        sys.stdout.flush()
        if count == total:
            print


def main():

    range_image_array = np.empty([0, image_rows_full, image_cols, 1], dtype=np.float32)
    # find all bag files in the given dir
    path = '/home/yifu/data/short_experiment'
    print('Data Path:',path)
    create_4dir(join(path,'range image'))
    create_4dir(join(path,'processed pcd'))
    if not os.path.exists(join(path,'numpy')):
        os.mkdir(join(path,'numpy'))

    bag_file_path = join(path)
    bag_files = sorted(os.listdir(bag_file_path))

    # loop through all bags in the given directory
    for file_name in bag_files:
        if file_name[-4:]!='.pcd':
            continue
        count = range_image_array.shape[0] 
        file_path = os.path.join(bag_file_path, file_name)
        pc = pypcd.PointCloud.from_path(file_path)
        pcdata = pc.pc_data.view(np.float32).reshape(-1,3)
        range_image = offset_range_img(pcdata)
        ''' 
        range_image = np.empty([1, image_rows_full, image_cols, 1], dtype=np.float32)

        # offset the range image
        for i in range(0,64):
            for j in range(0,1024):
                _j = (j+offset[i]) % 1024
                index = _j * 64 + i
                depth = np.sqrt(pcdata[index,0]**2 + pcdata[index,1]**2 + pcdata[index,2]**2)
                range_image[0,i,j] =depth 
        '''
        map_path = join(path,'range image')

        range_image_array = np.append(range_image_array, range_image, axis=0)
        progressbar(range_image_array.shape[0],len(bag_files))

            
    save_grey_img(range_image_array,map_path,'gt')
    npy_file_name = join(path,'numpy','gt.npy')
    np.save(npy_file_name, range_image_array)


if __name__ == '__main__':
    main()