import numpy as np
import math
from pypcd import pypcd
import os
from os.path import join
import time
import sys
from newercollege_lidar_angles import generate_azimuth_angles, to_eigen_variable,_elevation_angles

def angle2xyz(_azimuth, _elevation, depth):
    azimuth = _azimuth * math.pi / 180
    elevation = _elevation * math.pi / 180

    X = depth * math.cos(elevation) * math.cos(azimuth)
    Y = depth * math.cos(elevation) * math.sin(azimuth)
    Z = depth * math.sin(elevation) 
    if depth !=0:
        a=0
    return X,Y,Z

def get_low_res_index():
    image_rows_low = 16 # 8, 16, 32
    image_rows_high = 64 # 16, 32, 64
    upscaling_factor = int(image_rows_high / image_rows_low)
    low_res_index = range(0, image_rows_high, upscaling_factor)
    return low_res_index

def noise_remove(origImages,predImages,):
    # get low_res_index
    low_res_index = get_low_res_index()
    # remove noise
    predImagesNoiseReduced = np.copy(predImages[:,:,:,0:1])
    noiseLabels = predImages[:,:,:,1:2]
    predImagesNoiseReduced[noiseLabels > predImagesNoiseReduced * 0.03] = 0 # after noise removal
    predImagesNoiseReduced[:,low_res_index] = origImages[:,low_res_index]
    predImagesNoiseReduced[:,low_res_index,:,0:1] = origImages[:,low_res_index,:,0:1] # copy some of beams from origImages to predImages

    return predImagesNoiseReduced

def range2xyz(Images,):
    azimuth_offset = generate_azimuth_angles()
    output = np.empty([64,1024,3],dtype=np.float32)
    for i in range(0,64):
        for j in range(0,1024):
            # pixel_azimuth = _azimuth_angle_offsets[i] + j * 360 / 1024 # in Degrees
            pixel_azimuth =  j * 360 / 1024 + azimuth_offset[i] # in Degrees    this offset seems noisy?
            pixel_azimuth = 360 - pixel_azimuth
            pixel_elevation = _elevation_angles[i]  # in Degrees
            x, y, z = angle2xyz(pixel_azimuth, pixel_elevation, Images[i,j,0])
            output[i,j,0]= x
            output[i,j,1]= y
            output[i,j,2]= z
    return output

def range2pcd(Images,path,name,index):

    cloud_xyz = range2xyz(Images)
    output_pcd = pypcd.make_xyz_point_cloud(cloud_xyz.reshape(-1,3))

    output_pcd.save(join(path,'processed pcd','%s'%name,'%04d.pcd'%index))
    # output_pcd.save('/home/yifu/Workspace/lidar_super_resolution/data/long_experiment_prd/%s.pcd'%(name))


def get_low_res_input(gt):
    low_res_index = get_low_res_index()
    low_res_input = np.zeros(gt.shape, dtype=np.float32) # for visualizing NN input images
    low_res_input[:,low_res_index] = gt[:,low_res_index]
    return low_res_input


def main():
    normalize_ratio = 100.0
    #path = '/home/yifu/data'
    path = '/home/yifu/data/long_experiment'
    if not os.path.exists(join(path,'range_image')):
        os.mkdir(join(path,'range_image'))
        os.mkdir(join(path,'range_image','gt'))
        os.mkdir(join(path,'range_image','input'))
        os.mkdir(join(path,'range_image','prd'))
        os.mkdir(join(path,'range_image','prd_clean'))

    if not os.path.exists(join(path,'processed pcd')):
        os.mkdir(join(path,'processed pcd'))
        os.mkdir(join(path,'processed pcd','gt'))
        os.mkdir(join(path,'processed pcd','prd'))
        os.mkdir(join(path,'processed pcd','input'))
        os.mkdir(join(path,'processed pcd','prd_clean'))



    gt = np.load(join(path,'numpy','gt.npy'))
    prd = np.load(join(path,'numpy','myouster_range_image-UNet-from-16-to-64_prediction.npy'))*normalize_ratio
    
    # np.savetxt('prd.csv',prd[0,:,:,0], delimiter=',')
    # np.savetxt('gt.csv',gt[0,:,:,0], delimiter=',')

    Images = noise_remove(gt, prd)
    
    low_res_index = get_low_res_index()
    low_res_input = np.zeros(gt.shape, dtype=np.float32) # for visualizing NN input images
    low_res_input[:,low_res_index] = gt[:,low_res_index]

    

    for i in range(0,Images.shape[0]):
        sys.stdout.write("\rProcessing %dth /%d" % (i,Images.shape[0]))
        sys.stdout.flush()
        range2pcd(low_res_input[i],path,'input',i)
        range2pcd(gt[i],path,'gt', i)
        range2pcd(prd[i],path,'prd', i)
        range2pcd(Images[i],path,'prd_clean', i)

if __name__ == '__main__':
    main()