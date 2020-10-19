import numpy as np
import math
from pypcd import pypcd


_elevation_angles = [
         17.74400,  17.12000,  16.53600,  15.98200,  15.53000,  14.93600,  14.37300,  13.82300,
         13.37300,  12.78600,  12.23000,  11.68700,  11.24100,  10.67000,  10.13200,   9.57400,
          9.13800,   8.57700,   8.02300,   7.47900,   7.04600,   6.48100,   5.94400,   5.39500,
          4.96300,   4.40100,   3.85900,   3.31900,   2.87100,   2.32400,   1.78300,   1.23800,
          0.78600,   0.24500,  -0.29900,  -0.84900,  -1.28800,  -1.84100,  -2.27500,  -2.92600,
         -3.37800,  -3.91000,  -4.45700,  -5.00400,  -5.46000,  -6.00200,  -6.53700,  -7.09600,
         -7.55200,  -8.09000,  -8.62900,  -9.19600,  -9.65700, -10.18300, -10.73200, -11.28900,
        -11.77000, -12.29700, -12.85400, -13.41500, -13.91600, -14.44200, -14.99700, -15.59500]

_azimuth_angle_offsets = [
        3.10200, 0.9290, -1.2370, -3.37800, 3.06300, 0.9120, -1.2170, -3.33200,
        3.04500, 0.9220, -1.1850, -3.28500, 3.04200, 0.9340, -1.1670, -3.25400,
        3.03000, 0.9420, -1.1390, -3.22700, 3.03400, 0.9580, -1.1210, -3.18600,
        3.04800, 0.9840, -1.0840, -3.15800, 3.05900, 0.9980, -1.0660, -3.13400,
        3.08500, 1.0270, -1.0420, -3.11100, 3.11700, 1.0500, -1.0660, -3.07100,
        3.14900, 1.0800, -0.9890, -3.05800, 3.19000, 1.1130, -0.9580, -3.03700,
        3.23000, 1.1440, -0.9310, -3.02700, 3.27400, 1.1900, -0.8990, -3.02200,
        3.32700, 1.2360, -0.8810, -3.00600, 3.39300, 1.2750, -0.8600, -3.00400]

_azimuth_pixel_offsets = 16 * [0, 6, 12, 18]

def angle2xyz(_azimuth, _elevation, depth):
    azimuth = _azimuth * math.pi / 180
    elevation = _elevation * math.pi / 180

    X = depth * math.cos(elevation) * math.cos(azimuth)
    Y = depth * math.cos(elevation) * math.sin(azimuth)
    Z = depth * math.sin(elevation) 
    if depth !=0:
        a=0
    return X,Y,Z

def noise_remove(origImages,predImages,):
    # get low_res_index
    image_rows_low = 16 # 8, 16, 32
    image_rows_high = 64 # 16, 32, 64
    upscaling_factor = int(image_rows_high / image_rows_low)
    low_res_index = range(0, image_rows_high, upscaling_factor)
    # remove noise
    predImagesNoiseReduced = np.copy(predImages[:,:,:,0:1])
    noiseLabels = predImages[:,:,:,1:2]
    predImagesNoiseReduced[noiseLabels > predImagesNoiseReduced * 0.03] = 0 # after noise removal
    predImagesNoiseReduced[:,low_res_index] = origImages[:,low_res_index]
    predImagesNoiseReduced[:,low_res_index,:,0:1] = origImages[:,low_res_index,:,0:1] # copy some of beams from origImages to predImages

    return predImagesNoiseReduced

def range2pcd(Images,name):
    output = np.empty([64,1024,3],dtype=np.float32)
    for i in range(0,64):
        for j in range(0,1024):
            # pixel_azimuth = _azimuth_angle_offsets[i] + j * 360 / 1024 # in Degrees
            pixel_azimuth =  j * 360 / 1024 # in Degrees
            pixel_azimuth = 360 - pixel_azimuth
            pixel_elevation = _elevation_angles[63-i]  # in Degrees
            x, y, z = angle2xyz(pixel_azimuth, pixel_elevation, Images[i,j,0])
            output[i,j,0]= x
            output[i,j,1]= y
            output[i,j,2]= z
    output_pcd = pypcd.make_xyz_point_cloud(output.reshape(-1,3))

    output_pcd.save('/Volumes/EthanSSD/linux backup/Documents/%s.pcd'%name)
    # output_pcd.save('/home/yifu/Workspace/lidar_super_resolution/data/long_experiment_prd/%s.pcd'%(name))


def main():
    normalize_ratio = 100.0
    # gt = np.load('/home/yifu/Documents/SuperResolution/long_experiment_64.npy')
    gt = np.load('/Volumes/EthanSSD/linux_backup/Documents/SuperResolution/long_experiment_64.npy')
    # prd = np.load('/home/yifu/Documents/SuperResolution/myouster_range_image-UNet-from-16-to-64_prediction.npy') * normalize_ratio # 
    prd = np.load('/Volumes/EthanSSD/linux_backup/Documents/SuperResolution/myouster_range_image-UNet-from-16-to-64_prediction.npy')
    np.savetxt('prd.csv',prd[0,:,:,0], delimiter=',')
    np.savetxt('gt.csv',gt[0,:,:,0], delimiter=',')

    # noise removal
    
    low_res_input = np.zeros(gt.shape, dtype=np.float32) # for visualizing NN input images
    low_res_input[:,low_res_index] = gt[:,low_res_index]

    Images = noise_remove(gt, prd)
    for i in range(0,Images.shape[0]):
        range2pcd(low_res_input[i],'%d_input'%i)
        range2pcd(gt[i],'%d_gt'%i)
        range2pcd(Images[i],'%d_prd'%i)

if __name__ == '__main__':
    main()