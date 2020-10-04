from pypcd import pypcd
import pcl
import math
import numpy as np
import os

# p = pcl.load('/home/yifu/Downloads/cloud_0000_1583840211_539847424.pcd')

image_rows_full = 64
image_cols = 1024

# define offset
offset = [0 for x in range(64)]
for i in range(0,64):
    offset[i]= (i%4) * 6

range_image_array = np.empty([0, image_rows_full, image_cols, 1], dtype=np.float32)
# find all bag files in the given dir
data_set_name = '/home/yifu/Workspace/lidar_super_resolution/data/long_experiment'
bag_file_path = os.path.join(data_set_name)
bag_files = sorted(os.listdir(bag_file_path))
print(bag_files)
# loop through all bags in the given directory
for file_name in bag_files:
    if file_name[-4:]!='.pcd':
        continue
    # bag file path
    file_path = os.path.join(bag_file_path, file_name)
    # open ros bags
    pc = pypcd.PointCloud.from_path(file_path)
    pcdata = pc.pc_data.view(np.float32).reshape(-1,3)
    range_image = np.empty([1, image_rows_full, image_cols, 1], dtype=np.float32)
    '''
    new_cloud_data = pc.pc_data.view(np.float32).reshape(64,1024,3)
    range_image = new_cloud_data[:,:,0]**2 + new_cloud_data[:,:,1]**2 + new_cloud_data[:,:,2]**2
    range_image = range_image.reshape(1,64,1024,1)
    tmp = np.copy(range_image)
    '''

    # offset the range image
    for i in range(0,64):
        for j in range(0,1024):
            _j = (j+offset[i]) % 1024
            index = _j * 64 + i
            depth = np.sqrt(pcdata[index,0]**2 + pcdata[index,1]**2 + pcdata[index,2]**2)
            range_image[0,63-i,j] =depth  # flip the point cloud
    
    range_image_array = np.append(range_image_array, range_image, axis=0)
    if range_image_array.shape[0]%5==0:
        print(range_image_array.shape[0])
    # if (range_image_array.shape[0]>=1000):
    #     break
        

npy_file_name = '/home/yifu/Documents/SuperResolution/long_experiment_64.npy'
np.save(npy_file_name, range_image_array)






# new_cloud_data = 
# a = np.asarray(p)
# tmp = p.make_RangeImage()

angularResolution = 1 * math.pi / 180 
maxAngleWidth = 360 * math.pi / 180 
maxAngleHeight = 180 * math.pi / 180 

noiseLevel = 0
minRange = 0
borderSize = 1

# test = tmp.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
#                                   sensorPose, coordinate_frame, noiseLevel, minRange, borderSize)
# visual = pc.pcl_visualization.CloudViewing()
# visual.ShowMonochromeCloud(pc)


# downsampling: take 1 row from every 4 row. 
# use pcl/opencv to convert this pointcloud into depth array

