import numpy as np
from pypcd import pypcd
import os
import matplotlib.pyplot as plt
from os.path import join
import time
import sys

import rospy
import struct
import rosbag

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from numpy2pcd import get_low_res_input,noise_remove, range2xyz, range2pcd
from readpcd import offset_range_img,image_rows_full,image_cols,save_grey_img,create_4dir,progressbar




range_image_array = np.empty([0, image_rows_full, image_cols, 1], dtype=np.float32)


file_path = '/home/yifu/data/long_experiment/cloud_0000_1583840211_539847424.pcd'
pc = pypcd.PointCloud.from_path(file_path)
pcdata = pc.pc_data.view(np.float32).reshape(-1,3)
range_image = offset_range_img(pcdata)

map_path = '/home/yifu/data/bagfiles/gt_input_prd'

range_image_array = np.append(range_image_array, range_image, axis=0)

        
# save_grey_img(range_image_array,map_path,'gt')
npy_file_name = '/home/yifu/data/bagfiles/gt_input_prd/gt.npy'
np.save(npy_file_name, range_image_array)

gt = np.load(npy_file_name)
i=0
range2pcd(gt[i],map_path,'gt', i)