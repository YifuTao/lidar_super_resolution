import numpy as np
from pypcd import pypcd
import os
#import matplotlib.pyplot as plt
from os.path import join
import time
import sys

import struct

#from sensor_msgs import point_cloud2
#from sensor_msgs.msg import PointCloud2, PointField
#from std_msgs.msg import Header

from numpy2pcd import get_low_res_input,noise_remove, range2xyz, range2pcd
from readpcd import offset_range_img,image_rows_full,image_cols,save_grey_img,create_4dir,progressbar




range_image_array = np.empty([0, image_rows_full, image_cols, 1], dtype=np.float32)


file_path = '/home/yifu/data/long_experiment/cloud_0000_1583840211_539847424.pcd'
pc = pypcd.PointCloud.from_path(file_path)
pcdata = pc.pc_data.view(np.float32).reshape(-1,3)
np.savetxt("/home/yifu/data/long_experiment/processed pcd/raw %s.csv"%sys.version[:3], pcdata, delimiter=",")

range_image = offset_range_img(pcdata)

# recover
cloud = range2xyz(range_image[0])
np.savetxt("/home/yifu/data/long_experiment/processed pcd/%s.csv"%sys.version[:3], cloud.reshape(-1,3), delimiter=",")
output_pcd = pypcd.make_xyz_point_cloud(cloud.reshape(-1,3))
output_pcd.save(join('/home/yifu/data/long_experiment/processed pcd/numpysave.pcd'))