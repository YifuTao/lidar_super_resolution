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
import sensor_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from numpy2pcd import get_low_res_input,noise_remove, range2xyz
from readpcd import offset_range_img,image_rows_full,image_cols,save_grey_img,create_4dir,progressbar
# generate depth and colored point cloud


bag_path = '/home/yifu/data/bagfiles/NewerCollege_short_raw_2.bag'
outbag = rosbag.Bag('/home/yifu/data/bagfiles/gt_input_prd.bag','w')

bag_topic = '/object_scan/raw_cloud'
bag = rosbag.Bag(bag_path)

gt_path = '/home/yifu/data/bagfiles/NewerCollege_short_raw_2/gt.npy'
gt = np.load(gt_path)


count = 0
for topic, depth_image, t in bag.read_messages(topics=[bag_topic, ]):

    depth_points = sensor_msgs.msg.PointCloud2()
    depth_points.header = depth_image.header
    depth_points.width = depth_image.width
    depth_points.height  = depth_image.height
    depth_points.fields.append(sensor_msgs.msg.PointField(
        name = "x",offset = 0,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
    depth_points.fields.append(sensor_msgs.msg.PointField(
        name = "y",offset = 4,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
    depth_points.fields.append(sensor_msgs.msg.PointField(
        name = "z",offset = 8,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
    depth_points.point_step = 16 
    depth_points.row_step = depth_points.point_step * depth_points.width
    buffer = []
    buffer_rgb = []

    range_img = range2xyz(gt[count])

    for v in range(depth_image.height):
        for u in range(depth_image.width):
            ptx = range_img[v,u,0]
            pty = range_img[v,u,1]
            ptz = range_img[v,u,2]
            buffer.append(struct.pack('ffff',ptx,pty,ptz,1.0))
    depth_points.data = "".join(buffer)
    outbag.write("/lidarsr/prediction", depth_points, t)
    count +=1
    if count > 3:
        break
outbag.close()
