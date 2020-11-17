import numpy as np
from pypcd import pypcd
import os
import matplotlib.pyplot as plt
from os.path import join
import time
import sys
import rosbag
import ros_numpy
import struct
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

from readpcd import offset_range_img,image_rows_full,image_cols,save_grey_img,create_4dir,progressbar
from readbag import get_topics_types
from numpy2pcd import range2xyz

header = Header()
header.frame_id = "map"

fields = [PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        ]   # from point_cloud2.py def create_cloud_xyz32


def save_bag(range_imgs):
    bag = rosbag.Bag('test.bag','w')
    bag_topic = '/object_scan/raw_cloud'

    for i in range(0,range_imgs.shape[0]):
        clouds = range2xyz(range_imgs[i])
        cloud_msg = pc2.create_cloud_xyz32(header, clouds.reshape(-1,3).tolist()) #TODO unsure about the order when I convert 64x1024 to 63000
        bag.write(bag_topic, cloud_msg)

def create_pc2_msg(_header,_height,_width,_fields,_point_step,_data):
    return PointCloud2(header=_header,
                        height=_height,
                        width=_width,
                        is_dense=True,
                        is_bigendian=False,
                        fields=_fields,
                        point_step=_point_step,
                        row_step=_point_step * _width,
                        data=_data)

def convert_binary(range_img,height,width):
    buffer=[]
    for i in range(height):
        for j in range(width):
            buffer.append(struct.pack('fff',range_img[i,j,0],range_img[i,j,1],range_img[i,j,2]))
    return "".join(buffer)

def add_prd(path,range_imgs):
    input_bag = rosbag.Bag(path)
    topics,types = get_topics_types(input_bag)

    output_bag = rosbag.Bag('test.bag','w')
    bag_topic = '/object_scan/raw_cloud'

    count = 0
    for topic, msg, t in input_bag.read_messages(topics=[bag_topic, ]):
        
        cloud = range2xyz(range_imgs[count])
        cloud_binary = convert_binary(cloud,msg.height,msg.width)
        cloud_msg = create_pc2_msg(msg.header,msg.height,msg.width,fields,16,cloud_binary)#clouds.reshape(-1,3))

        # cloud_msg = pc2.create_cloud_xyz32(msg.header, clouds.reshape(-1,3).tolist()) #TODO unsure about the order when I convert 64x1024 to 63000
        output_bag.write(bag_topic, cloud_msg)
        count += 1
        progressbar(count,input_bag.get_message_count()/len(topics))


def main():
    bag_path = '/home/yifu/data/bagfiles/NewerCollege_short_raw_2.bag'
    np_path = '/home/yifu/data/bagfiles/NewerCollege_short_raw_2/prd.npy'
    range_imgs = np.load(np_path)
    add_prd(bag_path,range_imgs)
    # save_bag(range_imgs)
    



if __name__ == '__main__':
    main()