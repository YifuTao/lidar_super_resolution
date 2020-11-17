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


from numpy2pcd import get_low_res_input,noise_remove, range2xyz
from readpcd import offset_range_img,image_rows_full,image_cols,save_grey_img,create_4dir,progressbar


def get_topics_types(bag):
    topics = bag.get_type_and_topic_info()[1].keys()
    types = []
    for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
        types.append(bag.get_type_and_topic_info()[1].values()[i][0])
    return topics,types

def readbag(path,ros_topic):
    # bag = rosbag.Bag('/home/yifu/data/bagfiles/NewerCollege_short_raw_2.bag')
    bag = rosbag.Bag(path)
    topics,types = get_topics_types(bag)
    range_image_array = np.empty([0, image_rows_full, image_cols, 1], dtype=np.float32)
    for topic, msg, t in bag.read_messages(topics=[ros_topic, ]):
        lidar = pc2.read_points(msg)
        points = np.array(list(lidar))
        # TODO create pcd files to test the algorithm
        range_image = offset_range_img(points)
        range_image_array = np.append(range_image_array, range_image, axis=0)
        progressbar(range_image_array.shape[0],bag.get_message_count()/len(topics))
    bag.close()
    return range_image_array


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

def add_prd(path,gt,_input,prd):
    input_bag = rosbag.Bag(path)
    topics,types = get_topics_types(input_bag)

    output_bag = rosbag.Bag('test.bag','w')
    bag_topic = '/object_scan/raw_cloud'

    count = 0
    for topic, msg, t in input_bag.read_messages(topics=[bag_topic, ]):
        # prd
        cloud = range2xyz(prd[count])
        cloud_binary = convert_binary(cloud,msg.height,msg.width)
        cloud_msg = create_pc2_msg(msg.header,msg.height,msg.width,fields,16,cloud_binary)#clouds.reshape(-1,3))
        # TODO unsure about the order when I convert 64x1024 to 63000
        output_bag.write(bag_topic, cloud_msg)
        # gt
        cloud = range2xyz(gt[count])
        cloud_binary = convert_binary(cloud,msg.height,msg.width)
        cloud_msg = create_pc2_msg(msg.header,msg.height,msg.width,fields,16,cloud_binary)
        # input
        cloud = range2xyz(_input[count])
        cloud_binary = convert_binary(cloud,msg.height/4,msg.width)
        cloud_msg = create_pc2_msg(msg.header,msg.height/4,msg.width,fields,16,cloud_binary)
        # TODO unsure about the order when I convert 64x1024 to 63000
        output_bag.write(bag_topic, cloud_msg)
        count += 1
        progressbar(count,input_bag.get_message_count()/len(topics))


def main():
    ## read bag
    '''
    bag_path = '/home/yifu/data/bagfiles/NewerCollege_short_raw_2.bag'
    folder_path = bag_path[:-4]
    if not os.path.exists(folder_path):
        os.mkdir(folder_path)
    print('folder path:%s'%folder_path)
    create_4dir(join(folder_path,'range image'))
    bag_topic = '/object_scan/raw_cloud'
    range_image_array = readbag(bag_path,bag_topic)
    save_grey_img(range_image_array,join(folder_path,'range image'),'gt')
    np.save(join(folder_path,'gt.npy'), range_image_array)
    '''
    ## save bag
    bag_path = '/home/yifu/data/bagfiles/NewerCollege_short_raw_2.bag'
    gt_path = '/home/yifu/data/bagfiles/NewerCollege_short_raw_2/gt.npy'
    gt = np.load(gt_path)
    low_res_input = get_low_res_input(gt)
    prd_path = '/home/yifu/data/bagfiles/NewerCollege_short_raw_2/prd.npy'
    prd = np.load(prd_path)
    prd_cleaned = noise_remove(gt, prd)

    
    add_prd(bag_path,gt,low_res_input,prd_cleaned)
    # save_bag(range_imgs)


if __name__ == '__main__':
    main()