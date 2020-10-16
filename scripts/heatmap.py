from pypcd import pypcd
from readpcd import cal_range
import numpy as np


file_path = '/Users/taoyifu/data/0_prd.pcd'
prd = pypcd.PointCloud.from_path(file_path)
_prd = prd.pc_data.view(np.float32).reshape(prd.pc_data.shape + (-1,))

file_path = '/Users/taoyifu/data/0_gt.pcd'
gt = pypcd.PointCloud.from_path(file_path)
_gt = gt.pc_data.view(np.float32).reshape(gt.pc_data.shape + (-1,))

image_rows_full = 64
image_cols = 1024
dis = np.empty([prd.pc_data.shape[0], 1], dtype=np.float32)
rgb = np.empty([prd.pc_data.shape[0], 1], dtype=np.float32)

diff = np.subtract(_prd,_gt)

for i in range(0,prd.pc_data.shape[0]):
    dis[i] = cal_range(diff[i])

n=prd.pc_data.shape[0]
rgb = np.hstack((np.repeat(0.01, n)[:, np.newaxis], np.repeat(0.01, n)[:, np.newaxis], dis/dis.max()))
encoded_colors = pypcd.encode_rgb_for_pcl((rgb * 255).astype(np.uint8))
new_data = np.hstack((_prd, encoded_colors[:, np.newaxis]))
new_cloud = pypcd.make_xyz_rgb_point_cloud(new_data)
new_cloud.save('new_cloud.pcd')
