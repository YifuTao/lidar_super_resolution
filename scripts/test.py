import numpy as np
from pypcd import pypcd
import os

# generate a 3d data set containing waves like the sea
# more or less ;)

# generate a grid of numbers for x and z
x = np.linspace(-10, 10, num=100)
z = np.linspace(-10, 10, num=100)
xx, zz = np.meshgrid(x, z)

# calculate "waves" for the grid
yy = np.sin(xx) + np.cos(zz) + 3

# flatten the arrays
xx = xx.astype(np.float32).reshape(1, -1)
yy = yy.astype(np.float32).reshape(1, -1)
zz = zz.astype(np.float32).reshape(1, -2)

# the number of points
n = yy.size

# Generate different blue values depending on the y value
blue = (yy/np.max(yy))

# Make rgb columns for every point
rgb = np.hstack((np.repeat(0.01, n)[:, np.newaxis], np.repeat(0.01, n)[:, np.newaxis], blue.T))

# Next step: Store the new points in a pcd file

# Encode the colors (distinct red, green and blue color columns => one single column) 
encoded_colors = pypcd.encode_rgb_for_pcl((rgb * 255).astype(np.uint8))

# hstack the X, Y, Z, RGB columns
new_data = np.hstack((xx.T, yy.T, zz.T, encoded_colors[:, np.newaxis]))
_new_data = np.hstack((xx.T, yy.T, zz.T))


# Use the pypcd utility function to create a new point cloud from ndarray
new_cloud = pypcd.make_xyz_rgb_point_cloud(new_data)
_new_cloud = pypcd.make_xyz_point_cloud(_new_data)


new_cloud.save('new_cloud.pcd')


normalize_ratio = 100.0
image_cols = 1024

project_name = 'SuperResolution'
home_dir = os.path.expanduser('~')
origImages = np.load(os.path.join(home_dir, 'Documents', project_name, 'ouster_range_image.npy')) # 
# predicted image: n x rows x cols x 2
predImages = np.load(os.path.join(home_dir, 'Documents', project_name, 'ouster_range_image-from-16-to-64_prediction.npy')) * normalize_ratio # 

image_rows_low = 16 # 8, 16, 32
image_rows_high = 64 # 16, 32, 64
image_rows_full = 64
# ouster
ang_res_x = 360.0/float(image_cols) # horizontal resolution
ang_res_y = 33.2/float(image_rows_high-1) # vertical resolution
ang_start_y = 16.6 # bottom beam angle
sensor_noise = 0.03
max_range = 80.0
min_range = 2.0

upscaling_factor = int(image_rows_high / image_rows_low)
low_res_index = range(0, image_rows_high, upscaling_factor)
predImages[:,low_res_index,:,0:1] = origImages[:,low_res_index,:,0:1] # copy some of beams from origImages to predImages
origImagesSmall = np.zeros(origImages.shape, dtype=np.float32) # for visualizing NN input images
origImagesSmall[:,low_res_index] = origImages[:,low_res_index]

predImagesNoiseReduced = np.copy(predImages[:,:,:,0:1])

# remove noise
if len(predImages.shape) == 4 and predImages.shape[-1] == 2:
    noiseLabels = predImages[:,:,:,1:2]
    predImagesNoiseReduced[noiseLabels > predImagesNoiseReduced * 0.03] = 0 # after noise removal
    predImagesNoiseReduced[:,low_res_index] = origImages[:,low_res_index]

predImages[:,:,:,1:2] = None
