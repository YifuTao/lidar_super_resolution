from PIL import Image
import numpy as np
from numpy2pcd import noise_remove
import imageio
import os
from readpcd import xyz2range
def save_grey_img(Images,path,name):
    scale = 255/(Images.max()-Images.min())
    save_video(Images,path,name)

    for i in range(0,Images.shape[0]):
        im = Image.fromarray(Images[i,:,:,0]*scale)
        grey = im.convert("L")
        grey.save(os.path.join(path,name,'%s_%d.png'%(name,i)))


def save_video(Images,path,name):
    
    # listing = os.listdir(path1)

    write_to = os.path.join(path,'%s.mp4'%name)# .format(output) # have a folder of output where output files could be stored.

    writer = imageio.get_writer(write_to, format='mp4', mode='I', fps=20)

    for i in range(0,Images.shape[0]):
        writer.append_data(Images[i])
    writer.close()


def heatmap(img1,img2,path,name):
    rgb = np.zeros([img1.shape[0],img1.shape[1],img1.shape[2],3], dtype=np.float32)
    diff = np.subtract(img1,img2)
    diff = np.abs(diff)
    clamping_threshold_max = diff.max()
    clamping_threshold_min = diff.min()

    #for n in range(0,img1.shape[0]):
    for n in range(0,2):
        for i in range(0,img1.shape[1]):
            for j in range(0,img1.shape[2]):
                scaler = (diff[n][i][j] - clamping_threshold_min)/(clamping_threshold_max - clamping_threshold_min)
                rgb[n,i,j,0] = min(max(min(4.0*scaler - 1.5, -4.0*scaler + 4.5), 0.0), 1.0)    #R
                rgb[n,i,j,1] = min(max(min(4.0*scaler - 0.5, -4.0*scaler + 3.5), 0.0), 1.0)    #G
                rgb[n,i,j,2] = min(max(min(4.0*scaler + 0.5, -4.0*scaler + 2.5), 0.0), 1.0)    #D
        im = Image.fromarray(np.uint8(rgb[n]*255))
        im.save(os.path.join(path,name,'%s_%d.png'%(name,n)))
    
    save_video(rgb,path,name)
    
def main():
    path = '/Volumes/EthanSSD/data'
    gt = np.load(os.path.join(path,'long_experiment_64.npy'))
    prd = np.load(os.path.join(path,'myouster_range_image-UNet-from-16-to-64_prediction.npy'))
    
    cleaned = noise_remove(gt, prd)

    map_path = os.path.join(path,'range_image')
    heatmap(gt,cleaned,map_path,'error')    
    #save_grey_img(gt,path,'gt')
    #save_grey_img(prd,path,'prd')
    #save_grey_img(cleaned,path,'prd_clean')

    
    

    
if __name__ == '__main__':
    main()