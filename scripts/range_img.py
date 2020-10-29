from PIL import Image
import numpy as np
from numpy2pcd import noise_remove
import imageio
import seaborn as sns
import matplotlib.pyplot as plt

from os.path import join 
from readpcd import xyz2range
from numpy2pcd import flip,get_low_res_index

def save_grey_img(Images,path,name):
    scale = 255/(Images.max()-Images.min())
    save_video(Images,path,name)

    for i in range(0,Images.shape[0]):
        im = Image.fromarray(Images[i,:,:,0]*scale)
        grey = im.convert("L")
        grey.save(join(path,name,'%s_%d.png'%(name,i)))


def save_video(Images,path,name):
    
    # listing = os.listdir(path1)

    write_to = join(path,'%s.mp4'%name)# .format(output) # have a folder of output where output files could be stored.

    writer = imageio.get_writer(write_to, format='mp4', mode='I', fps=20)

    for i in range(0,Images.shape[0]):
        writer.append_data(Images[i])
    writer.close()

def plot_histogram(array):
    # rng = np.random.RandomState(10)  # deterministic random data
    # a = np.hstack((rng.normal(size=1000),
    #             rng.normal(loc=5, scale=2, size=1000)))
    _ = plt.hist(array, bins='auto')  # arguments are passed to np.histogram
    plt.title("Histogram")
    #Text(0.5, 1.0, "Histogram with 'auto' bins")
    plt.show()

def heatmap(img1,img2,path,name):
    rgb = np.zeros([img1.shape[0],img1.shape[1],img1.shape[2],3], dtype=np.float32)
    diff = np.subtract(img1,img2)
    diff = np.abs(diff)
    for i in range(0,64):
        for j in range(0,1024):
            if img2[0,i,j]==0:
                diff[0,i,j]=0
    # plot_histogram(diff[0].reshape(-1))
    # clamping_threshold_max = diff.max()
    clamping_threshold_max = 1
    clamping_threshold_min = diff.min()
    print('heatmap range is 0 to %f m'%(clamping_threshold_max-clamping_threshold_min))

    # print colour scale
    for i in range(0,64):
        for j in range(0,1024):
            scaler = j/1024.0
            rgb[0,i,j,0] = min(max(min(4.0*scaler - 1.5, -4.0*scaler + 4.5), 0.0), 1.0)    #R
            rgb[0,i,j,1] = min(max(min(4.0*scaler - 0.5, -4.0*scaler + 3.5), 0.0), 1.0)    #G
            rgb[0,i,j,2] = min(max(min(4.0*scaler + 0.5, -4.0*scaler + 2.5), 0.0), 1.0)    #D
    im = Image.fromarray(np.uint8(rgb[0]*255))
    im.save(join(path,name,'colour scale.png'))
    
    for n in range(0,img1.shape[0]):
    #for n in range(0,20):
        for i in range(0,img1.shape[1]):
            for j in range(0,img1.shape[2]):
                if (img2[n,i,j]==0 or img1[n,i,j]==0):
                    rgb[n,i,j,0] = 0
                    rgb[n,i,j,1] = 0
                    rgb[n,i,j,2] = 0
                else:
                    scaler = (diff[n][i][j] - clamping_threshold_min)/(clamping_threshold_max - clamping_threshold_min)
                    rgb[n,i,j,0] = min(max(min(4.0*scaler - 1.5, -4.0*scaler + 4.5), 0.0), 1.0)    #R
                    rgb[n,i,j,1] = min(max(min(4.0*scaler - 0.5, -4.0*scaler + 3.5), 0.0), 1.0)    #G
                    rgb[n,i,j,2] = min(max(min(4.0*scaler + 0.5, -4.0*scaler + 2.5), 0.0), 1.0)    #B
        if (n%20==0):
            print(n)        


        im = Image.fromarray(np.uint8(rgb[n]*255))
        im.save(join(path,name,'%s_%d.png'%(name,n)))
    
    save_video(rgb,path,name)
    
def main():
    path = '/home/yifu/data'
    normalize_ratio = 100.0

    gt = np.load(join(path,'long_experiment_64.npy'))
    prd = np.load(join(path,'myouster_range_image-UNet-from-16-to-64_prediction.npy'))*normalize_ratio
    gt = flip(gt)
    prd = flip(prd)
    cleaned = noise_remove(gt, prd)

    low_res_index = get_low_res_index()
    low_res_input = np.zeros(gt.shape, dtype=np.float32) # for visualizing NN input images
    low_res_input[:,low_res_index] = gt[:,low_res_index]

    map_path = join(path,'range_image')
    heatmap(gt,cleaned,map_path,'error')    
    #save_grey_img(gt,map_path,'gt')
    #save_grey_img(prd,map_path,'prd')
    #save_grey_img(cleaned,map_path,'prd_clean')

    
    

    
if __name__ == '__main__':
    main()