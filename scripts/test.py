import numpy as np
from pypcd import pypcd
import os
import matplotlib.pyplot as plt
from os.path import join

path = "/home/yifu/data/pcd/input"
for count, filename in enumerate(os.listdir(path)):
    index =  filename[:-4]
    new = '{:04d}.pcd'.format(int(index))
    os.rename(join(path,filename),join(path,new))