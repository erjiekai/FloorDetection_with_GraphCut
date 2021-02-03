from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
import sys

def readFile(fname):
    try:
        print("reading: " + fname)
        data = np.loadtxt(fname)
        print(data.shape)
        return data
    except IOError:
        print(fname + " not found!!")
        return []

size = 30
asgf = readFile("original/asgf.txt")
new_asgf = asgf[120:120+size,150:150+size]
np.savetxt("asgf.txt", new_asgf, fmt='%1.8e', header='file automatically generated\nsize: '+str(size)+' '+str(size), comments='#')

camera = readFile("original/camera_array.txt")
new_camera = camera[120:120+size,150:150+size*3]
np.savetxt("camera_array.txt", new_camera, fmt='%1.8e', header='file automatically generated\nsize: '+str(size)+' '+str(size*3), comments='#')