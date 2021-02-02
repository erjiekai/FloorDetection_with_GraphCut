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

fig, axs = plt.subplots(3,3)

""" 3D data """
# data = readFile("camera_array.txt")
# np.set_printoptions(threshold=sys.maxsize)
# # print(data[:,0])

# rows = data.shape[0]
# cols = int(data.shape[1]/3)
# image_max = 255

# array = np.empty((rows,cols,3))

# for i in range(rows):
#     for j in range(cols):
#         array[i,j,0]=(data[i,j*3]/image_max)
#         array[i,j,1]=(data[i,j*3+1]/image_max)
#         array[i,j,2]=(data[i,j*3+2]/image_max)
#         # print(int(array[i,j,0]),int(array[i,j,1]),int(array[i,j,2]))
# print(array.shape)

# # image = PIL.Image.fromarray(image_data, "RGB")

# axs[0, 0].imshow(array)
# axs[0,0].set_title('camera')

""" colour depth data """
# data2 = readFile("colour_array.txt")
# np.set_printoptions(threshold=sys.maxsize)
# # print(data2[:,0])

# rows = data2.shape[0]
# cols = int(data2.shape[1]/3)
# image_max = 255

# array2 = np.empty((rows,cols,3))

# for i in range(rows):
#     for j in range(cols):
#         array2[i,j,0]=(data2[i,j*3]/image_max)
#         array2[i,j,1]=(data2[i,j*3+1]/image_max)
#         array2[i,j,2]=(data2[i,j*3+2]/image_max)
#         # print(int(array2[i,j,0]),int(array2[i,j,1]),int(array2[i,j,2]))
# print(array2.shape)

# axs[0, 1].imshow(array2)
# axs[0, 1].set_title('Colour Depth')

""" 2D data """
# data = readFile("asgf.txt")
# np.set_printoptions(threshold=sys.maxsize)
# print(data.max())

# rows = data.shape[0]
# cols = int(data.shape[1])

# axs[0, 2].imshow(data)
# axs[0, 2].set_title('asgf')

""" processed asgf data """
asgf = readFile("asgf_processed.txt")

rows = asgf.shape[0]
cols = int(asgf.shape[1])

axs[0, 0].imshow(asgf)
axs[0, 0].set_title('processed asgf')

""" asgf left diff data """
asgf_left_diff = readFile("asgf_left_diff.txt")

rows = asgf_left_diff.shape[0]
cols = int(asgf_left_diff.shape[1])

axs[0, 1].imshow(asgf_left_diff)
axs[0, 1].set_title('asgf left difference')

""" asgf up diff data """
asgf_up_diff = readFile("asgf_up_diff.txt")

rows = asgf_up_diff.shape[0]
cols = int(asgf_up_diff.shape[1])

axs[0, 2].imshow(asgf_up_diff)
axs[0, 2].set_title('asgf up difference')



""" black and white data """
bw = readFile("bw.txt")

rows = bw.shape[0]
cols = int(bw.shape[1])

axs[1, 0].imshow(bw)
axs[1, 0].set_title('black and white')

""" bw left diff data """
bw_left_diff = readFile("bw_left_diff.txt")

rows = bw_left_diff.shape[0]
cols = int(bw_left_diff.shape[1])

axs[1, 1].imshow(bw_left_diff)
axs[1, 1].set_title('bw left difference')

""" bw up diff data """
bw_up_diff = readFile("bw_up_diff.txt")

rows = bw_up_diff.shape[0]
cols = int(bw_up_diff.shape[1])

axs[1, 2].imshow(bw_up_diff)
axs[1, 2].set_title('bw up difference')





"""  left diff data """
left_diff = readFile("left_diff.txt")

rows = left_diff.shape[0]
cols = int(left_diff.shape[1])

axs[2, 0].imshow(left_diff)
axs[2, 0].set_title('left difference')

"""  up diff data """
up_diff = readFile("up_diff.txt")

rows = up_diff.shape[0]
cols = int(up_diff.shape[1])

axs[2, 1].imshow(up_diff)
axs[2, 1].set_title('up difference')


""" 2D data """
data2 = readFile("saturated_vertices.txt")

rows = data2.shape[0]
cols = int(data2.shape[1])

axs[2, 2].imshow(data2)
axs[2, 2].set_title('saturated_vertices')


plt.show()
