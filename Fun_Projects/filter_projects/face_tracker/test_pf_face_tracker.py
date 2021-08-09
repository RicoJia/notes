import numpy as np
import cv2
from face_tracker import face_tracker_pf
import time # TODO
from PIL import Image
import os
import matplotlib.pyplot as plt

# setup initial location of window by using two corner points
corner_points = [(0,0), (3,3)]

# open image
def open_img(pic_file_path):
    # you will see 4 channels, the 4th channel is "trasparency" from png.
    #return an Image object
    return Image.open(pic_file_path)

def to_np_array(p): #p is Image object
   return np.array(p) 

def get_state_ranges(img): 
    print (img.shape)
    # ranges is [(upper_lim, lower_lim, standard_deviation_of_noise), ...]
    height = img.shape[0]
    width = img.shape[1]
    x_range = (0, width, 0.6)
    y_range = (0, height, 0.6)  #? 
    vx_range = (-width, width, 0.6)      #assume face can flash across the window in 1s
    vy_range = (-height, height, 0.6)
    hx_range = (0, width, 0.6)
    hy_range = (0, height, 0.6)
    at_dot_range = (0,2, 0.6)        #scale change
    return [x_range, y_range, vx_range, vy_range, hx_range, hy_range, at_dot_range]


path = os.path.dirname(os.path.abspath(__file__)) + "/test.png"
img = to_np_array(open_img(path))[:,:,:3]
ranges = get_state_ranges(img)

# initialize face tracker
PARTICLE_NUM = 2  
SCALE_CHANGE_DISTURB = 0.001
VELOCITY_DISTURB = 40
FRAME_RATE = 30.0

#test
tracker_input = {"ranges":ranges, "PARTICLE_NUM":PARTICLE_NUM, "SCALE_CHANGE_DISTURB":SCALE_CHANGE_DISTURB, "VELOCITY_DISTURB":VELOCITY_DISTURB, "FRAME_RATE":FRAME_RATE, "ROI_corner_points":corner_points, "SIGMA_WEIGHT" : 0.2, "initial_frame" : img}

print(img[0,0,:])
tracker = face_tracker_pf(tracker_input)
# after initializing ROI, run one iteration
return_state = tracker.run_one_iteration(img)
print(img[0,0,:])

