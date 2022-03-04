from numpy.core import arrayprint
import build.example as example
import numpy as np
import threading
import pickle
import time

j = 200
test_done = False
for i in range(100): 
    example.pybind_test()

    infile = open("hee.txt",'rb')
    new_dict = pickle.load(infile)
    print(new_dict)
    infile.close()



