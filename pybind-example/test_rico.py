from numpy.core import arrayprint
import build.example as example
import numpy as np
import threading
import pickle
import time

j = 200
test_done = False

def pickle_converter_callback(): 
    global test_done 
    test_done = True

for i in range(100): 
    example.pybind_test(pickle_converter_callback)
    while test_done == False: 
        pass

    infile = open("pickled_test1.txt",'rb')
    new_dict = pickle.load(infile)
    print(new_dict)
    infile.close()

    infile = open("pickled_test2.txt",'rb')
    new_dict = pickle.load(infile)
    print(new_dict)
    time.sleep(0.1)


