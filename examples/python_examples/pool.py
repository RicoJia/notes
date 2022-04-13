#!/usr/bin/python3
import time 
from multiprocessing import Pool

def test(x): 
    print(f"start process: ${x}")
    time.sleep(x)
    print(f"end_process f{x}")

pool = Pool()
pool.map(test, range(0,5))  # one input is from the iterable
