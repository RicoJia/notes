#!/usr/bin/python3
import time 
def test_time_str(): 
    time_str = time.strftime("%Y, %m, %d")  #returns string that corresponds to the format strings
    print(time_str)
    str_1 = "123"
    str_2 = "123"
    str_3 = "123"
    str_4 = "123"

def test_cpu_time(): 
    """
    CPU time (time.clock()) is the time cpu process a task + system calls of the process in total 
    Wall time (time.time())is the time actually passes, like looking at the time on the wall
    Wall time - CPU time = intentional sleep, system wait to get a task processed 
    """
    print(f"wall time: {time.time()}, cpu time: {time.process_time()}")

if __name__ == "__main__":
    test_cpu_time()

