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
    1. In C++, steady clock is always going up, while system clock may be subject to
        - NTP synchronization.
        - timezone, daylight saving time does not affect it since they're wrt UTC

    2. In python:
        - CPU time is: time.process_time()(not affected by NTP)
            - time cpu process a task + system calls of the process in total, NOT INCLUDING SLEEP.
            - (time.clock()) has been removed
        - Wall time: time.time()(affected by NTP) and time.perf_counter()(for short term precise profiling, not affected by NTP)
            - time actually passes, like looking at the time on the wall. Wall time - CPU time = intentional sleep, system wait to get a task processed.  
        - Monotonic Wall time: time.monotonic()
    """
    print(f"wall time: {time.time()}, cpu time: {time.process_time()}")

    start = time.perf_counter()
    time.sleep(1)
    print(f"perf counter: total time elapsed: {time.perf_counter() - start}")

if __name__ == "__main__":
    test_cpu_time()

