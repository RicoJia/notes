#!/usr/bin/python3
import threading
import concurrent.futures

def test_event(): 
    def foo(ev):
        print(f"flag: {ev.isSet()}")
        ev.wait(20) #timeout
        print(f"flag: {ev.isSet()}")

    ev = threading.Event()
    th1 = threading.Thread(name="Th1", target=foo, args=(ev,))
    th1.start()
    time.sleep(1)
    ev.set()


def test_threadpool(): 
    def foo(i): 
        print(i)

    with concurrent.futures.ThreadPoolExecutor() as executor: 
        futures = []
        for i in range(5): 
            futures.append(executor.submit(foo, i=i))
        for future in futures: 
            future.result()

def test_lock(): 
    lock = threading.Lock()
    name = 10
    def worker(): 
        lock.acquire()
        nonlocal name
        name += 1
        print(name)
        lock.release()

        with lock: 
            print (name)

    for j in range(2): 
        t = threading.Thread(target = worker)
        t.start()

    main_thread = threading.currentThread()
    for t in threading.enumerate(): 
        if t is not main_thread: 
            t.join()

import signal
import time
import os
from multiprocessing import  Process
def test_process(): 
    """
    Theory: 
        1. you have SIGUSR1, SIGUSR2 to communicate between processes. 
        2. Use signal handler like below
        3. launch process, https://zhuanlan.zhihu.com/p/64702600
    """
    def setter(): 
        print("setter")
        time.sleep(1)
        # get parent pid
        # The default behavior of a signal with no handler, e.g., os.killpg(os.getppid(), signal.SIGUSR2) 
        # is to print out message: "User defined signal1"
        os.killpg(os.getppid(), signal.SIGUSR2)
    pset = Process(target=setter,args=()) #实例化进程对象
    pset.start()
    shutdown = False
    def sig_handler_usr2(signum, frame):
        nonlocal shutdown
        shutdown = True
        print("usr2 is called")
    signal.signal(signal.SIGUSR2, sig_handler_usr2)
    print("sleeping")
    while not shutdown:
        time.sleep(0.5)

    pset.join()

def test_multiprocess_queue(): 
    """
    1. Queue(max_size)
    2. queue.put(), queue.get() by default will block the main thread and wait for an item to come
    3. queue.put_nowait(), queue.get_nowait() will not block
    """
    from multiprocessing import Queue
    q = Queue(4)
    q.put(1)
    
def test_forking(): 
    """
    1. Forking is to spawn a child process from the "fork" point. All data is copied to the child process. 
        - A process is a program that gets loaded from disk to stack, and gets executed. It has data, kernel state that holds all data necessary. 
        - Parent PID will be non-zero
    2. so if you do forking, each child process will spawn the same number of processes. 
        - so the total number of forked process will be 2^(number of fork() statements)
        - for a child process, your pid will remain 0 until you spawn someone.
    """
    import os 
    import time
    num = 7 
    # here we see 32 pids
    pid = os.fork()
    print(pid)
    pid = os.fork()
    print(pid)
    # pid3 = os.fork()
    # pid4 = os.fork()
    time.sleep(10)

if __name__ == "__main__": 
    # test_lock()
    # test_process()
    test_forking()
