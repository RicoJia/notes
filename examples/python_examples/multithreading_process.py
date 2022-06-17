#!/usr/bin/python3
import threading
import concurrent.futures
import time

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

def test_multiple_threads_queue():
    """
    1. thread.setDaemon(True) will make a daemon thread,which automatically & immediately joins when the main thread is joined.
        - a regular non-daemon thread will have to wait until it finishes
    2. task_done() signifying one item has been processed to the queue
    3. join() waits for a thread to finish
    4. About shutdown:
        1. Common Practices
            - send a sentinel value with the message is common practice
            - Or have a special function that sets a flag to 0. 
        2. Pain: if you don't signal the thread, the thread will never know when to finish. Also, __del__ is not the way to go
            - garbage collection happens when reference count = 1. But 
                - when a daemon thread is still running, the enclosing object skips destruction, and it's garbage collected when exiting the program
                - when a non-daemon thread is still running, the main thread will hang because it will wait for the thread to finish
            - notes: 
                - x.__del__() may not be called during program exit.
                - del x doesn’t directly call x.__del__() — the former decrements the reference count for x by one, and the latter is only called when x’s reference count reaches zero.
        3. you can't forcibly kill a thread like killing a process (implemented on SIGTERM)
    """
    from queue import Queue, Empty
    from threading import Thread
    class Example:
        def __init__(self):
            self.should_run = True
            self.q = Queue()
            self.th = Thread(target = self.__work)
            self.th.start()
        def __work(self):
            while self.should_run:
                try: 
                    self.q.get(timeout=1)
                    self.q.task_done()
                except Empty:
                    print("work should_run: ", self.should_run)
                    pass
        def put(self, value):
            self.q.put(value)
        def shutdown(self):
            self.should_run = False
        def __del__(self):
            self.should_run = False
            print("del should_run: ", self.should_run)
            self.q.join()
    f = Example()
    f.put(1)
    # f.shutdown()

def test_daemon_thread():
    """
    1. Daemon Thread joins after the process is joined
        - shutdown: 
            1. If daemon thread has finished, join() succeeds and calls __del__ of daemon object
            2. if not, it will be garbage collected and its object will live until then, whose __del__ is not guaranteed to be called
    2. Problem: printing stuff in daemon thread is dangerous, may not be ablt to get lock for stdout at shutdown
    """
    from threading import Thread
    import queue
    class TestDaemon:
        def __init__(self):
            self.queue = queue.Queue()
            dth = Thread(target = self.daemon_func)
            dth.setDaemon(True)
            dth.start()
        def daemon_func(self):
            while True:
                time.sleep(0.01)
        def __del__(self):
            self.queue.join()

    t = TestDaemon()

import signal
import os
from multiprocessing import  Process
def test_subprocess(): 
    """
    1. Subprocess is to open anything over command line as a process. multiprocessing allows to divide a program
    """
    pass
def test_process(): 
    """
    Theory: 
        1. you have SIGUSR1, SIGUSR2 to communicate between processes. 
        2. Use signal handler like below
            - Linux has a "process group", parent process, child process belongs to the same process group. 
            - os.killpg(pid_group, signal)
                - pgid = os.getpgid(os.getpid())
            - os.kill(pid, signal)
                - os.getppid() is to get parent pid
                - os.getpid() will return the pid
                - process.pid can be used to get pid
        3. launch process, https://zhuanlan.zhihu.com/p/64702600
    """
    def getter(): 
        shutdown = False
        def sig_handler_usr2(signum, frame):
            nonlocal shutdown
            shutdown = True
            print("usr2 is called")    
        signal.signal(signal.SIGUSR2, sig_handler_usr2)
        print(f"getter sleeping, os pid: {os.getppid()}")
        while not shutdown:
            time.sleep(0.5)

    def sig_handler_usr2_parent(signum, frame):
        print("parent sigusr2")
        pass
    signal.signal(signal.SIGUSR2, sig_handler_usr2_parent)
    pset = Process(target=getter,args=()) #实例化进程对象
    pset.start()
    time.sleep(1)
    # The default behavior of a signal with no handler, e.g., os.killpg(os.getppid(), signal.SIGUSR2) is to print out message: "User defined signal1"
    print("time to kill")
    # Get current process group id
    # method 1: send signal to group
    pgid = os.getpgid(os.getpid())
    os.killpg(pgid, signal.SIGUSR2)
    # method 2: send signal to a specific process
    os.kill(pset.pid, signal.SIGUSR2)
    pset.join()

def test_multiprocess_queue(): 
    """
    # cv2.imread  cannot work with non-english filenames
    1. Queue(max_size)
    2. queue.put(), queue.get() by default will block the main thread and wait for an item to come
        - multiprocessing.Queue.get() will yield Queue.Empty if empty
    3. queue.put_nowait(), queue.get_nowait() will not block
    """
    from multiprocessing import Process, Queue
    import os, time, random
    # 写数据进程执行的代码:
    def _write(q,urls):
        print('Process(%s) is writing...' % os.getpid())
        for url in urls:
            q.put(url)
            print('Put %s to queue...' % url)
    # 读数据进程执行的代码:
    def _read(q):
        print('Process(%s) is reading...' % os.getpid())
        while True:
            url = q.get(True)
            print('Get %s from queue.' % url)

    # 父进程创建Queue，并传给各个子进程：
    q = Queue()
    _writer1 = Process(target=_write, args=(q,['url_1', 'url_2', 'url_3']))
    _writer2 = Process(target=_write, args=(q,['url_4','url_5','url_6']))
    _reader = Process(target=_read, args=(q,))
    # 启动子进程_writer，写入:
    _writer1.start()
    _writer2.start()
    # 启动子进程_reader，读取:
    _reader.start()
    # 等待_writer结束:
    _writer1.join()
    _writer2.join()
    time.sleep(2)
    # _reader进程里是死循环，无法等待其结束，只能强行终止:
    _reader.terminate()

    # 2 - use a message to terminate the process
    import multiprocessing
    def consumer(queue):
        while True:
            msg = queue.get()
            print(msg)
            if (msg == 'DONE'):
                break

    def producer(count, queue):
        for i in range(count):
            queue.put(i)
        queue.put('DONE')

    queue = multiprocessing.Queue()
    consumer_process = multiprocessing.Process(target=consumer, args=[queue])
    consumer_process.daemon = True
    consumer_process.start()
    count = 10**4
    producer(count, queue)
    consumer_process.join()
    print("Sent {0} numbers to Queue()".format(count))

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

def test_process_scanning():
    import psutil
    for proc in psutil.process_iter():
        print(proc.name().lower())
        # try:
        #     # Check if process name contains the given name string.
        #     if processName.lower() in proc.name().lower():
        #         return True
        # except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        #     pass

if __name__ == "__main__": 
    # test_lock()
    # test_process()
    # test_forking()
    # test_multiprocess_queue()
    # test_process_scanning()
    test_multiple_threads_queue()
    # test_daemon_thread()
