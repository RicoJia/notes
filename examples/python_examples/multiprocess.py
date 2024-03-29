#!/usr/bin/python3
import threading
import concurrent.futures
import time

################################################################
## MultiProcessing
################################################################
import signal
import os
from multiprocessing import  Process, Manager
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

def test_manager_and_shared_list():
    def worker(shared_list):
        shared_list.append(1)
        shared_list.append(5)

    with Manager() as manager:
        shared_list = manager.list()
        p = Process(target=worker, args=(shared_list, ))
        p.start()
        p.join()
        print(shared_list) 
        
def test_process_pipe():
    def worker(in_p):
        # you can do in_p.poll() as well
        msg = in_p.recv()
        print("msg: ", msg)
        
    def client(out_p):
        import time
        time.sleep(2)
        out_p.send("herro")
        
    import multiprocessing
    conn1, conn2 = multiprocessing.Pipe()
    worker = multiprocessing.Process(target=worker, args=(conn1, ))
    client = multiprocessing.Process(target=client, args=(conn2, ))

    worker.start()
    client.start()

    worker.join()
    client.join()

        

def test_simple_server_and_client(is_client):
    '''
    This example is a great tool for inter-process communication.   
    BELOW EXAMPLE CAN BE EXPANDED INTO DIFFERENT PROGRAMS, and we can use UNIX or TCP sockets!
    1. Listener: 
        - Can be unix, tcp sockets. The type can be inferred from the format of the address
        - Try unix sockets
    '''
    from multiprocessing.connection import Listener, Client, Connection
    AUTH_KEY = b"rico is great"
    # 1. tcp sockets
    # ADDR = ('', 15000)
    # 2. Unix socket, file doesn't have to exist
    ADDR = '/tmp/servconn3'
    if not is_client:
        # Note the Listener.accept() only returns the socket, no address!
        work_listener = Listener(ADDR, authkey=AUTH_KEY)
        while True:
            worker_connection: Connection = work_listener.accept()
            msg = worker_connection.recv()
            print(f"received message from : {msg}")
            worker_connection.send("Server has received your msg")
    else: 
        client_server = Client(ADDR, authkey=AUTH_KEY)
        client_server.send("I'm a bird")
        ret_msg = client_server.recv()
        print(ret_msg) 

if __name__ == "__main__": 
    # test_lock()
    # test_process()
    # test_forking()
    # test_multiprocess_queue()
    # test_process_scanning()
    # test_multiple_threads_queue()
    # test_daemon_thread()

    # test_threading_timer()
    # test_process_pipe()
    test_manager_and_shared_list()
    import argparse
    parser = argparse.ArgumentParser()
    # interesting, -- does make a difference
    parser.add_argument("--client", action="store_true", default=False, required=False)
    args = parser.parse_args()
    if args.client:
        print("client")
    else:
        print("server")

    # test_simple_server_and_client(args.client)
