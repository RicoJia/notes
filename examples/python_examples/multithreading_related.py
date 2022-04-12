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

if __name__ == "__main__": 
    test_lock()
