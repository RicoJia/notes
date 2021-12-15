import threading
import time


def foo(ev):
    print(f"flag: {ev.isSet()}")
    ev.wait(20) #timeout
    print(f"flag: {ev.isSet()}")

ev = threading.Event()
th1 = threading.Thread(name="Th1", target=foo, args=(ev,))
th1.start()
time.sleep(1)
ev.set()

