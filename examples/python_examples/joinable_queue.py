#!/usr/bin/python3
import multiprocessing

def foo(q):
    print(q.get())
    # will signal the queue of the item's retrieval
    q.task_done()

q = multiprocessing.JoinableQueue()
process1 = multiprocessing.Process(target=foo, args=(q, ))
process1.start()
q.put("asdfa")
q.join()
process1.join()

def bar(q2):
    """

    :q2: TODO
    :returns: TODO

    """
    item = q2.get()
    print(item)

q2 = multiprocessing.Queue()
process2 = multiprocessing.Process(target=bar, args=(q2, ))
process2.start()
q2.put("fff")
process2.join()


