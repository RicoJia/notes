import threading, time

def func2():
    """TODO: Docstring for func2.
    :returns: TODO

    """
    raise RuntimeError("ERROR")

def func1(): 
    try:
        func2()
    except: 
        print("1")

def test_excepthook():
    """
    1. Does not work on Ipython, Jupyter Notebook
    """
    import sys
    def my_excepthook(ex_cls, ex, tb):
        msg = "Oops! There's an Error.\n"
        print(msg)

    sys.excepthook = my_excepthook

    print(1/0)

def test_multithreaded_excepthook():
    """
    Thread Excepthook is done on a process base. 
    - you can see thread name.
    """
    def custom_hook(args):
        # report the failure
        print(f'Thread failed: {args.exc_value}')
        print("args: ", args)

    # example of an unhandled exception in a thread
    from time import sleep
    import threading

    # target function that raises an exception
    def work():
        print('Working...')
        sleep(1)
        # rise an exception
        raise Exception('Something bad happened')

    # set the exception hook
    threading.excepthook = custom_hook
    # create a thread
    thread = threading.Thread(target=work)
    thread2 = threading.Thread(target=work)
    # run the thread
    thread.start()
    thread2.start()
    # wait for the thread to finish
    thread.join()
    thread2.join()
    # continue on
    print('Continuing on...')

if __name__ == "__main__":
    # func1()
    # test_excepthook()
    test_multithreaded_excepthook()


