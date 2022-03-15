import numpy as np
def test_scope():
    """
    1. Function and local scopes are the same as cpp
    2. If doesn't count as a "new scope", since it's checked only at run time. But you can create new var
    """
    if True: 
        x=123
    print("test is: ", x)

    def create_global():
        # create global var
        global global_var 
        global_var = 78

    def read_global(): 
        global global_var
        print(global_var)
        # then you can modify the global var as well

    create_global()
    read_global()

def test_nested_func(): 
    class Foo(object):
        def __init__(self):
           self.haha = "mark" 
        def nested_func(self): 
            def another_func(): 
                self.haha = "markhaha"
            another_func()
            print(self.haha)

    f = Foo()
    f.nested_func()

def test_another_nested_func(): 
    name = 10
    def worker():
        # nonlocal is needed here
        nonlocal name
        name += 1
        print(name)
    worker()

def np_masking(): 
    arr = np.random.rand(4,3,3)
    print(arr)
    mask = np.logical_and((arr > arr.min()), (arr < arr.max()))
    print(mask)

def property_test():
    class foo: 
        pass
    f = foo()
    f.temperature = 800
    print(f.temperature)

    print(f.__class__.__name__)

if __name__ == "__main__": 
    # test_nested_func()
    test_scope()
