import numpy as np

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
    test_nested_func()
