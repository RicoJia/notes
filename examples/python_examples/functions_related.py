#!/usr/bin/python3
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

def test_optional_arg(): 
    from typing import Optional, Union
    # telling the type checker that an object of the specific type is required
    # this is called type hints 
    # Python doesn't enforce type hints, but mypy, pycharm will. 
    def func(arg: Union[int]): 
        print(type(arg))
    # Optional is Union(..., None)
    def func2(arg: Optional[int]): 
        print(type(arg))
    func(None)
    func2(2)

def kwargs_test(): 
    """
    kwargs is just a dictionary
    """
    def test_args(*args): 
        print(args) # should see a tuple

    class test_kwargs: 
        def __init__(self, **kwargs): 
            print(kwargs)   # should see a dictionary
            print(kwargs["foo"])
            for key, value in kwargs.items():
                print(value)

    test_kwargs(foo="bar", love=101)
    test_args(101, "123")

def test_partial():
    # partial
    from functools import partial
    def func(a, b): 
        print(a, b)
    func_w = partial(func, b = 12)
    func_w(a = 13)

if __name__ == "__main__": 
    # test_nested_func()
    # test_scope()
    test_optional_arg()
