#!/usr/bin/python3
import numpy as np

global_var = "global"
def test_scope():
    """
    1. The outer function variables are by-default read only. If you declare a variable locally without nonlocal or global, then that will be a local variable, nothing to do with the outer one
        - When we use =, that is to assign the NAME of an object a new object. And that is not allowed without nonlocal/global 
        - If else... doesn't count as a "new scope", since it's checked only at run time. 
        - A function can get the variable of the closest upper scope. This is called "lexical scoping", > 40 yrs. 
    2. We can modify a outside mutable WITHOUT nonlocal 
        - e.g., ls.append() that is to modify the object itself
    """
    some_var = "test_scope"
    ls_cp = []
    def worker():
        # nonlocal is needed here
        global global_var
        global_var = "inner scope"
        nonlocal some_var 
        some_var = "another inner scope"
        ls_cp = [123]
    worker()
    print(global_var)
    print(some_var)
    print(ls_cp)

def test_nested_func_in_class(): 
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

def test_type_hints(): 
    """
    1. Python's type hints are NOT enforced, but can be used by IDE. 
    2. Use string as default value for funcs whose args are defined later
    3. for < python 3.9, you need typing to indicate what exactly goes into the container
    4. typing.sequence can be used to refer to list, tuple
    5. Type alias: define custom aliases 
    6. Optional: there may or may not be a returned value. If not, None is returned
    7. Union: an arg can be one of any types here. Optional is Union(type, None)
    8. Callable: Can be a function, or a class with __call__. See below for how to use it
    9. Any: wildcard
    10. Template T in C++
    """
    # basic - type hint with default val
    def func1(age: int = 20): 
        print (age)
    func1()

    # string for funcs whose args are defined later
    def func2(p: 'Person') -> str: 
        return "hello "+p.name
    class Person: 
        name = "hehe"
    p = Person()
    print(func2(p))

    # for < python 3.9, you need typing to indicate what exactly goes into the container
    from typing import Tuple, Dict
    def func3(scores: Tuple[int, int], di: Dict[int, str]): 
        print(scores, di)
    func3((3,4), {1:"asdf"})

    # typing.sequence can be used to refer to list, tuple
    from typing import Sequence as Seq1
    def func4(seq:Seq1): 
        for item in seq: 
            print(item)
    func4([2,3,4])

    # Type alias: define custom aliases. Could be useful in this case: 
    # from production.contracts import DEFAULT_POSE, tree
    # CameraNodeConfigSpec = tree.CameraNodeConfigSpec
    from typing import Tuple
    Vector2D = Tuple[int, int]
    def func5(vector: Vector2D):
        print(vector)
    func5((1,2))

    # Optional: there may or may not be a returned value. If not, None is returned
    from typing import Optional
    def func6(i : int) -> Optional[int]:  
        if i < 4: 
            return 100
    func6(4)

    from typing import Union
    def func7(i: Union[int, str]): 
        print(i)
    func7("str")

    # Callable: Can be a function, or a class with __call__. See below for how to use it
    from typing import Callable
    class Bar: 
        def __call__(self, i: int): 
            print(i)
    def func8(f: Callable): 
        # pass the arg here, not when you construct it.
        f(9)
    func8(Bar())

    # Any: wildcard
    from typing import Any
    def foo() -> Any: 
        return "FUBAR"
    print(foo()) 

    # Template T in C++, T 必须是 str 或 int 其中一种
    from typing import TypeVar, List
    T = TypeVar('T', int, str)
    def func9(a: T, b: T): 
        print(a, b)
    # this will complain since we have mixed types
    func9(1, "sr")

if __name__ == "__main__": 
    # test_nested_func_in_class()
    # test_scope()
    # test_optional_arg()
    test_type_hints()
