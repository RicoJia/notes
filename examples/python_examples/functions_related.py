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
    """
    How it works: (apart from kwargs support). partial is returning a wrapper with extended args
    def partial(func, *part_args):
        def wrapper(*extra_args):
            args = list(part_args)
            args.extend(extra_args)
            return func(*args)
        return wrapper
    func_wrapper = partial(func, 1)
    func_wrapper(12)        # pass 12 in as extra_args
    """
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

def test_decorator(): 
    """
    1. Decorator is not to execute a function with extra args. Instead it will return a "decorated function" 
        - Decorator is just a syntactic sugar for Foo = decorate(Foo)
        - the way to use it is to add timethis on top of some_func.  
        - with functions.wraps, we can have: 
            1. __name__ being "some_func" instead of "wrapper"
            2. Access the wrapped function
            3. Of course wraps is optional. Decorator can work without it
    2. This is called "metaprogramming", which is to write a program that modified an existing program
        - decorators heavily use closures and returns a callable
        - **in python, an object with __call__() is a callable**
    """
    import time 
    from functools import wraps
    # 1
    def timethis(func): 
         @wraps(func)
         # have to use args and kwargs to wrap a function with.
         def wrapper(*args, **kwargs):
             start = time.time() 
             result = func(*args, **kwargs) 
             end = time.time() 
             print(func.__name__, end-start) 
             # return the same thing as a convention
             return result 
         return wrapper
    # decorator is just equivalent to a wrapper
    def foo(): 
        print("foo")
    foo = timethis(foo)
    foo()

    @timethis
    def some_func(): 
        print('hehe')

    some_func()
    print("function attributes: ", some_func.__name__)
    some_func.__wrapped__()

def test_class_decorator(): 
    """
    1. Motivation: make all class functions decorated the same way. 
        - Caution: __getattribute__() might be recursive
    """
    # 1
    def time_all_class_methods(Cls):
        # Cls: class
        # decoration body - doing nothing really since we need to wait until the decorated object is instantiated
        class Wrapper:
            def __init__(self, *args, **kwargs):
                print(f"wrapper init")
                self.decorated_obj = Cls(*args, **kwargs)

            def __getattribute__(self, s):
                """
                Must return a callable. s is a string 
                """
                print("1")
                x = super().__getattribute__("decorated_obj").__getattribute__(s)
                return x
        return Wrapper  # decoration ends here

    @time_all_class_methods
    class MyClass:
        def __init__(self):
            print("MyClass.__init__")
        def method_x(self):
            print("Calling from MyClass.method_x")
        def method_y(self):
            print("Calling from MyClass.method_y")

    """
    see
    wrapper init
    MyClass.__init__
    then in function calls, can see 1 being printed out
    """
    mc = MyClass()
    mc.method_x()
    mc.method_y()

    # 2
    REGISTERED_SPECS = dict()
    def _register_spec(name: str, cls: Type):
        alias = name or cls.__name__
        REGISTERED_SPECS[alias] = cls
        return cls
    from functools import partial
    def is_spec(name: str = None):
        return partial(_register_spec, name)


if __name__ == "__main__": 
    # test_nested_func_in_class()
    # test_scope()
    # test_optional_arg()
    # test_type_hints()
    test_decorator()
    # test_class_decorator()
