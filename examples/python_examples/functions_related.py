#!/usr/bin/python3
import numpy as np

###############################################################################
### Basics
###############################################################################
global_var = "global"
def test_lambda(): 
    """
    1. lambda can be called like this
    2. Lambda can capture values, but they're again references bound at runtime.
    3. If you want to store value of lambda, store it.
    """
    # 1 
    print((lambda x: x > 10)(11))

    # 2 
    x = 10
    lam = lambda y: y+x
    x = 300
    print("Lambda can capture values, but they're again references bound at runtime: ", lam(20), " see 320 instead of 30")

    # 3
    x = 10
    lam  = lambda y, x = x: y+x
    x = 300
    print(f"If you want to store value of lambda, store it. See {lam(20)} instead of 320")

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

def property_test():
    class foo: 
        pass
    f = foo()
    f.temperature = 800
    print(f.temperature)
    print(f.__class__.__name__)

def get_current_funcs_info():
    """
    1. func name: {sys._getframe(0).f_code.co_name}, file name: {__file__}, line number: {sys._getframe(0).f_lineno}
    2. all params: {sys._getframe(0).f_locals}
    """
    def dummy_method():
        pass
    import sys
    print(f"func name: {sys._getframe(0).f_code.co_name}, file name: {__file__}, line number: {sys._getframe(0).f_lineno}")
    print(f"all params: {sys._getframe(0).f_locals}")

###############################################################################
### Args
###############################################################################
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

def test_default_arg():
    """
    1. use immutables only as default args. mutables, such as list, is shared with future function calls, hence they can be changed
        - This is really tricky!
    """
    def test_l(l = []): 
        l.append(123)
        return l
    l1 = test_l()
    l2 = test_l()
    # see [123, 123], [123, 123]
    print("two different function calls share the same mutable: ", l1, l2)

def kwargs_test(): 
    """
    1. kwargs(keyworded arguments) is just a dictionary
    2. args(positional args) and kwargs for the same function
        - remember kwargs don't have an order, so have them at the very last
        - also whatever comes after *args will be kwargs
    3. You can enforce functions to have only keyworded,
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

    # 2
    def test_args_kwargs(*args, b, **kwargs): 
        print("args: ", args)
        print("b: ", b)
        print("kwargs: ", kwargs)
    test_args_kwargs(1,2,3,4, b="asdf", LOL=12)

    # 3
    def recv(**kwargs): 
        print("kwargs is enforced: ", kwargs)
    def recv2(*, block): 
        print("1 keyworded arg is enforced: ", block)
    recv(LOL="23")
    recv2(block="asdf")

def test_partial():
    """
    1. How it works: (apart from kwargs support). partial is returning a wrapper with extended args
    2. equivalent implementation
    """
    # 1 partial
    from functools import partial
    def func(a, b): 
        print("func: ", a, b)
    func_w = partial(func, b = 12)
    func_w(a = 13)

    def rico_partial(func, *args, **kwargs):
        # simplified version
        # def wrapper(a): 
        #     # kwargs here is a dict, need to unpack it
        #     return func(a, **kwargs)
        # return wrapper
        def wrapper(*extra_args, **extra_kwargs):
            # need nonlocal since we are reusing args, and kwargs, which will be true local vars
            nonlocal args, kwargs
            # args here is a tuple already
            args = list(args)
            args.extend(extra_args)
            kwargs = {**kwargs, **extra_kwargs}
            return func(*args, **kwargs)
        return wrapper
    rico_func_w = rico_partial(func, b = 12)
    rico_func_w(a=13)

###############################################################################
### Misc
###############################################################################

def test_control_flow(): 
    """
    1. decorator that wraps a generator function, which launches an output queue 
        - test() is the generator class here
        - send(None) to start generator
        - THE POINT OF THIS DECORATOR is to step thru all yield functions in just oneline
    2. Return Async, which takes in lambda as a computation func and a callback
    """
    def apply_async(func, args, *, callback): 
        # do computation, then put result on to the queue
        result = func(*args)
        callback(result)

    from queue import Queue 
    from functools import wraps
    def inlined_async(func): 
        # TODO?
        @wraps(func) 
        def wrapper(*args): 
            f = func(*args) 

            result_queue = Queue()
            # start the generator f.
            result_queue.put(None)
            while True:
                result = result_queue.get()
                try:
                    # first you send None, get Async, apply_async, next iteration you send 5, get stop_iteration
                    a = f.send(result)
                    # do computation, then put result on to the queue
                    apply_async(a.func, a.args, callback=result_queue.put)
                except StopIteration: 
                    break
        return wrapper

    add = lambda x,y: x+y
    class Async: 
        def __init__(self, func, args): 
            self.func = func 
            self.args = args
            print("async constructed")

    @inlined_async
    def test():
        r = yield Async(add, (2,3))
        print("test r: ", r)

    test()

def test_type_hints(): 
    """
    1. Python's type hints are NOT enforced, but can be used by IDE. 
        - stored in func.__annotation__. just call func.__annotation__
    2. Use string as default value for funcs whose args are defined later
    3. for < python 3.9, you need typing to indicate what exactly goes into the container
    4. typing.sequence can be used to refer to list, tuple
    5. Type alias: define custom aliases 
    6. Optional: there may or may not be a returned value. If not, None is returned
    7. Union: an arg can be one of any types here. Optional is Union(type, None)
    8. Callable: Can be a function, or a class with __call__. See below for how to use it
    9. Any: wildcard
    10.Template T in C++
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

    # typing for list 
    from typing import List
    def func3_5(scores: List[int]): 
        print(scores)
    func3_5([1,2,3])

    # typing that stores __name__ for a class 
    from typing import Type
    def func3(p: Type): 
        print("Type.type can store __name__",  p.__name__)
    func3(Person)

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

###############################################################################
### Closure, Nested Functions, decorator
###############################################################################

def test_nested_func_in_class(): 
    """
    1. nested function can modify the same member in class
    """
    # 1
    class Foo(object):
        def __init__(self):
           self.haha = "mark" 
        def nested_func(self): 
            def another_func(): 
                self.haha = "markhaha"
            another_func()
            print("modify member in class: ", self.haha)
    f = Foo()
    f.nested_func()

def test_closure_func(): 
    """
    1. closure function 
        - can hold extra internal states, which is really useful
        - again, global variables are bound in runtime
    2. To modify the internal states, expose them as methods of function project
        - Yes you can add attributes to a function object!
    """
    # 1
    x = 20
    def wrap():
        some_num = 9
        def func(*args):
            s = sum(args) + x + some_num
            return s
        return func

    x = 10
    w = wrap()
    x = 100
    print(w(3,4,5))

    # 2
    def another_wrap():
        some_num = 9
        def func(*args):
            s = sum(args) + some_num
            return s
        
        def set_some_num(val):
            # must use nonlocal as we're modifying upper level variable
            nonlocal some_num
            some_num = val
        def get_some_num(): 
            return some_num

        func.set_some_num = set_some_num
        func.get_some_num = get_some_num
        print("1")
        return func
    
    aw = another_wrap()
    aw.set_some_num(100)
    print(f"some_num now is 100: {aw.get_some_num()}, so sum is {aw(0)}")

def test_decorator(): 
    """
    1. Decorate(func)(*args, **kwargs) is the canonical setup for decorator, actually calling a nested function 
        - Decorator is just a syntactic sugar for Foo = decorate(Foo), which exeuctes the 1st layer of the wrapper, and return the second layer  
            @decorate
            def Foo(...):
                pass
        - If you're curious, you can the following, which executes 1st, 2nd layers of the wrapper, and try to store the 3rd
            @decorate()
            def Foo(...)
            
    3. With functions.wraps, we can have: 
        1. __name__ being "some_func" instead of "wrapper"
        2. Access the wrapped function
    4. Of course wraps is optional. Decorator can work without it
    5. Decorator is not to execute a function with extra args. Instead it's a fucntion returning a wrapped function
        - This is called "metaprogramming", which is to write a program that modified an existing program
        - decorators heavily use closures and returns a callable
        - **in python, an object with __call__() is a callable**
    """
    # 1 basic example -  decorator is a fucntion returning a wrapped function
    # this step actually executes the function_decorator, wraps the function and return the wrapped func to the function object
    # equivalent to calling function_decorator(decorator_kwargs)(wrapper_kwargs)
    def function_decorator():
        print("start function_decorator")
        def dummy_wrap(func):
            print("start dummy_wrap")
            def wrapped_func():
                print('=' * 30)
                func()
            print("end function_decorator")
            return wrapped_func
        return dummy_wrap
    
    @function_decorator()
    def test():
        print('1, Hello World!')
    # execute wrapped_func
    test()

    # 3
    import time 
    from functools import wraps
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

def test_closure_as_class():
    """
    1. You can make a fake class by using a function: 
        1. In the current function, you can get the functions using sys._getframe(1).f_locals
        2. You can add attributes to the class by adding to self.__dict__
        3. callable(value) to see if the value is a callable
    2. This method is a bit faster than the conventional method because we are using a function, which doesn't have self. 
    """
    import sys
    class ClosureReturn():
        def __init__(self):
           locals = sys._getframe(1).f_locals 
           print("here are the local attributes: ", locals)
           self.__dict__.update((key, value) for key, value in locals.items() if callable(value))

    def stack_in_closure(): 
        items = []
        def push(val): 
            items.append(val)
        def pop():
            return items.pop()
        def lube():
            print("LUBE")

        return ClosureReturn()
    
    fake_stack = stack_in_closure()
    fake_stack.push(12)
    print("fake stack is popping: ", fake_stack.pop())
    fake_stack.lube()

def test_chaining_decorators(): 
    """
    1. execute top to bottom . func1(a)(b)...
    """
    def func1(func):
      def inner(arg):
        print(func1.__name__)   #this is how you print the name
        func(arg)
      return inner

    def func2 (func):
      def inner(arg):
        print(func2.__name__)
        func(arg)
      return inner

    @func1
    @func2
    def func(msg):      #will first execute func1, then func2.
      print(msg)

    func("holi")

def test_class_decorator(): 
    """
    1. Motivation: make all class functions decorated the same way. 
        - Caution: __getattribute__() might be recursive
    2. decorator(args1)(func/class)
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
    # mc = MyClass()
    # mc.method_x()
    # mc.method_y()

    # 2
    from typing import Tuple, Type
    REGISTERED_SPECS = dict()
    # decorator(args1)(func/class)
    def is_spec():
        def _register_spec(cls: Type):
            name = None
            # ClassName.__name__ can work
            alias = name or cls.__name__
            REGISTERED_SPECS[alias] = cls
            print("Haha from inside wrapper")
            return cls
        return _register_spec
    # note that we need (), then it's equivalent to have is_spec()(class Foo)
    @is_spec()
    class Foo:
        def __init__(self):
            print("Foo.__init__")
        def method_x(self):
            print("Calling from Foo.method_x")
        def method_y(self):
            print("Calling from Foo.method_y")
    mc = Foo()
    print(REGISTERED_SPECS)

def test_function_frame(): 
    """
    Callstack is LIFO, so getting into a function pushes a frame on to the stack
        - so the current function is stack[0]
    """
    a = 2
    import sys
    print("getframe(0): ", sys._getframe(0).f_locals)
    print("getframe(1): ", sys._getframe(1).f_locals)
    # print("getframe(2): ", sys._getframe(2).f_locals)

if __name__ == "__main__": 
    # get_current_funcs_info()
    # test_closure_as_class()
    # test_nested_func_in_class()
    # test_scope()
    # test_optional_arg()
    # test_type_hints()
    test_decorator()
    # kwargs_test()
    # test_class_decorator()
    # test_default_arg()
    # test_lambda()
    # test_partial()
    # test_control_flow()
    # test_closure_func()
    # test_function_frame()
