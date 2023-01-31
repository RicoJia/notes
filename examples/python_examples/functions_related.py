#!/usr/bin/python3
import numpy as np

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

def test_signature():
    '''
    1. sig.bind_partial(values / types to bind to each arg in func). 
        - return OrderedDict
        - Allows partially binding a func. sig.bind does not support this
    2. sig.parameters: stores function's original signature. 
    '''
    from inspect import signature
    def func(a, b: float):
        return a + b
    sig = signature(func) 
    print(sig.parameters)
    # see a <class 'inspect._empty'> POSITIONAL_OR_KEYWORD
    # does not change after defining bound_types
    print(sig.parameters["a"].name, sig.parameters["a"].default, sig.parameters["a"].kind)

    # OrderedDict([('a', <class 'int'>), ('b', 12)])
    bound_types = sig.bind_partial(int,b=12).arguments
    print(bound_types)
    # see 'int'
    print(bound_types["a"])

def test_3_level_function_signature():
    def foo(var1, var2:str, var3:int = 99):
        import inspect
        sig = inspect.signature(foo)
        configs = {}
        default_params = {}
        actual_params = {}
        for param in sig.parameters.values():
            # we are safe to use eval because we don't have any out-of-scope variables that override param.name
            if param.default is param.empty:
                default_params[param.name] = None
            else:
                default_params[param.name] = param.default
            if eval(param.name) != default_params[param.name]:
                actual_params[param.name] = eval(param.name)
        # Now we have a dictionary of configs: overriding order is:
        # user-specified args > config_yaml > default_args
        configs = {}
        configs.update(default_params)
        configs.update(actual_params)
        print("configs: ",configs, "default params: ", default_params, "actual params: ", actual_params)
    foo(1,"haha")
###############################################################################
### Type Hints
###############################################################################
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
    # from typing import type
    def func3(p: type): 
        print("type.type can store __name__",  p.__class__.__name__)
    func3(Person())

    # typing.sequence can be used to refer to list, tuple
    from typing import Sequence as seq1
    def func4(seq:seq1): 
        for item in seq: 
            print(item)
    func4([2,3,4])

    # type alias: define custom aliases. could be useful in this case: 
    # from production.contracts import default_pose, tree
    # cameranodeconfigspec = tree.cameranodeconfigspec
    from typing import Tuple
    vector2d = Tuple[int, int]
    def func5(vector: vector2d):
        print(vector)
    func5((1,2))

    # optional: there may or may not be a returned value. if not, none is returned
    from typing import Optional
    def func6(i : int) -> Optional[int]:  
        if i < 4: 
            return 100
    func6(4)

    # callable: can be a function, or a class with __call__. see below for how to use it
    from typing import Callable
    class bar: 
        def __call__(self, i: int): 
            print(i)
    def func8(f: Callable[[], None]): 
        # pass the arg here, not when you construct it.
        f(9)
    func8(bar())

    # any: wildcard
    from typing import Any
    def foo() -> Any: 
        return "fubar"
    print(foo()) 

    # template t in c++, t 必须是 str 或 int 其中一种
    from typing import TypeVar, List
    # t must be int or str
    t = TypeVar('t', int, str)
    def func9(a: t, b: t): 
        print(a, b)
    # this won't complain since we have mixed types
    func9(1, (3,4))

    import typing
    def func10(a:typing.Union[str, bytes, int]):
        print(a)

    func10(1)
    func10("lol")

def test_typing_callable():
    def func(bool)->int:
        return 1
    from typing import Callable
    def test_func(func: Callable[[bool], int]):
        pass
    test_func(func)

def test_type_hints_quirks():
    """
    1. If you have a function taking in itself as an argument, you need '' for type hint.
    """
    class Foo:
        def __eq__(self, other: 'Foo'):
            return True

    f =Foo()
    g = Foo()
    print(f==g)

def test_factory_func_type_hints():
    """See above for the quirk. 
    """
    class Foo:
        @classmethod
        def foo_fact(cls) -> "Foo":
            # Class funcs are usually in factory funcs btw
            return Foo()
    f = Foo.foo_fact()

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

def test_bound_class_method():
    '''
    1. Everybody knows that self (instance) needs to be bound to a class method. How?
    2. Function itself is also a descriptor, with __get__(self, instance, owner_cls)
        - This is the secret sauce that binds the function to a class
    3. 
    '''
    # 1
    class Foo(object):
        def f(self):
            pass
    # see unbound method
    print(Foo.__dict__["f"])
    # see unbound method
    print(Foo.f)
    # see bound method
    print(Foo().f)

    import types
    def bar(self):
        pass
    bound_bar = types.MethodType(bar, Foo())
    print("bound_bar: ", bound_bar)

if __name__ == "__main__": 
    # get_current_funcs_info()
    # test_closure_as_class()
    # test_nested_func_in_class()
    # test_scope()
    # test_optional_arg()
    # test_type_hints()
    # test_type_hints_quirks()
    # test_signature()
    # test_3_level_function_signature()
    # kwargs_test()
    # test_class_decorator()
    # test_default_arg()
    # test_lambda()
    # test_partial()
    # test_control_flow()
    # test_closure_func()
    # test_function_frame()

    # test_bound_class_method()
    # test_factory_func_type_hints()
     test_typing_callable()
