##############################################################
### Property, Descriptor
##############################################################

def test_property():
    """
    1. use property to 
        1. do additional processing
        2. Compute things on demand
    2. property is a bundle of setter, getter, and deleter functions
        - use @property to make a funciton a propertie's getter
        - ALL functions have the same name!!
        - Java programmers will make everything a property
    3. getter must be defined first, setter, deleter must follow
        - if getter is not defined, then you'll see errors
    4. Alternatively, you can use property(getter_func, setter_func, deleter_func) to make something a property
        - You can call the setter, getter functions directly, especially for remote calls, which doesn't work on the properties
    5. you can see setter, getter, deleter funcs through ClassName.attr.fget
    """
    class Foo(object): 
        # 3
        @property
        def name(self):
            return self.name_str 

        @name.setter
        def name(self, name:str):
            print("setting names")
            if not isinstance(name, str):
                # there's TypeError
                raise TypeError("name is not string")
            self.name_str = name

        @name.deleter
        def name(self):
            # there's AttributeError
            raise AttributeError("cannot delete")

    f = Foo()
    f.name="Luis"
    print(f.name) 
    try: 
        del f.name
    except AttributeError:
        pass

    # 4 
    class Bar:
        def get_name(self):
            return self.name_str
        def set_name(self, name):
            print("setting names")
            if not isinstance(name, str):
                # there's TypeError
                raise TypeError("name is not string")
            self.name_str = name
        def del_name(self):
            raise AttributeError("cannot delete")

        name = property(get_name, set_name, del_name)
    b = Bar()
    b.set_name("Guaya")
    print("get name: ", b.get_name())

    # 5
    print("fget: ", Foo.name.fget)

# -------------------------------- Decorator --------------------------------
from functools import wraps
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
        """
        1. need * args, **kwargs to make sure they are tuple and dict
        func(*args, **kwargs) actually does the unpacking.  
        2. @wraps is VERY important. Otherwise, foo.__name__, foo.__doc__, foo.__annotations__
        will all be wraps. 
        """
        def timer(func):
            """
            Timer doc
            """
            @wraps(func)
            def wrap(*args, **kwargs):
                import time 
                start = time.time()
                # Still not sure why we need *? 
                res = func(*args, **kwargs)
                end = time.time()
                print(func.__name__, end-start)
                return res
            return wrap
        
        @timer
        def foo():
            """
            Foo doc
            """
            print("Foo")            
                
        print(foo.__name__, foo.__doc__, foo.__annotations__)
        print(foo.__name__, foo.__doc__, foo.__annotations__)


def test_internal_wrap():
    '''
    1. You need to use wraps on the decorator, or directly set __wrapped__ attribute, for the original function
    '''
    from functools import wraps
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            #TODO
            print(f'lol')
            return func(*args, **kwargs)
        return wrapper
    @decorator
    def foo(num):
        print(num)
    foo(4)

    original = foo.__wrapped__
    print(original) 

def test_class_decorator():
    """
    1. class decorator is a function that returns a modified class, to make all class functions decorated the same way. 
        - Caution: __getattribute__() might be recursive
    2. decorator(args1)(func/class)
    """
    def log_getattribute(cls):
        orig_getattribute = cls.__getattribute__
        def getattribute(self, name):
            print('Calling {name} from {cls_name}'.format(name=name, cls_name=cls.__name__))
            return orig_getattribute(self, name)
        cls.__getattribute__ = getattribute
        return cls
    @log_getattribute
    class Foo(object):
        def __init__(self, a):
            self.a = a
            
    # equivalent to Foo = log_getattribute(Foo)
    f = Foo(3)
    f.a

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

def test_decorator_deep_dive():
    '''
    Misc:
    1. setattr will call __setattr__
    2. You can add an attribute to a function object, but not a list
    Decorator Binding:
    1. binding to decorator happens during function definition time, not when a function is called
    2. How decorator works (1 layer witha args + 1 layer with func + Additional wrapper layer)
        @decorator(args...)
        func(...)
        1. Outmost layer: if args passed into the outmost layer have all been supplied with params. Also, it returns a function (layer 1) with just 1 arg
            - if the outmost layer just gets 1 arg, then it has to be func
            - and you call it like
                @decorator
                func(...)
        2. Layer 1: takes in func as arg, which must return a function with the same number of args as func.
        3. additional wrapper layer: it's good as long as it takes in same number of args as func
    3. The "self-return" trick is also just following how the decorator works. 
    '''
    from functools import wraps, partial 
    import logging
    def attach_wrapper(obj, func=None): 
        print(obj, "func", func)
        if func is None: 
            print("binding obj to attach_wrapper")
            # the "self-return" trick
            return partial(attach_wrapper, obj) 
        print("rico2")
        setattr(obj, func.__name__, func) 
        return func
        # # equivalent to
        # def wrap2(func):
        #     print("rico2")
        #     return func
        # return wrap2

    def logged(level=logging.INFO, name=None, message="heh"): 
        def decorate(func): 
            logname = name if name else func.__module__ 
            log = logging.getLogger(logname) 
            logmsg = message if message else func.__name__

            @wraps(func) 
            def wrapper(*args, **kwargs): 
                log.log(level, logmsg) 
                return func(*args, **kwargs)
            
            @attach_wrapper(wrapper)
            def set_message(newmsg): 
                # be careful with nonlocal
                nonlocal logmsg 
                logmsg = newmsg

            # set_message = attach_wrapper(wrapper)

            return wrapper

        return decorate

    logging.basicConfig(level=logging.DEBUG)
    def func(name):
        print(name)

    func = logged()(func)
    func("rico")

    func.set_message("newmsg")
    func("rico")
# -------------------------------- Descriptor --------------------------------

def test_descriptor():
    """
    1. Foundation for property, classmethods, staticmethods, and larger libs
        - Only works on per-class basis
    2. Key idea is once descriptor object is assigned to class variable, If you assign the class var to another val, 
    the val will be pluged into __set__(instance, value)
        - and subsequent obj.attr will trigger __get__(instance, cls)
    2. Can be used to 
         - avoid duplicating properties
         - Enforce types
    """
    # 1, 2
    class Int:
        def __init__(self, name):
            self.name = name
        def __get__(self, instance, cls):
            # instance is the upper level object. If Int is a class variable, cls is the class, instance is None, 
            # In that case, it's common practice to return the object itself.
            if instance is None:
                return self
            else:
                return instance.__dict__[self.name]
        def __set__(self, instance, value):
            print("setting value: ", value)
            if not isinstance(value, int):
                raise TypeError("value should be int")
            instance.__dict__[self.name] = value
        def __delete__(self, instance):
            del instance.__dict__[self.name]

    class Foo:
        i = Int("my_var")
        def __init__(self):
            # Note: only works on per-class basis, so without the class variable, this wouldn't work
            # self.i = Int("myvar")
            pass
    # 1
    f = Foo()
    # calls Foo.i.__set__(f, 22)
    f.i = 33
    try:
        f.i = 23.3
    except TypeError:
        pass
    print("equivalent to: ")
    print("Int.__get__() will return the int: ", f.i)
    Foo.i.__set__(f, 22)
    print(f.i)    

def test_lazy_attr():
    """
    1. Descriptors can be used as LAZY ATTRIBUTES.
        - ONE THING TO NOTE: you must NOT have __set__(), __delete__(). These will make the binding stronger
    """
    class LazyAttr(object):
        def __init__(self, func):
            self.__func = func
        def __get__(self, instance, cls):
            if not instance:
                # when we have class variable
                return self
            else:
                # store value to the instance under the same name as the func.
                # Without storing, the object will have to call this function again.
                value = self.__func(instance)
                setattr(instance, self.__func.__name__,value)
                #TODO
                print(f"1: ", value, "func name:", self.__func.__name__)
                return value
        # def __set__(self, instance, value):
        #     pass
        # def __delete__(self, instance):
        #     pass
        
    import numpy as np
    class Circle:
        def __init__(self, radius) -> None:
            self.radius = radius
        @LazyAttr
        def circumference(self):
            print("calculating Circumference")
            return 2 * np.pi * self.radius
    
    c = Circle(2.3)
    print("Circumference:", c.circumference, "vars: ", vars(c))
    print("Circumference:", c.circumference)
    
def test_type_check_descriptor():

    # Used to check if an object's attributes are of expected types
    class Typed:
        def __init__(self, name, expected_type):
            self.name = name
            self.expected_type = expected_type
        def __get__(self, instance, cls):
            if instance is None:
                return self 
            return instance.__dict__[self.name]
        def __set__(self, instance, val):
            if not isinstance(val, self.expected_type):
                raise TypeError(f"{val} is not the expected type")
            instance.__dict__[self.name] = val
        def __delete__(self, instance):
            del instance.__dict__[self.name]

    # a decorator
    def typeassert(**kwargs):
        def wrapper(cls):
            for name, expected_type in kwargs.items():
                cls.name = Typed(name, expected_type)
                # setattr(cls, name, 2)
                # t.__set__(None, cls.name)
            print(cls)
            return cls
        return wrapper

    #decorator(kwargs)(wrapper_kwargs)
    @typeassert(name=str, grade=int)
    class Foo:
        def __init__(self, name, grade):
            print(">>>>>>")
            self.name = name
            self.grade = grade

    # typeassert(...)(Foo(...)) -> wrapper(Foo(...))
    try:
        Foo("rico", "not_a_valid_grade")
    except:
        pass
    print("Foo: ", Foo.__dict__)
    
def test_property_change():
    """
    1. Can Extend one of the proeprty functions
        - usual signature is super(subclass, instance) 
        - but here we'd change __set__ in super(subclass, subclass)
    """
    class Foo(object):
        def __init__(self):
            self._name="RJ-Foo"
        @property
        def name(self):
            return self._name
        @name.setter
        def name(self, some_name):
            self._name=some_name

    class Foo_Extended(Foo):
        @Foo.name.setter
        def name(self, some_name):
            print("Extension setting name right now: ", some_name)
            super(Foo_Extended, Foo_Extended).name.__set__(self, some_name)
        
    f = Foo()
    print("f name: ", f.name)
    f.name="RJ-HA"
    print("f name: ", f.name)
    
    fe = Foo_Extended()
    fe.name="RJ"
    print("fe name: ", fe.name)
            
 ################################################################
 ### Design Patterns
 ################################################################
def test_base_field():
    """
    1. Base field has __init__, which avoids redundancy
        - field is just a placeholder
        - Subclasses can have their own fields.
    2. setattr has these advantages:
        1. programmatic
        2. can work with properties wrapped in properties, or slotclass, 
            - instead of like __dict__.update(another_dict)
    """
    class BaseField(object):
        _fields = None
        def __init__(self,*args, **fields):
            if (len(fields.keys()) + len(args)>len(self._fields)):
                raise TypeError("Fields has wrong length")
            for name,values in zip(self._fields, args):
                setattr(self,name,values)
            for name, val in fields.items():
                setattr(self, name, val)
            print("dict supports set arithmetics: ", type(fields.keys() - self._fields))
    class Foo(BaseField):
        _fields = ["name", "age", "grade"]
    f1 = Foo(1,2,3)
    print(vars(f1))
    f2 = Foo(shabi=100, grade=100, age= 99) 

def test_delegation():
    """
    1. Delegation is to have another object to call the same attribute
    2. use __getattr__ if there're too many attr 
    3. __getattr__ supports non __ attributes only
    """
    class A:
        def spam(self):
            print("A spam")
        def spam1(self):
            print("A spam1")
    class B:
        def __init__(self):
            self._a = A()
        def spam(self):
            #TODO 
            print(f"b spam")
            self._a.spam()
        def __getattr__(self, attr_name):
            print(f"{attr_name} not exist in B")
            getattr(self._a, attr_name)

    b = B()
    b.spam()
    b.spam1
    # b.spam1()  is not valid, because it's a method, not an attribute

    class ListLike:
        l = [1,2,3,4]
        def __getattr__(self, attr_name):
            return getattr(self.l, attr_name)

    ll = ListLike()
    ll.append(5)
    ll.sort()
    print(f"append, sort both work: ")
    try: 
        len(ll)
    except:
        print("len doesn't work")
        
def test_stateful_class():
    """
    1. Method 1:
        - The key is to assign a state to a class, then call the class's static function for switching
        - And you should have all the same functions for each state class, 
        - So you can just call each class's function correspondingly, and let each function handles the rest
    2. Method 2:
        - They key is to switch the entire class to another class
        - This however, can only be applied to simple cases!
    """
    # 1
    class Connection: 
        def __init__(self): 
            # just calling the function below
            self.new_state(ClosedState)
        def new_state(self, newstate): 
            self._state = newstate
        # Delegate to the state class 
        def read(self): 
            return self._state.read(self)
        def open(self): 
                return self._state.open(self)
    # Implementation of different states 
    class ClosedState: 
        @staticmethod 
        def read(conn): 
            """
            So you can't read while during a closed state. Therefore 
            """
            raise RuntimeError('Not open')
        @staticmethod
        def open(conn):
            conn.new_state(Open)
    class Open: 
        @staticmethod 
        def read(conn): 
            print('reading')
        @staticmethod 
        def open(conn): 
            raise RuntimeError('Already open')
    # implementation
    c = Connection()
    print(c._state)
    
    #2 
    class State:
        def __init__(self) -> None:
            self.new_state(StateA)
        def new_state(self, StateX):
            """
            Switching the entire class the StateX
            """
            self.__class__ = StateX
            
    class StateA(State):
        def action(self):
            print("State A invoked")    
            self.new_state(StateB)
    
    class StateB(State):
        def action(self):
            print("State B")
            self.new_state(StateA)
    
    s = State()
    s.action()
    s.action()
    
def test_visitor_pattern():
    """
    This might be too convulted. But the idea is:
        1. Separate data structure from operations, 
        2. When you apply operations, you can use getattr(instance, methname, default_value)
    """
    # 1. create a "tree": a node with left & right being Number
    class Node: 
        pass
    class UnaryOperator(Node): 
        def __init__(self, operand): self.operand = operand
    class BinaryOperator(Node): 
        def __init__(self, left, right): 
            self.left = left 
            self.right = right
    
    class Sub(BinaryOperator): 
        pass
    class Number(Node): 
        def __init__(self, value): 
            self.value = value
    t1 = Sub(Number(3), Number(4))    
    
    # Node Visitor: 
    class NodeVisitor: 
        def visit(self, node): 
            methname = 'visit_' + type(node).__name__ 
            meth = getattr(self, methname, None) 
            if meth is None: 
                meth = self.generic_visit 
            return meth(node)
    def generic_visit(self, node): 
        raise RuntimeError('No {} method'.format('visit_' + type(node).__name__))
    
    class Evaluator(NodeVisitor):
        def visit_Number(self, node): 
            return node.value
        def visit_Add(self, node): 
            # note there's a recursion here
            return self.visit(node.left) + self.visit(node.right)
        def visit_Sub(self, node): 
            return self.visit(node.left) - self.visit(node.right)
        def visit_Mul(self, node): 
            return self.visit(node.left) * self.visit(node.right)
        def visit_Div(self, node): 
            return self.visit(node.left) / self.visit(node.right)
        def visit_Negate(self, node): 
            return -node.operand

    e = Evaluator()
    e.visit(t1) # calling visit_Number first. 
    print(e.visit_Add(t1))
    from arepl_dump import dump
    dump()

def test_cyclic_datastructure():
    """
    1. It seems like python 3.8+ can detect cyclic data structures. But for older versions, we need weakref
    2. Garbage collector works on reference counting. Objects whose reference count is not 0 will not be garbage collected
    """
    # 1 
    class A:
        def __init__(self):
            print("Object A Created")
            
        def __del__(self):
            print("Object A Destroyed")
            
    class B:
        def __init__(self):
            print("Object B Created")
            
        def __del__(self):
            print("Object B Destroyed")

    #creating two objects
    a = A()
    b = B()

    #setting up circular reference
    a.b = b
    b.a = a

    # #deleting objects
    # del a
    # del b
    
    import gc
    # force garbage collection
    gc.collect()
    
    # 2 
    import weakref
    a. b = weakref.ref(b)
    b. a = weakref.ref(a)

            

if __name__ == "__main__":
    # test_type()
    # test_metaclass()
    # test_ORM()
    # test_property_change()
    # test_lazy_attr()
    # test_base_field()
    # test_abc()
    # test_stateful_class()
    # test_visitor_pattern()
    # test_cyclic_datastructure()
    # test_decorator()
    test_decorator_deep_dive() 
    