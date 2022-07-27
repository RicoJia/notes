#!/usr/bin/python3
##############################################################
### OOP
##############################################################

from functools import total_ordering

def inheritance_basics():
    """
    1. What does super() do?
    2. name mangling using __var, also do something like lambda_ for vars that has names like keys
        - __mangled_var can not be inherited, because it will be called self.Class_Name_mangled_var
    """
    # Parent class calls function in derived class
    class A:
        def foo(self):
            self.bar()

        def bar(self):
            print("from A")

    class B(A):
        def foo(self):
            super().foo()

        def bar(self):
            print("from B")
            # 2
            self.lambda_ = 3

    B().foo()   #calls "from B"

    #print_class_name(): 
    print("class name: ", B().__class__.__name__)        
    print(f"sdf")        
    
def test_enum(): 
    # SIMPLE WAY TO DO THIS
    Pen, Pencil, Eraser = range(0, 3)
    ls = [1,2,3]
    print(ls[Pen])

    from enum import Enum
    # Enum(name_of_enumeration, all fields, with 1,2,3...)
    Animals = Enum('Animals', 'ant bee cat dog')
    print(kkk.ant.value)
    class TestEnum(Enum):
        DOG = 1
        CAT = 2
    print(TestEnum.CAT.value)

def test_task_managed_class():
    """
    1. Task Management works with thread, and file system
    2. __exit__
        1. Contains instructions to properly close resource handler.
            - If exception is raised, then type, value, traceback (tb) is passed. Else they're none
            - type: type of exception
            - value: like division by zero
            - traceback: the whole traceack
    """
    class Divide:
        def __init__(self, num1, num2):
            self.num1 = num1
            self.num2 = num2
        def __enter__(self):
            print("acquired resources")
            return self
        def divide(self):
            return self.num1/self.num2
        def __exit__(self, exc_type, exc_val, exc_tb):
            print("__exit__")
            print("exc_type: ", exc_type, " \n exc_val: ", exc_val, "\n exc_tb: ", exc_tb)

    with Divide(3,1) as d:
        d.divide()
    print("=======")
    with Divide(3,0) as d:
        d.divide()

def test_decorator_in_class():
    '''
    1. The main purpose to have decorator funcs in a class is to store data together
        - property() returns a property descriptor with setter, getter, deleter 
    '''
    from functools import wraps
    # 1 decorator as a class function
    class Foo:
        def decorator1(self, func):
            @wraps(func)
            def wrapper(*args, **kwargs):
                print("regular func wrapper")
                return func(*args, **kwargs)
            return wrapper
        @classmethod
        def decorator2(cls, func):
            @wraps(func)
            def wrapper(*args, **kwargs):
                print("class method wrapper")
                return func(*args, **kwargs)
            return wrapper
    
    f = Foo()
    
    @f.decorator1
    def func1():
        print("func1")
    @Foo.decorator2
    def func2():
        print("func2")
    func1()
    func2()
    # how's class method wrapper used

    class Bar:
        # bar is 
        bar = property()
        # getter function
        # @property
        #   def bar(self):
        #       ...
        # equivalent to
        # bar = property(bar)   # a class property descriptor 
        @bar.getter
        def bar(self):
            return "bar"
        @bar.setter
        def bar(self, value):
            print(value)
            pass
    b = Bar()
    b.bar = 3
    print(b.bar)
                
def test_slots():
    """
    1. __slots__ makes the class a small list instead of a dictionary. 
        - any additional attribute will throw "object Foo has no attribute.."
        - so the class is a tuple, not a dictionary.
        - saves a lot of memory
        - list's get and set uses only O(1), so they're faster
    2. sys.getsizeof(obj) is how you can see the size of it, but it will ignore referenced object such as self.__dict__
    """
    class Foo:
        __slots__=["name", "grade", "f1", "f2"]
        def __init__(self, name, grade, f1=2, f2 = 4):
            self.grade = grade
            self.name = name
            self.f1 = f1
            self.f2 = f2
            # self.dum = 1

    f = Foo("1", 2)

def test_abc():
    """
    1. An abc class's essence is that it cannot be instantiated. So it can be used as an interface.
        - needs ALL FUNCS to be abstract functions
        - abstract function is parent class function with no implementation. Parent class having at least 1 abstract function is an abstract class. Same as in C++
    2. You can force another class to be "compatible" with the base
    """
    # 1 
    from abc import ABC, abstractmethod
    class Base(ABC):
        # equivalent to meta=ABCMeta
        def __init__(self, dummy):
            pass
        def some_Fun(self):
            pass
    
    class Foo(Base):
        pass
    f=Foo("dummy")
    
    class Baz():
        pass
    # 2
    Foo.register(Baz)
    b = Baz()
    print(isinstance(b, Base))   
    
def test_alternate_constructor():
    """
    1. Say we want to call another function with some predefined args, use a classmethod that calls __new__()
        - classmethod: returns a classmethod, which has access to class state
    2. It can be also useful when we want to manually fill in attributes,
        - Use __setattr__
    """
    # 1 
    from time import localtime
    class Date:
        def __init__(self, yr, mo, dd):
            self.yr = yr
            self.mo = mo
            self.dd = dd
        @classmethod
        def today(cls):
            print("creating alternate constructor")
            new_instance = cls.__new__(cls) 
            t = localtime()
            new_instance.yr = t.tm_year
            new_instance.mo = t.tm_mon
            new_instance.dd = t.tm_mday
            return new_instance
    random_date = Date(11,22,33)
    today = Date.today()
    
    # 2
    json_data = { 'year': 2012, 'month': 8, 'day': 29 }
    class DateJSON(Date):
        @classmethod
        def today(cls, json_data):
            new_instance = cls.__new__(cls)
            for key, val in json_data.items():
                setattr(new_instance, key, val)
            return new_instance
    date_from_json = DateJSON.today(json_data)
    # json_data is a child instance of Date
    print(vars(date_from_json), isinstance(date_from_json, Date))
    
def test_garbage_collect():
    """
    1. Garbage is collected when ref = 0. But by default that's thru simple reference. 
        - __del__ is not called when we have cyclic reference? 
            - So don't implement __del__, else we will get MEMORY LEAK
        - Great explanation: https://zhuanlan.zhihu.com/p/124290355
    2. To combat this, use weakref
    3. GC happens when program exits, or gc.collect(). There might be generational garbage collection, 
    but we don't know when that will happen.
        - A generation is a linkedlist that tracks nodes how many times they've been deleted. 
            - If a node has been deleted before, then it moves down by a genenration
            - Older generation is gc'ed at a lower frequency.
    """
    import gc, sys
    class Foo:
        def __del__(self):
            print("This is del")
    f = Foo()
    a = f
    # see 3
    print('引用次数：', sys.getrefcount(f))
    del a
    # see 2, the other one is Foo() itself
    print('引用次数：', sys.getrefcount(f))
    # won't see anything, it just releases the memory
    gc.collect()
    print("finish")
    
##############################################################
### Attributes
##############################################################

def test_class_variable(): 
    """
    1. Class variable is like static variable. But more versatile. You can change the value for one specific instance, or change the class variable itself.
        - Python you can add another variable that doesn't exist on the go. So once you change the value for a specific instance, you create the instance's own copy
    2. Need default value
    3. Class itself is a dictionary. Can use this to see class variables
    """
    class Foo: 
        var = 1
    f = Foo()
    g = Foo()
    Foo.var = 2
    print(g.var, f.var)
    # can see what's inside the class, and what's inside the instance, using dict
    print(Foo.__dict__, f.__dict__)
    
    # create the instance's own copy, which is NOT being affected by the class variable anymore.
    f.bar = 12
    Foo.var = 2
    f.var = 12
    print("Foo var is", Foo.var, "while f.bar : ", f.bar, "and f.var: ", f.var, " and g.var: ", g.var)
    print(Foo.__dict__, f.__dict__)

def test_sort_by_attr():
    """
    1. by default, sorted is ascending order. operator.attrgetter
    2. attrgetter gets attribute like attrgetter("attr")(obj)
    3. If we are sorting a dictionary, itemgetter is the way to go.
        - bit faster than using lambda, as well as attrgetter
    """
    # 1
    class Foo():
        def __init__(self, name, grade):
            self.name = name
            self.grade = grade
    ls = [Foo("Lee", 12), Foo("Chen", 10), Foo("Yoon", 9), Foo("Jia", 11)]
    from operator import attrgetter
    ls = sorted(ls, key=attrgetter("grade"))
    [print("sorted list by attributes: ", item.name) for item in ls]
    
    # 2
    print(f"attrgetter: {attrgetter('name')(ls[0])}")

    # 3 
    di = [{"name": "Lee", "grade": 12}, {"name": "Chen", "grade": 10}, {"name": "Yoon", "grade": 9}]
    from operator import itemgetter
    ls = sorted(di, key=itemgetter("grade"))
    print("sorted list: ", ls)
    
def test_get_attribute(): 
    """
    1. When you call a member in class A, you call A.__getattribute__() as well. So you might get recursion error
    2. the calling with super().__getattribute__() will access the proper attribute, because of MRO
    3. vs __getattr__: __getattr__ is called if no attribute is found, otherwise won't be called. So it can be used to "mute" AttributeError. But __getattribute__ will raise the error
        - __getattribute__() will be called first if both present
    """
    class Foo: 
        def __init__(self): 
            self.dummy = 100
        def __getattribute__(self, attr_name):
            """
            s is a string.
            """
            print ("1")
            # calling this directly = recursion
            # self.dummy
            # call with super() instead, which is equivalent to self.dummy with no ambiguity
            print(super().__getattribute__(attr_name))
    f = Foo()
    f.dummy
    try:
        f.notexistit
    except AttributeError:
        print("__getattribute__ will raise AttributeError")

    class Baz:
        b = 1
        def __getattr__(self, attr_name):
            print(f"attr name: {attr_name}")

    baz = Baz()
    baz.b
    baz.b_notexist

def test_hasattr():
    """
    1. hasattr can be helpful
    """
    class Foo: 
        f = 1
    foo = Foo()
    print(hasattr(foo, "f"), hasattr(foo, "b"))

def test_call():
    """
    __call__() is invoked when f()
    """
    class Foo():
        def __init__(self):
            #TODO 
            print(f"init")
        def __call__(self, name):
           #TODO 
           print(f"call {name}") 

    f= Foo()
    f("baz")
    
def test_call_function_by_name():
    """
    1. 2 ways: getattr, or operator.methodcaller
    """
    # 1
    class Calculator:
        def sum(self,x, y):
            return x+y
    c = Calculator()
    print(getattr(c, 'sum')(2,4))
    
    # 2
    import operator
    print(operator.methodcaller('sum', 4,5)(c))
    

def test_super():
    """
    1. if you call functions directly through parent classes' names, you will see functions exected twice
        - If you use super(), just one gets called, because it uses multiple resolution order (MRO)
        - super() returns a super object, which allows you to access parent class attributes

    2. MRO is a list of classes sorted using merge-sort
        - mro must be accessed thru class, like obj.__class__
    3. ref: https://python-reference.readthedocs.io/en/latest/docs/functions/super.html
    4. MRO resolution order: from left to right. At each step of inheritance, 
        - super().inherited_function() will call the next available func in the list!
    """
    class Base:
        def __init__(self):
            print("Base")

        def foo(self):
            print("Base")

    class Foo(Base):
        def __init__(self):
            print("Foo")
        def f(self):
            Base.foo(self)
            print("foo")
    class Bar(Base):
        def __init__(self):
            print("Bar")
        def f(self):
            Base.foo(self)
            # super().f()
            print("Bar")
    
    # Here you see 2 inits. if using super(), you will see one
    class Baz(Bar,Foo):
        def __init__(self):
            Bar.__init__(self)
            Foo.__init__(self)
        # def f(self):
        #     pass

    baz = Baz()
    print(baz.__class__.__mro__)
    baz.f()

def test_class_representations(): 
    """
    1. __repr()__ is when you type obj; __str()__ is print(obj)
        - type(self).__name__ is how to get name inside a class
    2. repr() calls __repr__()
        - eval("Foo") is to run the command as a string
    """
    class Foo:
        def __init__(self, f):
            self.f = f
        def __repr__(self):
            return f"repr {type(self).__name__, self.__dict__}"
        def __str__(self):
            return f"str {type(self).__name__, self.__dict__}"

    p = Foo(f=1)
    print("p: ", p)
    print(repr(p))

def test_class_defined_later():
    """
    1. Class can use a class that's defined later, 
        - But all classes need to be read before the first instance
    """
    class Baz:
        def b(self):
            self.b_class =Foo()
    class Foo:
        pass
    b = Baz()
    b.b()
    print(vars(b))
    
def test_comparison():
    """
    use functools.total_ordering to infer all the comparison methods.
        - __eq__ must be defined, then define another comparison func
    it basically decides which comparison func has been defined 
    """    
    def rico_total_order(cls):
        cls.__lt__ = lambda self, other: not(cls.__eq__(self, other) or cls.__gt__(self, other))  
        cls.__le__ = lambda self, other: not(cls.__gt__(self, other))  
        cls.__ge__ = lambda self, other: cls.__gt__(self, other) or cls.__eq__(self, other)  
        cls.__ne__ = lambda self, other: not cls.__eq__(self, other)
        return cls
 
    from functools import total_ordering
    # @total_ordering
    @rico_total_order
    class House:
        def __init__(self, area) -> None:
            self.area = area
        def __eq__(self, other):
            return self.area == other.area
        def __gt__(self, other):
            return self.area > other.area
    h1 = House(3)
    h2 = House(5)
    print(h1 >= h2)
   
    # #TODO
    # from arepl_dump import dump
    # dump()

def test_return_cached_instance():
    """
    Many times if you have an instance with the same args, you can return 
    the same instance
        - return a weak ref to the instance. Because the cache should not interfere
    """
    class Foo:
        def __init__(self, name):
            self.name = name

    import weakref 
    _spam_cache = weakref.WeakValueDictionary()
    def get_foo(name):
        if name not in _spam_cache:
            f = Foo(name)
            _spam_cache[name] = f
        return _spam_cache[name]


        
if __name__ == "__main__": 
    # test_alternate_constructor()
    # test_class_defined_later()
    # test_call_function_by_name()
    # test_garbage_collect()
    # test_comparison()
    # test_call() 
    test_decorator_in_class()
