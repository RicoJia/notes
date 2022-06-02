#!/usr/bin/python3
def inheritance_basics():
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

    B().foo()   #calls "from B"

    #print_class_name(): 
    print("class name: ", B().__class__.__name__)

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
    
def test_abstract_method():
    """
    1. abstract function is parent class function with no implementation. Parent class having at least 1 abstract function is an abstract class. Same as in C++
    2. Should use ABC (abstract base class), else there won't be error
        - but abstractmethod doesn't seem to do anything? 
    """
    from abc import ABC, abstractmethod
    class Foo(ABC):
        @abstractmethod
        def foo(self): 
            # print("foo")
            pass

    class FooC(Foo): 
        def bar(self): 
            print("bar")

    # will see error since Foo can not be instantiated
    # f = FooC()
    # f.foo()

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

def test_get_attribute(): 
    """
    1. When you call a member in class A, you will call A.__getattribute__() as well. So you might get recursion error
    2. the calling with super().__getattribute__() will access the proper attribute. Not sure why? 
    """
    class Foo: 
        def __init__(self): 
            self.dummy = 100
        def __getattribute__(self, s):
            """
            s is a string.
            """
            print ("1")
            # calling this directly = recursion
            # self.dummy
            # call with super() instead, which is equivalent to self.dummy with no ambiguity
            print(super().__getattribute__("dummy"))
    f = Foo()
    f.dummy

def test_hasattr(): 
    """
    1. hasattr can be helpful
    """
    class Foo: 
        f = 1
    foo = Foo()
    print(hasattr(foo, "f"), hasattr(foo, "b"))

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
    import sys
    print("sys.getsizeof object f's size: ", sys.getsizeof(f))
    from pympler import asizeof
    print("pympler getsizeof: ", asizeof.asizeof(f))

if __name__ == "__main__": 
    # inheritance_basics()
    # test_class_variable()
    # test_abstract_method()
    # test_enum()
    # test_get_attribute()
    # test_sort_by_attr()
    # test_hasattr()
    # test_class_representations()
    # test_task_managed_class()
    test_slots()
