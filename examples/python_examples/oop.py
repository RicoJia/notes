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
    print(f.bar)
    f.var = 12
    print(Foo.__dict__, f.__dict__)

def test_sort_by_attr():
    """
    1. by default, sorted is ascending order. operator.attrgetter
    """
    class Foo():
        def __init__(self, name, grade):
            self.name = name
            self.grade = grade, name, grade
    ls = [Foo("Lee", 12), Foo("Chen", 10), Foo("Yoon", 9), Foo("Jia", 11)]
    from operator import attrgetter
    ls = sorted(ls, key=attrgetter("grade"))
    [print("sorted list by attributes: ", item.name) for item in ls]
    
            

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

if __name__ == "__main__": 
    # inheritance_basics()
    # test_class_variable()
    # test_enum()
    # test_get_attribute()
    test_sort_by_attr()
