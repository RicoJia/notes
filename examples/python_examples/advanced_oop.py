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
        - use @name.setter, @name.deleter setters and deleters
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
            # instance is the upper level object. If Int is a class variable, cls is the class, instance is None, and it's common practice to return the object itself.
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


