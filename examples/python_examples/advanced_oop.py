
##############################################################
### Property, Descriptor
##############################################################
from inquirer import password


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
    
def test_abc():
    """
    1. An abc class's essence is that it cannot be instantiated. So it can be used as an interface.
        - needs ALL FUNCS to be abstract functions
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
##############################################################
### Metaclass
##############################################################
def test_type():
    """
    1. a regular class is of type 'type'
    2. Beauty of interpreted language: class can be created on the go, using type.
        - type(class_name, inheritance, functions)
        - Meanwhile compiled languages will have this compiled first
    """
    # 1
    class Foo:
        def foo(self):
            pass
    # see class Foo, class "type"
    f = Foo()
    print(type(f), type(Foo))

    # 2
    def fn(self):
        pass
    Foo_equivalent = type('Foo', (object,), dict(foo=fn))
    f = Foo_equivalent()
    # see the same as above
    print(type(f), type(Foo_equivalent))
    
def test_metaclass():
    """
    1. Except for type(), you can create a class using metaclass, i.e., a class is an instantiation of a class
        - __new__ is called with attributes passed in. Here it assembles things together.
    """
    # have Metaclass at the end of the name by convention
    class ListMetaclass(type):
        # so you can add another attribute to a class. At the end, return the class using type
        def __new__(cls, class_name, bases, attrs):
            attrs['add'] = lambda self, x: self.append(x)
            #TODO 
            print(f"attrs:", attrs)
            # here sometimes you will see super() as well
            return type.__new__(cls, class_name, bases, attrs)

    # with metaclass, the magic happens
    # Meanwhile, you need to inherit from the list class
    class MyList(list, metaclass=ListMetaclass):
        def foo(self):
            pass
    l = MyList()
    l.add(3)
    print(l)

def test_ORM():
    """
    ORM: object relational mapping, projecting a class to a form. (No need to manage SQL directly?)
        1. When you create a class (before creating an object), the class will call __new__() of its own or its parent class.
            1. Its parent class gets created first
            2. The class gets created next, with class variables being in attrs
                - here you can examine if the class variables and their types meet certain standards
        2. Later, when you create an object with args, the args can be type-checked if above you're passing in a descriptor
    """
    # 0, see descriptor for more details
    class Field(object):
        def __init__(self, name, column_type):
            self.name = name
            self.column_type = column_type
        def __str__(self):
            return '<%s:%s>' % (self.__class__.__name__, self.name)
    class StringField(Field):
        def __init__(self, name):
            super(StringField, self).__init__(name, 'varchar(100)')

    class IntegerField(Field):
        def __init__(self, name):
            super(IntegerField, self).__init__(name, 'bigint')

    # 1
    class ModelMetaclass(type):
        def __new__(cls, name, bases, attrs):
            if name=='Model':
                #TODO
                print(f"1 Model: kw: {attrs}")
                return type.__new__(cls, name, bases, attrs)
            #TODO
            print(f"2 ModelMetaclass: kw: {attrs}")
            print('Found model: %s' % name)
            mappings = dict()
            for k, v in attrs.items():
                if isinstance(v, Field):
                    print('Found mapping: %s ==> %s' % (k, v))
                    mappings[k] = v
            for k in mappings.keys():
                attrs.pop(k)
            attrs['__mappings__'] = mappings # 保存属性和列的映射关系
            attrs['__table__'] = name # 假设表名和类名一致
            return type.__new__(cls, name, bases, attrs)

    """
    note Model is inherited from dict. So the only thing is: 
       - if the class name is Model, we will return it; 
       - else when class is a child class, for each item in {attr}, we examine if it's a type of Field. If so, we add it to attrs[__mappings__]
    """
    class Model(dict, metaclass=ModelMetaclass):

        def __getattr__(self, key):
            try:
                return self[key]
            except KeyError:
                raise AttributeError(r"'Model' object has no attribute '%s'" % key)

        def __setattr__(self, key, value):
            self[key] = value

        def save(self):
            fields = []
            params = []
            args = []
            for k, v in self.__mappings__.items():
                fields.append(v.name)
                params.append('?')
                args.append(getattr(self, k, None))
            sql = 'insert into %s (%s) values (%s)' % (self.__table__, ','.join(fields), ','.join(params))
            print('SQL: %s' % sql)
            print('ARGS: %s' % str(args))

    #2
    # # using ORM. save() is provided by Model, but fields are from ORM
    class User(Model):
        # calls Model.__init__() first
        id = IntegerField('id')
        name = StringField('username')
        email = StringField('email')
        password = StringField('password')

    # 创建一个实例：
    u = User(id=12345, name='Michael', email='test@orm.org', password='my-pwd')
    print("User.__dict__: ", User.__dict__)
    # 保存到数据库：
    u.save()
    print("u: ", u)

def test_singleton():
    """
    1. If we don't consider inheritance, then __new__ is enough 
        - static method __new__ is called first, to create an instance 
            object.__new__(class, *args, **kwargs)
        - person = Person('John') =>
            person = object.__new__(Person, 'John')
            person.__init__('John')
    2. Above will have problem: child's new will return the same instance as parent!
        
    """
    # 1
    class SingletonClass(object):
      def __new__(cls):
        if not hasattr(cls, 'instance'):
          cls.instance = super(SingletonClass, cls).__new__(cls)
        return cls.instance

    singleton = SingletonClass()
    new_singleton = SingletonClass()

    print(singleton is new_singleton)
    singleton.singl_variable = "Singleton Variable"
    print(new_singleton.singl_variable)

    # 2 
    class SingletonChild(SingletonClass):
        pass

    singleton = SingletonClass()
    child = SingletonChild()
    print("child is singleton? ", child is singleton)
    singleton.singl_variable = "Singleton Variable"
    print(child.singl_variable)

    # 3
    class Singleton(type):
        # why type?
        def __init__(self, *args, **kwargs):
            self.__instance = None
            super().__init__(*args, **kwargs)
        # what does call do here?
        def __call__(self, *args, **kwargs):
            print("__call__")
            if self.__instance is None:
                self.__instance = super().__call__(*args, **kwargs)
                return self.__instance
            else:
                return self.__instance

    #metaclass?
    class Spam(metaclass=Singleton):
        def __init__(self):
            print("Spam")

    class SpamChild(Spam):
        pass
    s = Spam()
    s1 = Spam()
    s2 = SpamChild()
    print("s is s1: ", s is s1)
    print("s is s2: ", s is s2)

def test_singleton_thread(self):
    import threading
    from queue import Queue
        
    class SingletonWriter:
        def __new__(cls):
            if not hasattr(cls, 'instance'):
                setattr(cls, 'instance', super(SingletonWriter, cls).__new__(cls))
                setattr(cls, 'queue', Queue())
                setattr(cls, 'writer_thread', threading.Thread(target=SingletonWriter.writer_func, args=(cls.queue, )))
                cls.writer_thread.setDaemon(True)
                cls.writer_thread.start()  
            return cls.instance
        def put(self, payload):
            self.queue.put(payload)
        @staticmethod
        def writer_func(queue: Queue):
            while True:
                msg,logtimer_instance = queue.get()
                logtimer_instance.dummy += 1
                print(logtimer_instance.dummy)
                print(msg)
            
    class LogTimerMultithreaded:
        def __init__(self):
            self.singleton_writer = SingletonWriter()
            self.dummy = 0
        def start(self, msg):
            self.singleton_writer.put((msg, self))

    ltm1 = LogTimerMultithreaded()
    ltm2 = LogTimerMultithreaded()

    print(ltm1.singleton_writer is ltm2.singleton_writer)

    ltm1.start("hello world")
    ltm2.start("hello world2")
    ltm1.start("hello world")
    ltm2.start("hello world2")



if __name__ == "__main__":
    # test_type()
    test_metaclass()
    # test_ORM()
    # test_property_change()
    # test_lazy_attr()
    
    # test_base_field()
    # test_abc()