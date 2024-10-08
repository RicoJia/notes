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

    # 2 - You are creating a class here
    def fn(self):
        pass
    Foo_equivalent = type('Foo', (object,), dict(foo=fn))
    f = Foo_equivalent()
    # see the same as above
    print(type(f), type(Foo_equivalent))
    
def test_metaclass():
    """
    1. Except for type(), you can create a class using metaclass, i.e., a class is an instantiation of a class
        - https://lotabout.me/2018/Understanding-Python-MetaClass/
        - How a class is created:
            1. 找 metaclass
            2. call meta.__prepare__ to prepare namespace (the __dict__)
                - must return a dict, but can be used to store order of attributes by returning a OrderedDict  
            3. execute body of the class definition, store methods into __dict__
            4. Meta's meta's __call__
            5.  call meta.__new__ in the metaclass. (cls is type) since we're initializing the class
            6.  call meta.__init__ to finalize the class. (cls is meta already)
                - Just used to wrap up class creation. Either __new__ or __init__.
            7  call meta.__call__ (cls is class A, type.__call__() returns class A object)
                - Used to control instance creation of the following class.
            8  call class A.__new__ to create the object
            9  call class A.__init__ to change values of each attribute.
    2. Mix-in class: in MyList, list is a mix-in 
        - Mix-in technically cannot be instantiated. Its functions will be inherited. 
        - Multiple Mix-ins can be used so this relies on Python's multi-inherited 
    3. __new__ vs __init__:
        1. __new__: construct cls as a class object. 
        2. __init__ is called after __new__. So you can set attributes directly on cls
            - But updating attrs in __init__ is not effective. Per official document: 
            "the object provided as the namespace parameter is copied to a new ordered mapping and the original object is discarded. 
            The new copy is wrapped in a read-only proxy"
    """
    # have Metaclass at the end of the name by convention
    from collections import OrderedDict
    class ListMetaclass(type):
        # so you can add another attribute to a class. At the end, return the class using type
        @classmethod
        def __prepare__ (cls, clsname, bases):
            print(f"prepare: {cls}, {clsname}")
            return OrderedDict()
            
        def __new__(cls, class_name, bases, attrs):
            # 1. bases is empty if there's the class does not have a parent class
            # 2. it allocates memory, creates attributes, then 
            # 3. cls being passed in is the meta class itself
            # 4. the official doc refers attrs as namespace
            attrs['add'] = lambda self, x: self.append(x)
            #TODO 
            # print(f"new: attrs:", attrs)
            print("meta new, ", type(cls))
            # here sometimes you will see super() as well
            return type.__new__(cls, class_name, bases, attrs)

        def __init__(cls, name, bases, namespace, **kwargs) -> None:
            # super().__init__(name, bases, namespace, **kwargs)
            print("meta class init, called as last step to create a class: ", type(cls))

        def __call__(cls, *args, **kwargs):
            # return an instance
            instance = type.__call__(cls, *args, **kwargs)
            print("metaclass call", " instance: ", type(cls), type(instance), instance, *args, **kwargs, )
            return instance

    # with metaclass, the magic happens
    # Meanwhile, you need to inherit from the list class
    class MyList(list, metaclass=ListMetaclass):
        def __init__(self, a = 1):
            print("regular class init")
        def foo(self):
            pass
        def __new__(cls, *args, **kwargs):
            print("instance new")
            return super().__new__(cls, *args, **kwargs)
    l = MyList(1)
    l.foo()
    # l.add(3)
    # print(l)

def test_metaclass_inheritance_and_init():
    class Baz:
        def __init__(self, string, fo):
            #TODO Remember to remove
            # print(f'Rico: baz')
            pass

    class CatchOnEnterFailuresMeta(type):
        def __new__(meta_cls_itself, class_name, bases, attrs):
            if "on_enter" in attrs:
                old_on_enter = attrs["on_enter"]
                def failure_catchable_on_enter(self, userdata):
                    #TODO Remember to remove
                    print(f'Rico: ****************==============hello!')
                    old_on_enter(self, userdata)
                attrs['on_enter'] = failure_catchable_on_enter
            return type.__new__(meta_cls_itself, class_name, bases, attrs) 

        def __init__(cls, name, bases, attrs, **kwargs):
            # This doesn't do anything
            if "on_enter" in attrs:
                old_on_enter = attrs["on_enter"]
                def init_on_enter(self, userdata):
                    print(f'Rico: ****************==============hello2!')
                    old_on_enter(self, userdata)
                attrs['on_enter'] = init_on_enter
                # cls.on_enter = failure_catchable_on_enter
            print(f'Rico: {attrs}')
            super().__init__(name, bases, attrs)
        
    class Foo(metaclass=CatchOnEnterFailuresMeta):
        def __init__(self):
            super().__init__()
        def on_enter(self, userdata):
            print(f"on enter")

    # class Child(Foo):
    #     pass
    # Foo().on_enter(123)
    Foo.on_enter
    Foo.__dict__

def test_metaclass_equivalent():
    # Equivalent way to create a metaclass
    class MetaCls(type):
        """A sample metaclass without any functionality"""
        def __new__(cls, clsname, superclasses, attributedict):
            print("clsname:", clsname)
            print("superclasses:", superclasses)
            print("attrdict:", attributedict)
            return super(MetaCls, cls).__new__(cls, \
                        clsname, superclasses, attributedict)
    
    C = MetaCls('C', (object, ), {})
    class D(C):
        pass

##############################################################
### Metaclass Applications
##############################################################

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
                print(f"1 Model: kw: {attrs}")
                return type.__new__(cls, name, bases, attrs)
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
    
##############################################################
### Mixin
##############################################################
def test_mixin():
    """
    1. If you want some functions to go to other classes, but they're not enough for 1 inherited base class, use mixins
        1. Mixins are NEVER meant to be instantiated 
        2. They shouldn't have attributes in general, cuz they never know who use them
            - use __slots__ = () to ensure that. 
    2. Use the decorator (see next function), since it's clearer and faster
    """
    class SetKeyOnce(object):
        __slots__ = ()
        # Going to be used on container.
        def __setitem__(self, key, value):
            print("self: ", self)
            if key in self.keys():
                raise RuntimeError("Key already exists")
            # where the heck did this super() come in? TODO
            # Tricky: MRO resolution order: from left to right. At each step of inheritance, 
            # super().inherited_function() will call the next available func in the list!
            super().__setitem__(key, value)
            
    from collections import defaultdict
    class DefaultDictSetOnce(SetKeyOnce, defaultdict):
        pass
    ddso = DefaultDictSetOnce()
    ddso["sd"] = 3
    # ddso["sd"] = 4
    print(ddso)
    print(DefaultDictSetOnce.__mro__)
    
    
def test_decorator_replacing_mixin():
    """
    1. If you can use decorator, don't do mixin. This is 100% faster!
    """

    # Base class. Uses a descriptor to set a value 
    class Descriptor: 
        def __init__(self, name=None, **opts): 
            self.name = name 
            for key, value in opts.items(): 
                setattr(self, key, value)
        def __set__(self, instance, value): 
            instance.__dict__[self.name] = value
    # Decorator for applying type checking 
    # In a classic class decorator, we are allwoed to have only one argument in class_decorator(cls)
    # TRICKY: 
    # But we want to pass in expected_type first. So, when putting on top of class,
    # first time, Typed(int) will make expected_type = int, and Typed will stop taking in args
    # then it will return wrap, which is the real class decorator
    # Second time, wrap will really take in cls, and execute the rest of Typed

    def Typed(expected_type, cls=None):
        if cls is None:
            # equivalent to return a wrapped function
            def wrap(cls):
                return Typed(expected_type, cls)
            # return lambda cls: Typed(expected_type, cls)
            return wrap
        super_set = cls.__set__ 
        def __set__(self, instance, value): 
            if not isinstance(value, expected_type): 
                raise TypeError('expected ' + str(expected_type)) 
            super_set(self, instance, value) 
        cls.__set__ = __set__ 
        return cls

    # Specialized descriptors 
    # equivalent to: class Integer = Typed(int, None) = wrap(cls); 
    @Typed(int) 
    class Integer(Descriptor):
        pass

    # i = Integer("some_int", val=3)
    # print(i.__dict__) 

    #decorator for unsigned values: descriptor_cls + __set__. Then used as a class variable
    def Unsigned(cls): 
        super_set = cls.__set__
        def __set__(self, instance, value): 
            if value < 0: 
                raise ValueError('Expected >= 0') 
            super_set(self, instance, value) 
        cls.__set__ = __set__ 
        return cls

    @Unsigned 
    class UnsignedInteger(Integer): 
        pass
    
    class Foo:
        u = UnsignedInteger()
    f = Foo()
    try: 
        f.u = -3
    except ValueError:
        print("This is how to use decorator to add __set__ to a descriptor")

if __name__ == "__main__":
    test_metaclass()
    # test_mixin()
