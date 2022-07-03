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
    2. Mix-in class: in MyList, list is a mix-in 
        - Mix-in technically cannot be instantiated. Its functions will 
        be inherited. 
        - Multiple Mix-ins can be used so this relies on Python's multi-inherited 
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
    # # Decorator for applying type checking 
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
