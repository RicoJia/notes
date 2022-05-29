#========================================================================
## Iterator & Generator Basics
#========================================================================
def iterator_basics(): 
    """
    1. Iterable is something you can iterate over, like a list, dictionary, using the iterator inside them. 
        - Called Iterable protocol
    2. you must have ```__iter__``` and ```__next__``` for iterators
        - Use next() on iterables
        - Iterator is an object with next(). Use ```iter()``` to get an iterator. 
        - Once we have reached the bottom of an iterator, raises StopIteration. 
    3. iterable is an object with __iter__(), which returns an iterator
        - Use ```for i in ``` to loop over
    4. . dict is an iterable. But iter(di) gives you the keys
    """
    # 1
    class BdayIterator:
        def __init__(self): 
            self.count = 0
        def __next__(self):
            self.count += 1
            if self.count < 10: 
                return 100
            else:
                raise StopIteration   #or do this

    # __iter__ returns an iterator
    bday = BdayIterator()

    # manually iterating
    while True: 
        # __next__ returns the next value of the iterable
        try: 
            num = next(bday) 
        except StopIteration: 
            print("Iteration stopped")
            break

    # 2
    class BdayIterable():
        def __iter__(self): 
            return BdayIterator()
    for i in BdayIterable(): 
        print("bday iterable: ", i) 

    # 4
    ls = [1,2,3]
    ls_iter = iter(ls)
    # see 1, 2
    print(next(ls_iter), next(ls_iter))

    # create an iterable from dictionary
    di =  {"one": 1, "two":2}
    dict_iterator = iter(di)
    print(next(dict_iterator))

def generator_basics(): 
    """
    1. Generators are iterators
        - Use yield, which is like return, but returns a generator object, which can be iterated only once. 
        - Do not store all values in memory at once, generated on the fly
        - It's a generator function, which has __next__(), like iterator. But it could be easier
    2. By default, StopIteration exception when the last value is yielded. So you don't have to write this explicitly
    3. So use for i in.... For loop calls next(iter(iterable)), and returns a generator
    4. Design Patterns: 
        1. Good for stuff that's generated indefinitely, real time
        2. Good for search, which decouples search process from the upper stream code
    """
    def Bday_Gen():
        yield 1
        yield 2
    # 1. you get a StopIteration exception
    bday_gen = Bday_Gen()
    while (True):
        try:
            print(next(bday_gen))
        except StopIteration:
            break
    print("Done")

    # 2. Use for loop instead
    g = (x for x in range(10))
    print(next(g))   # this is totes valid
    print("===============")
    # can keep on iterating 
    for i in g: 
        print(i)

    # 3. Illegal since it can be iterated only once
    # for i in generator:     

    # 4. use for loop 
    bday_gen = Bday_Gen()
    print("===============")
    for b in bday_gen: 
        print(b)

    # 5 generator is good design pattern for search
    ls = [1,2,3,4,5,6,7,8, 2, 2]
    def search(num): 
        for n in ls: 
            if n == num: 
                yield n
    for s in search(2): 
        print(s)

def generateor_uses(): 
    """
    1. Elegant way to reduce and transform data. max(), sum(), join(), no need to create a list
        - alternatively, min(dic, key = ...)
    """
    s = ('ACME', 50, 123.45)
    # ","("abc") = "a,b,c"
    print(",".join(str(x) for x in s))

    portfolio = [ {'name':'GOOG', 'shares': 50}, {'name':'YHOO', 'shares': 75}, {'name':'AOL', 'shares': 20}]
    print(min(p["shares"] for p in portfolio))
    nums = [1,2,3,45]
    print("sum using generator: ", sum(n for n in nums))

def test_yield_from(): 
    """
    1. yield from => (python 3.3)
        for i in generator_func_yielded_from: 
          yield i
        - do not put this in your main function, else your main function will become a generator function, and needs to be called next on!
    2. After yield, you can return as well
        - You will see that as StopIteration: return_value if you call the generator using next()
        - After everything in yield from is ended, what's after will be finished
        - If you manually call next() just enough times to finish yielding, what's after yield from won't be executed, which is consistent with yield's behaviour
    """
    # 1
    def test(n): 
        i = 0
        while i < n: 
            print("test i: ", i)
            yield i
            i+=1
    def yield_from_func(n): 
        print("Start: ")
        j = yield from test(n)
        print("j: ", j)
        print("========")
        yield from test(n)
        print("End: ")

    # This line doesn't do anything, sense it's a generator
    yield_from_func(4)
    for i in yield_from_func(4): 
        print("main loop i: ", i)

    # 2
    print("After yield, you can return as well. But")
    print("You will see that as StopIteration: return_value if you call the generator using next()")
    def some_gen(): 
        yield 0
        yield 1
        return "Done0"
        # this will not be executed
        return "Done1"

    g = some_gen()
    print(next(g))
    # see StopIteration: Done
    # print(next(g))

    print("After yield, you can return, but you won't see the return value if you don't use yield from: ")
    g = some_gen()
    for i in g: 
        print(i)

    def test_some_gen():
        g = some_gen()
        res = yield from g
        print(res)
        print("After everything in yield from is ended, what's after will be finished")
    print("using yield from: ")
    for i in test_some_gen():
        print(i)
    print("If you manually call next() just enough times to finish yielding, what's after yield from won't be executed, which is consistent with yield's behaviour")
    tsg = test_some_gen()
    next(tsg)
    next(tsg)
    # next(tsg) # this will finish the part after yield from, but will also raise stopiteration

def test_reversed(): 
    """
    1. reversed(iterable) is to call __reversed()__
    """
    class MagicNumberIterable: 
        def __init__(self): 
            self.count = 0
        def __iter__(self): 
            while self.count < 10: 
                yield self.count
                self.count += 1
            # reset it so you can iterate multiple times
            self.count = 0
        def __reversed__(self): 
            self.count = 9
            while self.count >= 0: 
                yield self.count
                self.count -= 1
    mni = MagicNumberIterable()
    for i in mni: 
        print("directly on iterable: ", i)
    # When we call iter on an iterator it will always give us itself back. So in for loop it will be iter(iter(mni)), which is fine
    for i in iter(mni): 
        print("iter: ", i)
    for i in reversed(mni): 
        print("reversed: ", i)

def test_iterable_extra_state(): 
    """
    1. Have extra state in the object: make iterable and add the state to it
        - enumuerate(ls, start_index)
    """
    from collections import deque
    class ReaderWithExtraState:
        def __init__(self, ls): 
            self.history = deque()
            self.list = ls
        def __iter__(self): 
            for item_num, item in enumerate(self.list, 2): 
                self.history.append((item_num, item))
                yield item

    ls = [5,4,3,2,1]
    rwes = ReaderWithExtraState(ls)
    for item in rwes: 
        # you will see the history growing every step of the way
        print("history: ", rwes.history)
    
def test_slice_iterator():
    """
    1. itertools.islice(generator_func, start_id, end_id)
        - does consume the generator 

    2. itertools.dropwhile(lambda, generator_func)
    """
    # 1
    def count(n):
        while n < 200: 
            yield n
            n += 1
    c=  count(0)
    from itertools import islice
    for i in islice(c,10, 20): 
        print(i)

    # 2
    from itertools import dropwhile
    c=  count(0)
    def is_positive(n):
        return n < 6 
      
    value_list =[5, 6, -8, -4, 2]
    result = list(dropwhile(is_positive, value_list))
    print("res: ", result)

    def is_good(x): 
        return True
    ls = list(dropwhile(is_positive, c)) 
    print(ls)


#========================================================================
## Coroutine
#========================================================================
def test_coroutine_basic_idea(): 
    """
    1. A function with yield can be constructed as a generator object
    2. To start the generator object, you need to call __next__. 
        - generator object has the function send(), which is a bi-directional communcation to/from the generator
        - Note that __next__ is essentially send(None). So you're only retrieving the yielded value back.
        - NOTE: TO start a generator, you MUST send(None)
    3. Each send() (including __next__()) function from consumer will start a new cycle: 
        1. producer gets consumer's message from send()
        2. Producer does its thing 
        3. Producer returns the last value, and WAIT at yield again
    4. The basic idea of coroutine, is to pause a function, and come back into it. Two functions can use yield to achieve bi-directional comm, and the generator pauses at the next yield
    """
    def test():
        print("generator start")
        n = 1
        while True:
            received_value = yield n
            print("received_value = %d" % received_value)
            n += 1

    # 创建generator对象, see "class generator"
    generator = test()
    print(type(generator))

    print("\n---------------\n")

    # 启动generator
    # see generator start, next_result = 1; but now received value because yield will be finished, but received_value will come from the next call
    next_result = generator.__next__()
    print("next_result = %d" % next_result)

    print("\n---------------\n")

    # 发送值给yield表达式
    yielded_result = generator.send(666)
    # see received 666, and yielded result = 2
    print("yielded_result = %d" % yielded_result)

def test_coroutine(): 
    """
    1. Coroutine is: one a single thread, producer -> consumer -> producer -> consumer
        - In C++, guess you can use simple functions with static vars. 
        - In Python, no static vars
        - Note that in Coroutine, yield is a consumer, consuming data from send()
    """
    def consumer():
        print("[CONSUMER] start")
        r = 'start'
        while True:
            n = yield r
            if not n:
                print("n is empty")
                continue
            print("[CONSUMER] Consumer is consuming %s" % n)
            r = "200 ok"


    def producer(c):
        # 启动generator a, see [consumer start here]
        start_value = c.send(None)
        print("start_value: ", start_value)
        n = 0
        while n < 3:
            n += 1
            print("[PRODUCER] Producer is producing %d" % n)
            r = c.send(n)
            print('[PRODUCER] Consumer return: %s' % r)
        # 关闭generator
        c.close()
    # 创建生成器
    c = consumer()
    # 传入generator
    producer(c)


def test_asyncio(): 
    """
    1. asyncio.coroutine is a wrapper that calls next() on the function. Python3.4 - 3.10
        - So it's basically a middle man, emit all values from yield, and catch return value
        - Of course, if you call it manually, return value will surface after StopIteration
        - this is where "async.run_until_complete" becomes handy

    2. async.coroutine is like this having the init_coroutine decorator
        https://www.fythonfang.com/blog/2017/5/18/Python3-Coroutine-and-asyncio
    """
    # # 1
    # import functools
    # import types
    # def init_coroutine(func):
    #     # 将普通函数变成generator
    #     @functools.wraps(func)
    #     def wrapper(*args, **kwargs):
    #         rs = func(*args, **kwargs)
    #         # res is any return value at the very end of rs
    #         res = yield from rs
    #         return res
    #     wrapper = types.coroutine(wrapper)
    #     return wrapper
    #
    # # equivalent to decorating consumer
    # print("test init_coroutine: ")
    # @init_coroutine
    # def test_c(): 
    #     yield 1
    #     return "done"
    # c = test_c()
    # print("next c: ", next(c))
    # try: 
    #     print("next c: ", next(c))
    # except StopIteration as e: 
    #     print("Of course, if you call it manually, return value will surface after StopIteration")
    #     print(e)

    # 1. does asyncio only work with yield from?
    import asyncio
    @asyncio.coroutine 
    def another_test_c(): 
        # yield 1 throws "Task got bad yield" error?
        yield 1
        print("about to be done")
        return "done"
    print("test_c is coroutine:",asyncio.iscoroutinefunction(another_test_c))
    print("test_c() is coroutine: ", asyncio.iscoroutine(another_test_c()))
    loop = asyncio.get_event_loop()
    loop.run_until_complete(another_test_c())
    loop.close()

    # # 2
    # import asyncio
    # # @asyncio.coroutine
    # def compute(x, y):
    #     print("Compute %s + %s ..." % (x, y))
    #     # yield x + y
    #     yield from asyncio.sleep(1.0)
    #     print("compute")
    #     return x + y
    #
    # @asyncio.coroutine
    # def print_sum(x, y):
    #     # yield x + y
    #     result = yield from compute(x, y)
    #     print("%s + %s = %s" % (x, y, result))
    #
    # loop = asyncio.get_event_loop()
    # print("start")
    # # 中断调用，直到协程执行结束
    # loop.run_until_complete(print_sum(1, 2))
    # print("end")
    # loop.close()

if __name__ == "__main__": 
    # generator_basics()
    # test_yield_from()
    test_coroutine_basic_idea()
    # test_coroutine()
    # test_asyncio()
