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


if __name__ == "__main__": 
    # 2
    import asyncio
    @asyncio.coroutine
    def compute(x, y):
        print("Compute %s + %s ..." % (x, y))
        # yield x + y
        yield from asyncio.sleep(1.0)
        print("compute")
        return x + y

    @asyncio.coroutine
    def print_sum(x, y):
        # yield x + y
        result = yield from compute(x, y)
        print("%s + %s = %s" % (x, y, result))

    loop = asyncio.get_event_loop()
    print("start")
    # 中断调用，直到协程执行结束
    loop.run_until_complete(print_sum(1, 2))
    print("end")
    loop.close()

