#!/usr/bin/python3
import numpy as np
#========================================================================
def test_zip(): 
    """
    1. Note zip creates an iterator (so it can be iterated only once), which returns a tuple. 
        - stops at the smallest sequence
    2. zip(ls1, ls2, ls3) you can zip multiple files
    """
    di = {1:"a", 2:"b"}
    foo = zip(di.values(), di.keys())
    for i in foo: 
        print(i)
    print("not going to print anything afterwards")
    for i in foo: 
        print(i)

    ls1 = [1,2]
    ls2 = [-1,-2, -3]
    ls3 = [1,-2, -3, 4]
    for i in zip(ls1, ls2, ls3): 
        print(i)

def deep_copy():
    di = {1:"a", 2:"b"}
    di2 = di.copy()
    di3 = di
    di[1] = "ccc"
    print(di, di2, di3)

    ls1 = ["a", "b"]
    ls2 = ls1.copy()
    ls1[0] = "ccc"
    # Not all objects have copy. use deep copy
    import copy
    ls3 = copy.deepcopy(ls1)
    print(ls1, ls2, ls3)

    # 'Pickling an AuthenticationString object is '
    # TypeError: Pickling an AuthenticationString object is disallowed for security reasons
    # That means you may have something that has unpicklable string, like a process object

def test_unpack():
    """
    1. Unpack works with all iterables: iterator, tuple, string, generator.
    2. make sure variable name isn't being used anywhere else.
    3. Star unpacking: just no more than 1 *. Good for known patterns
        - Everything comes out * is in a list.
        - use _ to ignore, or ign
    """
    a,b,c,d = "1234"
    print(a)
    # Too many things to unpack
    try:
        a,b,c = "1,2,3,4"
    # Use Exception can ignore KeyboardInterrupt, which otherwise get caught by except
    except Exception as e:
        print(e)

    # solution: star unpacking.
    a, *rest = "1234"
    print(rest)    #see ['2', '3', '4']
    # find average of nums from the middle of a list
    first, *middle, last = [100, 90, 80,80, 99]
    print(sum(middle)/len(middle))
    # Design pattern, can be used as tag:
    records = [("foo", 1,2), ("bar", "hello")]
    for tag, *args in records:
        if tag == "foo":
            print(args)

    # Also, need star expression to unpack args for func
    def sum_(a, b):
        return a + b
    ls = [1,2]
    # This wouldn't work to pass a list into the func
    # print(sum_(ls))
    print(sum_(*ls))

    line = 'nobody:*:-2:-2:Unprivileged User:/var/empty:/usr/bin/false'
    uname, * fields, homedir, ign = line.split(':')
    print(homedir)

def iterator_basics(): 
    """
    1. Iterable is something you can iterate over, like a list, dictionary, using the iterator inside them. 
        - Called Iterable protocol
    2. you must have ```__iter__``` and ```__next__``` for iterators
        - Use next() on iterables
        - Iterator is an object with next(). Use ```iter()``` to get an iterator. 
        - Once we have reached the bottom of an iterator, raises StopIteration 
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

    #create an iterable from dictionary
    di =  {"one": 1, "two":2}
    dict_iterator = iter(di)
    print(next(dict_iterator))

def generator_basics(): 
    """
    1. Use yield, which is like return, but returns a generator object, which can be iterated only once. Generators are iterators
        - Do not store all values in memory at once, generated on the fly
        - It's a generator function, which has __next__(), like iterator. But it could be easier
    2. By default, it raises StopIteration exception
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

def test_permutations():
    """
    1. permutations: all possible permutation
    2. Of smaller length
    """
    items = [4,3,2,1]
    from itertools import permutations
    for p in permutations(items): 
        print(p)
    # 2
    for p in permutations(items, 2): 
        print(p)

#========================================================================
def test_tuples():
    """
    1. tuples are faster and smaller than list
        - can print type(tup) 
    2. del tuple[1] won't work
    3. ==, < do work, by position 
    4. Quirk about tuple: SINGLE element, we have to append ',' but no need in other cases (e.g, no elements)
    """
    tup = (1,2,3)
    print(type(tup), tup[1])
    # tuple has no deletion
    try: 
        del tup[1]
    except: 
        print("no delete for tuple")

    print((1,2,3) == (2,3,4))
    print((1,2,3) < (2,3,4))
    
    # quirk
    #tuple = (1) #error
    tuple = (1, )

def test_named_tuples(): 
    """
    1. Still a tuple, can be unpacked, but instead of [1], you use field name
    2. Can use _replace() to make a new namedtuple with new fields
    3. Note: if you define the namedtuple inside function, pickle cannot find it
        - https://stackoverflow.com/questions/16377215/how-to-pickle-a-namedtuple-instance-correctly
        - multiprocessing.Queue actually uses this...
    """
    from collections import namedtuple
    Subscriber = namedtuple("some_name", ["addr", "name"])
    sub = Subscriber("123 st", "Jo")
    sub_new = sub._replace(addr="456st")
    print("field: ", sub_new.addr) 
    print(sub, sub_new)
    print("converted to dictionary: ", sub_new._asdict())

# vidb, ddd, debugger for python 
def dict_operations():
    """
    1. dict to list - need to convert items, values (values-view object) to list explicitly
        - filter
    2. Sort a dictionary based on key
        1. Use operator.itemgetter is a bit faster. itemgetter is a callable that calls __getitem__ 
        1. you can do sorted(dic, key = lambda k : dic[k])
    3. max, min can use itemgetter as well
    """
    # 1
    di = {1:"a", 2:"b"}
    ls = list(di.items())
    print(list(filter(lambda x: x[0] < 5, ls)))
    # dict_items() type, not supporting filter() directly on dict.items(). Must use list
    print(type(di.items()))
    print(di.values())

    # 2 
    dic = [ {'fname': 'Brian', 'lname': 'Jones', 'uid': 1003}, {'fname': 'David', 'lname': 'Beazley', 'uid': 1002}, {'fname': 'John', 'lname': 'Cleese', 'uid': 1001}, {'fname': 'Big', 'lname': 'Jones', 'uid': 1004}]
    print("sort dictionary by single key: ", sorted(dic, key = lambda k : k["lname"]))
    print("sort dictionary by two keys: ", sorted(dic, key = lambda k : (k['lname'], k['fname']))) 

    #3 
    from operator import itemgetter
    print("sort dictionary by single key, itemgetter: ", sorted(dic, key = itemgetter("lname")))
    print("sort dictionary by double key, itemgetter: ", sorted(dic, key = itemgetter("lname", "fname")))

def dictionary_basics(): 
    """
    1. if no value is found, get() will return default value (like None), dict[key]will raise an error
    2. delete an element 
        - pop doesn't throw errors
    3. when you do list(dict), iter(dict), they operate on keys. 
        - next(iter(dict)). need ```iter``` because dict is not an iterable
        - my_dict.values() gives you an "value-view" object, not a list. Similarly, my_dict.items(), .keys() gives you "item-view" keys-view objects
    4. merging 2 dicts
    5. Sort dictionary by value (ascending order) and return items in a list
        - items() seems like a tuple object, that's why you can use x[1] for accessing them
    """

    # 1
    person = {}
    person.get("key")   #get None
    person.get("key", "hehe")   #get hehe, default value
    # get KeyError
    # person["key"]

    # 2
    my_dict = {"lol": 1, "aaa": 2}
    del my_dict["lol"]
    print(my_dict)
    my_dict.pop("aaa")
    print(my_dict)

    # 3
    my_dict = {1:2, 3:4}
    print(list(my_dict))
    print(next(iter(my_dict))) 
    # And we need list() to convert dict_values object. This is a must
    print(list(my_dict.values()))

    # 4
    my_dict2 = {"1": "pen", "2": "hug"}
    print({**my_dict, **my_dict2})

    # 5
    dic = {1:100, 4: 50, 2:3}
    sorted_items = sorted(dic.items(), key=lambda x:x[1])
    print("descending: ", sorted_items)
    sorted_items = sorted(dic.items(), key=lambda x:x[0])
    print("descending by key: ", sorted_items)
    sorted_items = sorted(dic.items())
    print("descending by key, default key: ", sorted_items)
    sorted_items = sorted(dic.items(), reversed=True, key=lambda x:x[1])
    print("ascending: ", sorted_items)

def test_dict_less_known_features(): 
    """
    1. using np array as a key in dictionary, have to use to_bytes
    2. Find min, max of keys, or values:    
        - zip(keys(), values()), zip(values(), keys()). 
        - just find the min key or min value. 
        - just return the value of the min key. 
    3. Finding commonalities bw two dicts: items-view, keys-view objects support set operations, but not values-view objects pq values can have duplicates. 
        - Make a new dict with certain elements removed 
    """
    # 1
    my_array = np.array([1,2,3])
    my_dict = {}
    my_dict[my_array.tobytes()] = None

    # 2
    my_dict = {"1": 100, "2": 99}
    print("with max value: ", max(zip(my_dict.values(), my_dict.keys())))
    print("with max key", max(zip(my_dict.keys(), my_dict.values())))
    print("min value: ", min(my_dict.values())) 
    print("key of the min value: ", min(my_dict, key = lambda k:my_dict[k]))

    # 3 
    my_dict = {"1": 100, "2": 99}
    my_dict_2 = {"1": 100, "2": 99, "3": 98}
    print("common keys: ", my_dict.keys() & my_dict_2.keys())
    print("other keys: ", my_dict_2.keys() - my_dict.keys())
    print("common items: ", my_dict.items() & my_dict_2.items())
    
    # 4 
    my_dict_2 = {"1": 100, "2": 99, "3": 98}
    c = {key: my_dict_2[key] for key in my_dict_2.keys() - {"1"}}
    print("certain key removed: ", c)

def test_ordereddict():
    """
    1. Insertion order is preserved when you do next(iter), or for
    2. Implementation: using doubly linked list. and once a new item is inserted, it's inserted at the back. 
        - So it's twice the size as a regular dict
    """
    from collections import OrderedDict
    d = OrderedDict()
    d[5] = 6
    d[3] = 4
    d[1] = 2
    for key, val in d.items(): 
        print(key, val)

def test_default_dict(): 
    """
    1. Default dict is a subclass of dict. Can create default values. 
    2. Can create default list 
    3. Another motivation is to avoid keyError like dict[NOT_EXSISTENT]. 
        - We pass in a function as "default factory" to provide a default value if a key doesn't exist
    4. Iterate over default dict as you would with other iterables
    5. group items together, by using first sorting, then operation.groupby; but this TODO
    """
    # 1
    from collections import defaultdict
    # defaultdict(<class 'int'>, {0: 100, 1: 2})
    d1 = defaultdict(int)
    d1[0]+=100
    d1[1] += 2
    print(d1)

    # 2
    d2 = defaultdict(list)
    d2[0].append(3)
    d2[0].append(4)
    print(d2)

    # 3
    d3 = defaultdict(lambda: "not exist")
    i = d3[100]

    # 4 
    for key, val in d3: 
        print(key)
    
def test_chain_map(): 
    """
    1. ChainMap keeps a list of keys of multiple dictionaries, and can behave as one. Changes on each dict -> ChainMap; changes on ChainMap -> first dict
        - value from the first dictionary will be returned, if there're repeating keys
        - alternative: update, but that creates a totally new dict
    2. You can add ChainMap, which is useful for variables of different scope. 
        - or return a new chainmap (without the first one) for searching. 
    """
    from collections import ChainMap
    a = {'x': 1, 'z': 3 } 
    b = {'y': 2, 'z': 4 }
    c = {'z': 5, 'a': 4 }
    # 1
    chain = ChainMap(a,b,c)
    # ChainMap({'x': 1, 'z': 3}, {'y': 2, 'z': 4}, {'z': 5, 'a': 4})
    print("chainmap: ", chain)
    print("from first dic, z: ", chain["z"])
    chain["z"] = 100
    print("change value in chainmap: ", chain)
    b["z"] = 7
    print("But you can change values in the following dicts as well: ", chain)
    
    # 2
    print("adding an empty dict to chainmap: ", chain.new_child())
    print("new chainmap without the first dict: ", chain.parents)

def test_chain(): 
    """
    1. chain(iterable1, iterable2 ...) will make sa list of references to each element in the iterables. 
        - Everytime you have need complex iter tools, come to itertools
        - GREAT thing about chain: the iterables DO NOT HAVE TO BE of the same type.
    """
    from itertools import chain
    ls = [1,2,3]
    s = {4,5,6}
    for i in chain(ls, s): 
        print(i)

def set_funcs(): 
    """
    1. Basics: add, remove; intersection (&), union(|), rest (-)
        - discard will not raise an error, pop will
        - pop() popping a random value
        - does not support +=
    2. Set comprehension
    3. Can be used to remove duplicates in a Hashable function
    4. frozenset
        1. does not support indexing, 
        2. but can be used to unpack
    """
    # 1
    s = {1,2,3}
    s.add(4)
    print("s add: ", s)
    s.remove(2)
    # discard will not raise an error
    s.discard(2)
    print("s remove and discard", s)
    s2 = {2,3,4}
    print("s2 n s", s2.intersection(s))
    print("s2 n s", s2 & s)
    # shows S2 - (s2 & s)
    print("s2 - s", s2 - s)

    # 2 "set" comprehension
    ls = [1,2,3]
    s4 = set(i * 2 for i in ls)
    print(s4)
    # removes a random value in set 
    x = s4.pop()
    print("after pop ", s4)

    # 4will see TypeError: unhashable type: 'set'
    # because this is set is a mutable, so once it's changed, its hash has to change
    s2.add(frozenset({"12", "32"}))
    print(f"union: {s | s2}")
    # frozen set does NOT support indexing. This is how you retrive elements: 
    s3 = frozenset({"a", "b"})
    str1, str2 = s3
    print(str1, " ", str2)

def test_range(): 
    # you can access range object like list 
    r = range(1, 5)
    print(type(r), r[2])

    # create a set using range
    s = set(range(2))

def list_basics(): 
    """
    1. None in list works with if statement
    2. sorted will sort the list in place
    3. ls.reversed() will also does the job in place
    4. Create object with identical elements
        - see 'aaaa'. * in python will create APPEND the object with 1. Identical elements for immutables, and references for mutables
        - so USE LIST  comprehension if we want to have real duplicates
    5. list.extend() adds an iterable's values to the list
        - extend can work with non-local variables, while += can only work with locals
        - append is used to add another element in. +=, extend doesn't do those
    6. slice is a great object to hold list indices, which is to be used over and over
    """
    # 1
    ls = [1, None]
    if None in ls:
        print("lol")
    # 2
    ls = [(1,2), (2,-90), (3, 90)]
    sorted(ls)  # see [(1, 2), (2, -90), (3, 90)], always compares the first element!

    # 3
    ls = [1,2,3]
    ls.reversed()
    print(ls)

    # 4
    str_ = 'a'*4
    _str = 'a'*4
    print(str_)
    # * is no good in this case
    ls = [2 * [0]] * 3
    ls[0][0] = 1
    ls[1][1] = 100
    print(ls)
    # so USE LIST comprehension
    ls = [[0 for j in range (3)] for i in range(2)]
    ls[0][0] = 1
    print(ls)

    # 5
    list_1 = [1,2,3]
    list_2 = [4,5,6]
    list_1.extend(list_2)   # list_2 is an iterable,
    # two lists being merged to one
    print("extended list: ", list_1)
    list_1 = [1,2,3]
    list_1+=list_2
    print("+= list: ", list_1)
    list_1.append(3)
    print("append list: ", list_1)

    # 5.1
    l = [1,2,3]
    def foo(): 
        l.extend([4])
    def bar(): 
        l += [4]
    foo()
    # bar() # will fail: += can only work with locals
    print(l)

    # 6
    items = [0, 1, 2, 3, 4, 5, 6]
    # (start, stop, step), step is optional
    a_slice = slice(1, 4, 2)
    b_slice = slice(1,3)
    print("alisce: ", items[a_slice])
    print("blice: ", items[b_slice])

def useful_list_features(): 
    """
    1. compress can be used to filter out list items 
        - returns an iterable
    """
    from itertools import compress
    addresses = [ '5412 N CLARK', '5148 N CLARK', '5800 E 58TH', '2122 N CLARK' '5645 N RAVENSWOOD', '1060 W ADDISON', '4801 N BROADWAY', '1039 W GRANVILLE',]
    counts = [ 0, 3, 10, 4, 1, 7, 6, 1]
    more_than_5 = [c>5 for c in counts]
    print(list(compress(addresses, more_than_5)))

if __name__ == "__main__": 
    # set_funcs()
    # deep_copy()
    # iterator_basics()
    # dict_operations()
    # dictionary_basics()
    # test_ordereddict()
    # test_dict_less_known_features()
    # list_basics()
    # test_tuples()
    # test_named_tuples()
    # test_default_dict()
    # test_chain_map()
    # test_range()
    # test_unpack()
    # generator_basics()
    # generateor_uses()
    # test_zip()
    # useful_list_features()
    # test_reversed()
    # test_iterable_extra_state()
    # test_slice_iterator()
    # test_permutations()
    test_chain()
