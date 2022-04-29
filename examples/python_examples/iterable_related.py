#!/usr/bin/python3
import numpy as np
#========================================================================
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
    2. you must have ```__iter__``` and ```__next__``` for iterators
        - Use next() on iterables
        - Use ```iter()``` to get an iterator
        - Use ```for i in ``` to loop over

    """
    class Bday:
        def __iter__(self):
            return self
        def __next__(self):
            return 100
            # raise StopIteration   #or do this
    # __iter__ returns an iterator
    bday = Bday()         
    # __next__ returns the next value of the iterable
    print(next(bday))

def test_iterator_on_iterable(): 
    """
    1. You have to get an iterator, Then you can do next()
    2. or ```for i in``` calls the iterator inside an iterable
    3. dict is an iterable. But iter(di) gives you the keys
    4. Once we have reached the bottom of an iterator, raises StopIteration 
    """
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
    1. Use yield, which is like return, but returns a generator object, which can be iterated only once. 
        - Do not store all values in memory at once, generated on the fly
    2. By default, it raises StopIteration exception
    3. So use for i in.... For loop returns a generator
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

def test_zip(): 
    """
    Note zip creates an iterator (so it can be iterated only once)
    """
    di = {1:"a", 2:"b"}
    foo = zip(di.values(), di.keys())
    for i in foo: 
        print(i)
    print("not going to print anything afterwards")
    for i in foo: 
        print(i)
    
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
    dic = {1:100, 2:3}
    sorted_items = sorted(dic.items(), key=lambda x:x[1])
    print("descending: ", sorted_items)
    sorted_items = sorted(dic.items(), reverse=True, key=lambda x:x[1]) 
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
    # default dict is a subclass of dict
    from collections import defaultdict
    # created default values as 0 
    # defaultdict(<class 'int'>, {0: 100, 1: 2})
    d1 = defaultdict(int)
    d1[0]+=100
    d1[1] += 2
    print(d1)

    # create default values being []
    d2 = defaultdict(list)
    d2[0].append(3)
    d2[0].append(4)
    print(d2)

    #another motivation is to avoid keyError like dict[NOT_EXSISTENT]. We pass in a function as "default factory" to provide a default value if a key doesn't exist
    d3 = defaultdict(lambda: "not exist")
    i = d3[100]

    # you can create a set, append to it, etc. 
    # iterate this as you would with dict 
    for key, val in d3: 
        print(key)
    
def set_funcs(): 
    """
    1. Can be used to remove duplicates in a Hashable function
    """
    s = {1,2,3}
    s.add(4)
    print("s add: ", s)
    # 
    s.remove(2)
    # discard will not raise an error
    s.discard(2)
    print("s remove and discard", s)

    s2 = {2,3,4}
    print("s2 n s", s2.intersection(s))
    print("s2 n s", s2 & s)
    # shows S2 - (s2 & s)
    print("s2 - s", s2 - s)

    # will see TypeError: unhashable type: 'set'
    # because this is set is a mutable, so once it's changed, its hash has to change
    # use frozenset
    # s2.add({"12", "32"})
    s2.add(frozenset({"12", "32"}))
    print(f"union: {s | s2}")

    # frozen set does NOT support indexing. This is how you retrive elements: 
    s3 = frozenset({"a", "b"})
    str1, str2 = s3
    print(str1, " ", str2)

    # += does not work on set
    # s2+=s

    # "set" comprehension
    ls = [1,2,3]
    s4 = set(i * 2 for i in ls)
    print(s4)

    # removes a random value in set 
    x = s4.pop()
    print("after pop ", s4)

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
    3. ls.reverse() will also does the job in place
    4. Create object with identical elements
        - see 'aaaa'. * in python will create APPEND the object with 1. Identical elements for immutables, and references for mutables
        - so USE LIST  comprehension if we want to have real duplicates
    5. list.extend() adds an iterable's values to the list
        - extend can work with non-local variables, while += can only work with locals
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
    ls.reverse()
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
    print("extended list: ", list_1)
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

if __name__ == "__main__": 
    # set_funcs()
    # deep_copy()
    dict_operations()
    # dictionary_basics()
    # test_ordereddict()
    # test_dict_less_known_features()
    # list_basics()
    # test_tuples()
    # test_default_dict()
    # test_range()
    # test_unpack()
    # test_iterator_on_iterable()
    # generator_basics()
    # test_zip()
