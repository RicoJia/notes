#!/usr/bin/python3
import numpy as np
def kwargs_test(): 
    def test_args(*args): 
        print(args) # should see a tuple

    class test_kwargs: 
        def __init__(self, **kwargs): 
            print(kwargs)   # should see a dictionary
            print(kwargs["foo"])

    test_kwargs(foo="bar", love=101)
    test_args(101, "123")
     

    # partial
    from functools import partial
    def func(a, b): 
        print(a, b)
    func_w = partial(func, b = 12)
    func_w(a = 13)

def test_tuples():
    tup = (1,2,3)
    print(type(tup), tup[1])
    # tuple has no deletion
    try: 
        del tup[1]
    except: 
        print("no delete for tuple")
    

# vidb, ddd, debugger for python 
def dict_to_list():
    di = {1:"a", 2:"b"}
    ls = list(di.items())
    print(list(filter(lambda x: x[0] < 5, ls)))
    # dict_items() type, not supporting filter() directly on dict.items(). Must use list
    print(type(di.items()))
    print(di.values())

def dictionary_basics(): 
    # using np array as a key in dictionary, have to use to_bytes
    my_array = np.array([1,2,3])
    my_dict = {}
    my_dict[my_array.tobytes()] = None

    # if no value is found, get() will return default value (like None), dict[key]will raise an error
    person = {}
    person.get("key")   #get None
    person.get("key", "hehe")   #get hehe, default value
    # get KeyError
    # person["key"]

    # delete an element
    my_dict["lol"] = "lol"
    del my_dict["lol"]
    my_dict["key1"] = "val1"
    my_dict["key2"] = "val2"
    print(my_dict)

    # we're printing the keys only
    print(list(my_dict))
    # And we need list() to convert dict_values object. This is a must
    print(list(my_dict.values()))

    # merging 2 dicts
    my_dict2 = {"1": "pen", "2": "hug"}
    print({**my_dict, **my_dict2})

    # pop doesn't throw errors
    my_dict2.pop("1")
    print(my_dict2)

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

def set_funcs(): 
    s = set()
    s = {1,2,3}
    s.add(4)
    print("s add: ", s)
    # remove will raise an error if not existing 
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

def list_basics(): 
    # # None in list
    # ls = [1, None]
    # if None in ls:
    #     print("lol")
    #
    # #sort
    # ls = [(1,2), (2,-90), (3, 90)]
    # sorted(ls)  # see [(1, 2), (2, -90), (3, 90)], always compares the first element!

    # has to be used this way
    ls = [1,2,3]
    ls.reverse()
    print(ls)

    #### Create object with identical elements
    # see 'aaaa'. * in python will create APPEND the object with 1. identical elements for immutables, and references for mutables
    str_ = 'a'*4
    _str = 'a'*4
    print(str_)

    # * is no good in this case
    ls = [2 * [0]] * 3
    ls[0][0] = 1
    ls[1][1] = 100
    print(ls)

    # so USE LIST  comprehension
    ls = [[0 for j in range (3)] for i in range(2)]
    ls[0][0] = 1
    print(ls)

    ls2 = [4,5,6]
    tmp_ls = ls[0] + ls2
    print(tmp_ls)

    # unpack a list. if not enough params, we will run into error. Also we can do this in u, v
    u, v, g= ls2
    print(f"u: {u}, v: {v}")
    ls3 = [[1,2], [3,4], [5,6]]
    for u, v in ls3: 
        print(u, v)

if __name__ == "__main__": 
    # dict_to_list()
    # set_funcs()
    # deep_copy()
    dictionary_basics()
    # list_basics()
    # test_tuples()
    # test_default_dict()
    # test_range()
