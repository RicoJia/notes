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
    print(my_dict)

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

if __name__ == "__main__": 
    # dict_to_list()
    # set_funcs()
    # deep_copy()
    # dictionary_basics()
    list_basics()
    # test_tuples()
