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


# vidb, ddd, debugger for python 

def np_filter(): 
    arr = np.ones((4,3))
    arr[0,:] *= np.NINF

    not_inf = ~np.isinf(arr)   #matrix shows true for elements that're not inf
    a = not_inf.any(axis=1)     #examine each row? False if any element in the row is false
    arr[[True,False, False, False]] # shows first row. Different than arr[[1,0,0,0]]?
    filtered = arr[a]       # the last result we want


def dict_to_list():
    di = {1:"a", 2:"b"}
    ls = list(di.items())
    print(list(filter(lambda x: x[0] < 5, ls)))
    # dict_items() type, not supporting filter() directly on dict.items(). Must use list
    print(type(di.items()))
    print(di.values())

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

if __name__ == "__main__": 
    dict_to_list()
    # set_funcs()
