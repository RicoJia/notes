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
