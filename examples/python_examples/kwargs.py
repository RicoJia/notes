def test_args(*args): 
    print(args) # should see a tuple

def test_kwargs(**kwargs): 
    print(kwargs)   # should see a dictionary

test_kwargs(foo="bar", love=101)
test_args(101, "123")
 

# vidb, ddd, debugger for python 

