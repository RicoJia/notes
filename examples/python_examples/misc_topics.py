#!/usr/bin/python3
def test_math(): 
    import math
    print(math.log(2.7183))
    print(math.log(10,10))
    print(math.floor(0.4))

def test_warning():
    import warnings
    def fxn():
        warnings.warn("deprecated", DeprecationWarning)

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        fxn()

def test_reference(): 
    ls = [1,2,3]
    def try_to_modofy_list_element(element): 
        element = 1000
    # not modifying, since we're passing in alias to ls[0]. Variable names in Python are aliases to memory locations. Assigning an alias A to another will not modify the content of the previous A itself
    try_to_modofy_list_element(ls[0])

def test_tup(): 
    a, b = 1, 2
    print(a,b)

def test_string(): 
    #ljust returns a 20 char long str, with "O" padding char
    txt = "banana"
    x = txt.ljust(20, "O")
    print(x)

    # to split a string into a list of words, based on delim
    str_ = "lol, lol"
    # see['lol', ' lol']
    print(str_.split(','))
    print(str_.endswith("s"))
    # in total len(str_) is 15, with * on the sides
    print(str_.center(15, "*"))

    # str.strip() remove trailing/ leading spaces
    txt = "     banana     "
    x = txt.strip()
    print(x)

if __name__=="__main__":
    # test_warning()
    # test_math()
    # test_tup()
    test_string()
