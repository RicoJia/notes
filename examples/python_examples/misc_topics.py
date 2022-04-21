#!/usr/bin/python3
def test_math(): 
    import math
    print(math.log(2.7183))
    print(math.log(10,10))
    print(math.floor(0.4))

def test_div(): 
    a = -12
    b = 7
    # by default, we get float -1.7 here
    print("a/b ", a/b)
    # get 2, since for negative it's -2*7+2, this is remainder
    print("a%b: ", a%b)
    # get -2, since it's the quotient of the smallest closest num, floor division
    print("a//b: ", a//b)
    # see (-2, 2), the remainer
    print("", divmod(a, b))

    import math
    # fmod for negative number, find the closest larger number
    print(math.fmod(-10, 3))    #3*-3-1 = -10
    print(math.fmod(-5, 3))    #1*-3 - 2 = -5
    print(math.fmod(5, -3))    #-3*-1 + 2 = 5

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

def test_enum(): 
    from enum import Enum
    class Animal:
        DOG = 1
        CAT = 2
    ls = [1,2,3]
    print(ls[Animal.DOG]) 

if __name__=="__main__":
    # test_warning()
    # test_math()
    # test_tup()
    # test_enum()
    test_div()
