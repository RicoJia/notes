#!/usr/bin/python3
def test_is(): 
    """
    Is identifies if two objects are the same object, by checking their memory address
    == compares their values by checking their deep copies
    {}, [] are mutables, so you have to use ==
    (), "", '' are immutables, so use is
    """
    print("True is True", True is True)  #see True, because True is an object
    print("None is None", None is None)  # see True

    print("[] == []", [] == []) #because list is mutable, == should be used
    print("{} == {}", {} == {})      #because dict is mutable, == should be used.
    print("[] is []", [] is [])  # see false
    print("{} is {}", {} is {})      # see false
    print("'' == ''", '' == '')
    print("'' is ''", '' is '')      # string is immutable (once created, value never changes), refers to the same memory location
    print("() == ()", () == ())
    print("() is ()", () is ())
    # tuple is immutable as well

if __name__=="__main__":
    test_is()
