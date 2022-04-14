#!/usr/bin/python3
def test_string(): 
    #ljust returns a 20 char long str, with "O" padding char
    txt = "banana"
    x = txt.ljust(20, "O")
    print(x)

    # to split a string into a list of words, based on delim
    str_ = "lol, lol"
    # see['lol', ' lol']
    print(str_.split(','))
    # in total len(str_) is 15, with * on the sides
    print(str_.center(15, "*"))

    # str.strip() remove trailing/ leading spaces
    txt = "     banana     "
    x = txt.strip()
    print(x)

    # See if start with, end with 
    str_ = "turkiye"
    print(str_.endswith("e"))

    # find substring start index
    print(str_.find("r"))

    string = "123"
    # see abc1abc2abc3
    print(string.join("abc"))

def test_queue(): 
    import queue
    q = queue.Queue(2)
    q.put(5)
    q.put(10)
    # look at the last queue
    print(q.queue[-1])
    print(q.full())
    # get the element
    q.get()

if __name__=="__main__":
    test_string()
    test_queue()
