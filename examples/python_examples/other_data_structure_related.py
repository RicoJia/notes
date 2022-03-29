def test_str():
    string = "123"
    # see abc1abc2abc3
    print(string.join("abc"))

def test_queue(): 
    import queue
    q = queue.Queue()
    q.put(5)
    q.put(10)
    # look at the last queue
    print(q.queue[-1])
    # get the element
    q.get()

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

if __name__=="__main__":
    # test_str()
    # test_div()
    test_queue()
