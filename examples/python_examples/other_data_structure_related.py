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

    bit_num = b'sdf'
    # NOTE: str(text_bytes) can't specify the encoding
    print(str(bit_num))
    print(bit_num.decode('utf-8'))

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

def test_deque(): 
    """
    1. Natural choice for FIFO queue. pop, push O(1), while list is O(N)
    2. Uses: 
        1. if no length specified, the queue is unbounded
        2. popleft(): the left most element, pop(): the right most element
    """
    from collections import deque
    q = deque (maxlen = 3)
    q.append('a')
    q.append('b')
    q.append('c')
    q.append('d')
    print(q)    # see b c d
    print(q.popleft())  # see b, popleft(): the left most element
    print(q.pop())  # see d, pop(): the right most element

def test_heapq(): 
    """
    1. By default, returns the smallest element 
    2. nLargest, nSmallest. Uses heapq, but also, if N == 1, just get min(). if N -> len(list), will do sorting first
    """
    import heapq
    li = [5, 7, 9, 1, 3]
    print(heapq.nlargest(3, li))
    print(heapq.nsmallest(3, li))
    heapq.heapify(li)
    heapq.heappush(li,4)
    print (heapq.heappop(li))

if __name__=="__main__":
    # test_string()
    # test_queue()
    # test_deque()
    test_heapq()

