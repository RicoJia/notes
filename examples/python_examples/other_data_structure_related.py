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
        - heapify - first element being the smallest
    2. nLargest, nSmallest. Uses heapq, but also, if N == 1, just get min(). if N -> len(list), will do sorting first
        - heapq.heappop(li) returns the smallest element
    3. NOT threadsafe. 
    4. Can work with tuple
    5. Bugs:
       - Error: truth value of array with more than one element error. [link](https://stackoverflow.com/questions/39504333/python-heapq-heappush-the-truth-value-of-an-array-with-more-than-one-element-is). This is because the np array will be compared if the first element is the same
    """
    import heapq
    li = [5, 7, 9, 1, 3]
    print(heapq.nlargest(3, li))
    print(heapq.nsmallest(3, li))

    # heapify - first element being the smallest
    heapq.heapify(li)
    print(li)
    heapq.heappush(li,4)
    # pop the smallest element
    print ("heappop: ", heapq.heappop(li))
    heapq.heappush(li,4)
    print(li)

    # bug - truth value of array 
    import numpy as np
    heap = []
    try: 
        heapq.heappush(heap, (-60, np.array([1,2])))
        heapq.heappush(heap, (-60, np.array([1,2])))
    except: 
        # you can still see the two arrays inserted into heap
        print(heap)
    # sln - Python has a counter
    from itertools import count
    tiebreaker = count()
    heap = []
    heapq.heappush(heap, (-60, next(tiebreaker), np.array([1,2])))
    heapq.heappush(heap, (-60, next(tiebreaker), np.array([1,2])))
    print("proper heap for numpy arrays: ", heap)

def test_priority_q(): 
    """
    1. PriorityQueue is implemented by using heapq. The motivation is pop(), push() is more "queue like"
        - Keeps the insertion order for two items with the same priorities
        - Since it's using heapq underneath, by default, priorities are sorted for the smallest
    2. Is THREAD_SAFE. So a heapq is faster and smaller
    """
    import heapq
    class PriorityQueue:
        def __init__(self):
            self._queue = []
            self._index = 0
        def push(self, item, priority):
            # -priority is sorted for the smallest
            heapq.heappush(self._queue, (-priority, self._index, item))
            self._index += 1
        def pop(self):
            return heapq.heappop(self._queue)[-1]
    q = PriorityQueue()
    q.push("foo", 1)
    q.push("bar", 2)
    q.push("grok", 1)
    print(q.pop())
    print(q.pop())
    print(q.pop())



if __name__=="__main__":
    # test_string()
    # test_queue()
    # test_deque()
    # test_heapq()
    test_priority_q()

