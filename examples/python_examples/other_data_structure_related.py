#!/usr/bin/python3
def test_string(): 
    """
    1. ljust returns a 20 char long str, with "O" padding char
    2. to split a string into a list of words, based on delim
    3. str.strip() only remove trailing/ leading spaces
    4. See if start with, end with 
    5. find substring start index
    6. Print number with certain digits f"{12.456:10.1f}"
    7. nota: no escribe ```f"{arr[""]}"```, "" no se lleva bien con formatted string
    """
    # 1
    txt = "banana"
    x = txt.ljust(20, "O")
    print(x)

    # 2
    str_ = "lol, lol"
    # see['lol', ' lol']
    print(str_.split(','))
    # in total len(str_) is 15, with * on the sides
    print(str_.center(15, "*"))

    # 3
    txt = "     banana     bananana     "
    x = txt.strip()
    print(x)

    # 4
    str_ = "turkiye"
    print(str_.endswith("e"))

    # 5
    print(str_.find("r"))

    string = "123"
    # see abc1abc2abc3
    print(string.join("abc"))

    bit_num = b'sdf'
    # NOTE: str(text_bytes) can't specify the encoding
    print(str(bit_num))
    print(bit_num.decode('utf-8'))

    print(f"{12.456:10.1f}")    # 10 will add blanks

def test_matching (): 
    """
    1. you can search for strings that starts with one of the following. But you need tuple
    2. fnmatch can match with wildcard. Case-sensitivity is the same as the operating system 
    3. fnmatchcase will match the exact case. 
    """
    # 1
    filenames = [ 'Makefile', 'foo.c', 'bar.py', 'spam.c', 'spam.h' ]
    for f in filenames: 
        if f.startswith(("M", "fo")): 
            print(f)

    # 2 
    from fnmatch import fnmatch, fnmatchcase
    print("fn match: ", fnmatch("foo.txt", "*txt"))
    print("fn match: ", fnmatch("foo.txt", "*TXT"))
    print("fn match: ", fnmatch("foo.TXT", "*txt")) #still false
    print("fn matchcase: ", fnmatchcase("foo.TXT", "*txt")) #still false


"""
How Regex Works 
    - it's a finite state machine. (abc) is a->b->c; a(ab)*b, ab and aab are different. So it's called NFA (non-deterministic finite automata). So you end up using graph search. The state machine is coded up by Regex compiler.
"""
def test_regex(): 
    """
    1. regex refreshers: 
        - "\s" is white space. "\s*" means any number of white spaces
    2. x = re.findall("ai", txt) gives list of all matches
    3. re.search() returns a Match object. start() gives the first occurence of the match
    4. split func
        - x = re.split("\s", txt)
        - str.split() does not support regex. But str.split() already strips huge white spaces away
        - re.split() good for spliting multiple delimeters
    5. \d+ means one or more digits
    6. [] means charactger class, chars in this class can be matched to a single char. ()
    7. str.replace() and re.sub
    8. re.compile() can generate code for the state machine, great for reusing over and over. 
    """
    import re
    txt = "The rain in    Spain"
    x = re.findall("ai", txt)
    print(x)
    print(re.search("\s", txt).start())

    # you will see multiple " " 
    print(re.split("\s", txt))
    # but with txt.split(), you don't see the extra " "
    print(txt.split())

    line = 'asdf fjdk; afed, fjek,asdf,      foo'
    print(re.split(r"[;,\s]", line))
    print(re.split(r'[;,\s]\s*', line))

    str1 = "11/12/2020"
    print("\d+ test:", re.match(r"\d+/\d+/\d+", str1))

    str2=str1.replace("11", "haha")
    print(str2)
    # 3- means capture group
    str3=re.sub(r"(\d+)/(\d+)/(\d+)", r"\3-\2-\1", str1)
    print("using regex: ", str3)
    
    # 8
    pattern = re.compile(r"(\d+)/(\d+)/(\d+)")
    str3 = pattern.sub(r"\1-\2-\3", str1)
    print("using regex pattern: ", str3)


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
        - use nsmallest(key=...) to find the smallest items
    3. NOT threadsafe. 
    4. Can work with tuple
    5. Bugs:
       - Error: truth value of array with more than one element error. [link](https://stackoverflow.com/questions/39504333/python-heapq-heappush-the-truth-value-of-an-array-with-more-than-one-element-is). This is because the np array will be compared if the first element is the same
    """
    import heapq
    li = [5, 7, 9, 1, 3]
    print(heapq.nlargest(3, li))
    print(heapq.nsmallest(3, li))

    li2 = [  {'name': 'IBM', 'shares': 100, 'price': 91.1},

             {'name': 'AAPL', 'shares': 50, 'price': 543.22},

             {'name': 'FB', 'shares': 200, 'price': 21.09},

             {'name': 'HPQ', 'shares': 35, 'price': 31.75},

             {'name': 'YHOO', 'shares': 45, 'price': 16.35},

             {'name': 'ACME', 'shares': 75, 'price': 115.65}]
    print("heapq, find smallest item in a list of dict: ", heapq.nsmallest(2, li2, key= lambda k: k["price"]))

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

def test_type_cast():
    '''
    1. define __int__, __str__, __float__ for type conversion
    '''
    class Foo:
        def __int__(self):
            return 100
        def __str__(self) -> str:
            return "hello"
        def __float__(self) -> float:
            return 88.0

    f = Foo()
    print(int(f), float(f), str(f))

if __name__=="__main__":
    # test_string()
    test_type_cast()
    # test_matching()
    # test_queue()
    # test_deque()
    # test_heapq()
    # test_priority_q()
    # test_regex()
