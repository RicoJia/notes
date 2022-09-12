#!/usr/bin/python3
def test_math(): 
    """
    1. math.log(2.7183)
    2. Use Decimal, decimal, change context
    3. math.fsum() for accurate sum
    4. bin(num), oct(num), hex(num)
    5. int("int_str", base) to convert an integer string to base-10 integer
    6. num.to_bytes, num.bit_length()
    7. int.from_bytes(byte_string)
    """
    import math
    print(math.log(2.7183))
    print(math.log(10,10))
    print(math.floor(0.4))

    # 2
    from decimal import Decimal, localcontext
    with localcontext () as ctx: 
        ctx.prec = 3
        a = Decimal(3)
        b = Decimal(5)
        print("3 digits: ", b/a, "while without context: ")

    # 3 
    ls = [1.2e+18, 1, -1.2e+18]
    print("sum of ls: ", sum(ls), ", while using math.fsum: ", math.fsum(ls))

    # 4
    print("binary num for 10 (1010): ", bin(10))
    print("Oct num for 10 (0o12): ", oct(10))
    print("hex num for 10 (0xa): ", hex(10))

    # 5 
    print("0x4d2 is 1234: ", int("4d2", 16))

    # 6
    x = 0x01020304
    print(x, "big endian, the normal order we see: ", x.to_bytes(4, "big"))
    print(x, "Little endian, the reverse order we see: ", x.to_bytes(4, "little"))
    print(x, "bit length: ", x.bit_length())
    quotient, rem = divmod(x.bit_length(), 8) 
    print("number of bytes: ", int(quotient) + int(rem >0))
    
    # 7
    data = b"\x00\x124V\x00x\x90\xab\x00\xcd\xef\x01\x00#\x004"
    print("number of bytes of byte string: ", len(data))
    print("from bytes, little", int.from_bytes(data, "little"))

def test_more_math(): 
    """
    1. math.isinf(), int/inf = 0; inf/inf=nan; inf-inf = nan
    2. math.isnan();    
        - NAN cannot be compared against.
        - nan + - / 1 = nan, literally anything
    """
    from math import isinf, isnan
    inf = float("inf")
    print("isinf: ", isinf(inf))
    nan = float("nan")
    print("isnan: ", isnan(nan))
    nan2 = float("nan")
    print("nan CANNOT be compared to nan (no errors tho): ", nan == nan2, "neither by using is: ", nan is nan2)

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

def test_random(): 
    """
    1. random is deterministic using Mersenne Twister. so specify seed.
    2. random functions work on lists directly.
    """
    import random
    import time
    random.seed(time.time())
    ls = [1,2,3,4,5,6]
    print("random.choice chooses 1 element: ", random.choice(ls))
    print("random.sample: ", random.sample(ls, 2))
    random.shuffle(ls)
    print("random.shuffle: ", ls)

def test_datetime(): 
    """
    1. simple manipulation of days & time. 
        - timedelta can specify hours, but just shows days, total_seconds, seconds. No hours.
        - Can create an object using datetime()
        - Is aware of leap years
    2. dateutil.relativedelta(months) as a supplement for months, and it works with datetime.datetime
        - a =- 4; =- is interpreted as 2 tokens. So a = -4
    3. Use str.split("-") rather than datetime.strip(), which was written in python and not fast.
    """
    # 1
    from datetime import timedelta
    a = timedelta(weeks=1, days=2, hours = 3)
    b = timedelta(weeks=1, days=2, hours = 3)
    print("sum of timedelta: ", a + b, ". # of days: ", (a+b).days, " # of seconds, on top of # of days: ", (a+b).seconds, " # of total seconds: ", (a+b).total_seconds())
    
    # 2
    from datetime import datetime
    a = datetime(2020, 2, 28)
    b = datetime(2020, 3, 1)
    print("a-b in 2020: ", a-b) # see -2 days
    a = datetime(2021, 2, 28)
    b = datetime(2021, 3, 1)
    print("a-b in 2021: ", a-b) # see -1 days

    # 3
    from dateutil.relativedelta import relativedelta
    print("relative delta: ", a + relativedelta(months = 2))

    text = '2012-09-20'
    yr, mon, day = text.split("-")
    print("yr, mon, day: ", int(yr), int(mon), int(day))

def test_fractions(): 
    """
    1. fractions.Fraction() can allow: fraction, and closest approximate 
    """
    from fractions import Fraction
    a = Fraction(35)
    b = Fraction(64)
    c = a/b
    print("a/b: ", c, "numerator: ", c.numerator, ", to float: ", float(c))
    print("a float number to fraction: ", Fraction(0.4), ", you see a crazy fraction, right")
    print("Now it should be a lot better: ", Fraction(0.4).limit_denominator(10))


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

def test_or(): 
    """
    1. Or actually returns the first input when it's 'truthy' or you've reached the end. And this is because it Works with "short-circuit", i.e., keep searching until it finds a true
        - So if "sdf" is the first "truthy" value, then it will return it
    2. and - returns the first "falsy" input
    3. great for handling corner cases, where you might have None, or empty list.
    """
    name = None
    name2 = "sdf"
    print(name or name2)

def test_enum(): 
    from enum import Enum
    class Animal:
        DOG = 1
        CAT = 2
    ls = [1,2,3]
    print(ls[Animal.DOG]) 

def test_lru_cache_optimization():
    """
    1. LRU (Least-recently-used cache) caches the input and output of function calls (memoization)
    """
    from functools import lru_cache
    import time
      
    # Function that computes Fibonacci 
    # numbers without lru_cache
    def fib_without_cache(n):
        if n < 2:
            return n
        return fib_without_cache(n-1) + fib_without_cache(n-2)
          
    # Execution start time
    begin = time.time()
    fib_without_cache(30)
      
    # Execution end time
    end = time.time()
      
    print("Time taken to execute the\
    function without lru_cache is", end-begin)
      
    # Function that computes Fibonacci
    # numbers with lru_cache
    @lru_cache(maxsize = 128)
    def fib_with_cache(n):
        if n < 2:
            return n
        return fib_with_cache(n-1) + fib_with_cache(n-2)
          
    begin = time.time()
    fib_with_cache(30)
    end = time.time()
      
    print("Time taken to execute the \
    function with lru_cache is", end-begin)

def test_memory_leak():
    import tracemalloc
    import numpy as np
    tracemalloc.start()

    length=10000
    test_array=np.random.randn(length) # 分配一个定长随机数组
    snapshot=tracemalloc.take_snapshot() # 内存摄像
    top_stats=snapshot.statistics('lineno') # 内存占用数据获取

    print ('[Top 10]')
    for stat in top_stats[:20]: # 打印占用内存最大的10个子进程
        print (stat)
     
def test_walrus_operator():
    """
    Assign param to a value. Do not work with (), [] in list(), dict[]
    """
    walrut = 3
    # shows 4, python 3.8+
    # print(walrut := 4)

def test_color_printing():
    """
    Ref: https://gist.github.com/rene-d/9e584a7dd2935d0f461904b9f2950007
    """
    RED = "\033[0;31m"
    GREEN = "\033[0;32m"
    GOLD = "\033[0;33m"
    BOLD = "\033[1m"
    print(f"{RED} This is red, {GREEN} this is green, {GOLD}{BOLD} this is bold GOLD")

def test_strip():
    """
    1. take out the outermost characters in strip(str)
    """
    txt = ",,,,,rrttgg.....banana....rrr" 
    x = txt.strip(",.r")
    print(x)

def test_pathlib2():
    """
    1. pathlib2 is nice for managing parent, etc.
    """
    from pathlib2 import Path
    import os
    print(Path.cwd().parent.parent)
    slab_path = Path.cwd() / "slab"
    print(slab_path)
    str(slab_path)

def test_uuid():
    import uuid 
    print(uuid.uuid4())

def small_tricks():
    """
    1. [1,4,2] ->[(0, 1), (1, 4), (2, 2)]
    2. "hEllo_woRLD" -> "Hello_World"
    """
    ls = [1,4,2]
    print(list(enumerate(ls)))

    st = "hEllo_woRLD"
    print(str.title(st))

def tab_complete():
    import readline
    import logging
    LOG_FILENAME = '/tmp/completer.log'
    logging.basicConfig(filename=LOG_FILENAME,
                        level=logging.DEBUG,
                        )
    class SimpleCompleter(object):
        def __init__(self, options):
            self.options = sorted(options)
            return
        def complete(self, text, state):
            # state: number of times this function has been calledwith the same text, i.e., you just hit tab key
            response = None
            if state == 0:
                # This is the first time for this text, so build a match list of the current keyword
                if text:
                    self.matches = [s
                                    for s in self.options
                                    if s and s.startswith(text)]
                    logging.debug('%s matches: %s', repr(text), self.matches)
                else:
                    self.matches = self.options[:]
                    logging.debug('(empty input) matches: %s', self.matches)

            # Return the state'th item from the match list,
            # if we have that many.
            # helpful debug messages:
            # print("=============")
            # print(f"text: {text}, state: {state}")
            # print(f"matches: {self.matches}")
            # if you return a non-None object, the system will keep calling this function until it returns None
            try:
                response = self.matches[state]
                # print("reponse: ", response)
            except IndexError:
                response = None

            return response

    def input_loop():
        line = ''
        while line != 'stop':
            line = input('Prompt ("stop" to quit): ')
            print ('Dispatch %s' % line)

    # Register our completer function
    readline.set_completer(SimpleCompleter(['start', 'stop', 'list', 'print']).complete)

    # Use the tab key for completion
    readline.parse_and_bind('tab: complete')

    # Prompt the user for text
    input_loop()

def line_buffer_tab_complete():
    import readline
    import logging

    LOG_FILENAME = '/tmp/completer.log'
    logging.basicConfig(filename=LOG_FILENAME,
                        level=logging.DEBUG,
                        )

    class BufferAwareCompleter(object):
        
        def __init__(self, options):
            self.options = options
            self.current_candidates = []
            return

        def complete(self, text, state):
            response = None
            if state == 0:
                # This is the first time for this text, so build a match list.
                
                origline = readline.get_line_buffer()
                begin = readline.get_begidx()
                end = readline.get_endidx()
                being_completed = origline[begin:end]
                words = origline.split()

                print('origline', repr(origline))
                print('begin', begin)
                print('end', end)
                print('being_completed', being_completed)
                print('words', words)
                
                if not words:
                    self.current_candidates = sorted(self.options.keys())
                else:
                    try:
                        if begin == 0:
                            # first word
                            candidates = self.options.keys()
                        else:
                            # later word
                            first = words[0]
                            candidates = self.options[first]
                        
                        if being_completed:
                            # match options with portion of input
                            # being completed
                            self.current_candidates = [ w for w in candidates
                                                        if w.startswith(being_completed) ]
                        else:
                            # matching empty string so use all candidates
                            self.current_candidates = candidates

                        logging.debug('candidates=%s', self.current_candidates)
                        
                    except (KeyError, IndexError) as err:
                        logging.error('completion error: %s', err)
                        self.current_candidates = []
            
            try:
                response = self.current_candidates[state]
            except IndexError:
                response = None
            logging.debug('complete(%s, %s) => %s', repr(text), state, response)
            return response
                

    def input_loop():
        line = ''
        while line != 'stop':
            line = input('Prompt ("stop" to quit): ')
            print ('Dispatch %s' % line)

    # Register our completer function
    readline.set_completer(BufferAwareCompleter(
        {'list':['files', 'directories'],
         'print':['byname', 'bysize'],
         'stop':[],
        }).complete)

    # Use the tab key for completion
    readline.parse_and_bind('tab: complete')

    # Prompt the user for text
    input_loop()
if __name__=="__main__":
    # test_warning()
    # test_math()
    # test_more_math()
    # test_fractions()
    # test_random()
    # test_datetime()
    # test_or()
    # test_enum()
    # test_div()
    # test_lru_cache_optimization()
    # test_memory_leak()
    # test_walrus_operator()
    # test_uuid()
    # test_color_printing()

    # small_tricks()
    # tab_complete()
    line_buffer_tab_complete()
