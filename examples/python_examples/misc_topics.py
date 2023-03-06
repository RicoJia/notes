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

    # 4 If you want to reformat time, datetime.datetime.strptime() is the way to go.
    import datetime
    date_string = "21 June, 2018"
    # in datetime, lower case means number, upper case means alphabetitcal
    date_obj = datetime.datetime.strptime(date_string, "%d %B, %Y")
    print(date_obj)

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
    # shows 4
    print(walrut := 4)

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

def test_bytecode():
    import dis
    def func():
        di = {1:1, 2:2}
        di_c = di.copy()
    dis.dis(func)

def test_argparse():
    # 1 reading raw args
    import sys
    # print("The first arg is the file itself", sys.argv[0], "second arg: ", sys.argv[1])

    # 2. But argparse will disable sys
    import argparse
    parser = argparse.ArgumentParser()
    # required is default false. -- is optional, without -- is positional
    parser.add_argument("--bool", action="store_false")
    parser.add_argument("nums", nargs=2)
    parser.add_argument("variable_nums", nargs='*')
    # by specifying type, this could fail
    parser.add_argument("--some_int", type=int)
    # + means one or more
    args = parser.parse_args()
    print("bool should have a default value", args.bool)
    print("nums should be the first 2 args", args.nums)
    print("variable_nums are the rest of the args", args.variable_nums)

def test_enum():
    # 1: unique 
    from enum import Enum, unique, IntEnum
    class Animal(Enum):
        dog=1
        cat=1
        horse=3

    print(f'dog: name: {Animal.dog.name}, value: {Animal.dog.value}, cat: {Animal.cat}, if they are equal: {Animal.dog == Animal.cat}')


    try: 
        @unique
        class AnimalUnque(Enum):
            dog=1
            cat=1
            horse=3
    except ValueError:
        print(f'So two enums can be the same, so one is another ones alias. Using Enum.unique can avoid that')

    # 2: IntEnum
    try: 
        class AnimalInt(IntEnum):
            dog = 'a'
    except ValueError:
        print("Int Enum advantage 1: prevents strings in enum values")
    
    class AnimalInt(IntEnum):
        dog = 1
        cat = 2 
    print(f"IntEnum advantage 2: AnimalInt.dog == 1: {AnimalInt.dog == 1}, because IntEnum is a subclass of int {isinstance(AnimalInt.dog, int)}"
          "whereas Animal.dog == 1: {Animal.dog == 1}.")

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
    # test_bytecode()

    # test_argparse()
    test_enum()
