import cython_example
import array
import time
import numpy as np
import timeit
from functools import partial

def test_clip():
    arr_size = 100000
    arr = array.array('d', range(arr_size))

    # 10^-4s
    start = time.time()
    cython_example.clip(arr, 2, 3, arr)
    print(f"time: {time.time() - start}")

    # 0.014s
    start = time.time()
    arr = array.array('d', range(arr_size))
    for i, r in enumerate(arr):
        if r < 2:
            arr[i] = 2
        elif r > 3:
            arr[i] = 3
        else:
            arr[i] = arr[i]
    print(f"time: {time.time() - start}")

    arr = np.array(range(arr_size), dtype=float)
    cython_example.clip(arr, 2, 3, arr)

    # 3 * 10^-3s. Core of numpy is written in C
    start = time.time()
    arr = np.array(range(arr_size), dtype=float)
    np.clip(arr, 2, 3, arr)
    print(f"time: {time.time() - start}")

def test_prange():
    arr = array.array('d', range(10)) 
    cython_example.multiply(arr, 4)
    print(f'arr: {arr}')

def test_cython_threads_with_cpp():

    mat1 = np.eye(500).tolist()
    mat2 = np.eye(500).tolist()
    mult = partial(cython_example.matrix_mult, mat1, mat2) 
    num_runs = 10
    elapsed_time = timeit.timeit(mult, number = num_runs)
    print(f'matrix multiplication average time: {elapsed_time/num_runs}')
    
###################################################
## Simple Componentwise Testing
###################################################

def simple_test(x):
    y = 1
    for i in range(1, x+1):
        y *= i
    return y

def list_list_test(len, num):
    ls = [num for _ in range(len)]

def list_append_test(len, num):
    ls = [num for _ in range(len)]
    ls2 = []
    for _ in range(len):
        ls2 .append(num)
    # return ls

def bench_mark(python_func, cython_func, *args, **kwargs):
    num_runs = 20
    def run(func, func_name):
        bound_func = partial(func, *args, **kwargs)
        elapsed_time = timeit.timeit(bound_func, number=num_runs)
        print(f"{func_name.ljust(30)} took: {elapsed_time}")
    run(python_func, python_func.__name__+ "_python")
    run(cython_func, cython_func.__name__ +"_cython")

def dict_test(di: dict):
    key_indices = {}
    value_indices = {}
    id = 0
    for key, val in di.items():
        key_indices[key] = id
        value_indices[key] = id
    
if __name__ == '__main__':
    # test_clip()
    # test_prange()
    # test_cython_threads_with_cpp()
    bench_mark(simple_test, cython_example.simple_test, 10000)
    bench_mark(list_append_test, cython_example.vector_pushback_test, len=10000, num=99)
    bench_mark(list_list_test, cython_example.list_list_test, len=10000, num=99)
    bench_mark(list_list_test, cython_example.array_test, len=10000, num=99)
    bench_mark(dict_test, cython_example.map_string_test, {str(i): str(i+1) for i in range(10000)})
    bench_mark(dict_test, cython_example.map_int_test, {i: i+1 for i in range(10000)})
