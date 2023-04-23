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

if __name__ == '__main__':
    test_clip()
    test_prange()
    test_cython_threads_with_cpp()