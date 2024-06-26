cimport cython
cimport numpy as cnp
from cython.parallel import prange

from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.map cimport map

"""
- Motivation
    - A handcrafted C program can be slower than the cython program (because of optimization, etc.)
    - In Cython, because of explicit typing, code that's almost identical to python can still achieve
    5x speedups

- cdef,etc: 
    - cdef are functions can be called in c/c++, but not python
    def is something that can be called in python
    cpdef can be called in C, C++, Python
    - for def and cpdef, objects are passed by reference

- PyObject
    - int in python is a PyObject
    - Using python Memoryview obj, which is a flexible way to access buffers, such as c array, numpy array
        - Can even work with numpy array, since it follows memoryview protocol
        - In your function, you should have an "out" function as a convention: so it relies on 
        the caller to create that for you
        - np.zeros() can do the trick

- optimizations: (2.5x faster)
    - boundscheck: whether array goes out of bounds
    - wraparound: no negative indices 
    - If a function has python dict objects, then cython does not provide too big of a speed up. However, it still has certain function calls that are faster
    - Cython can be converted into C callables.

- Pxd files: we include declarations from c/c++ modules. so other cython can link to them
    # math.pxd, 
    cdef extern from "math.h":
        double sin(double x)
"""
@cython.boundscheck(False)
@cython.wraparound(False)
cpdef clip(double[:] a, double min, double max, double[:] out):
    if a.shape[0] != out.shape[0]:
        raise ValueError("input output arrays must have the same shape")
    for i in range(a.shape[0]):
        if a[i] < min:
            out[i] = min
        elif a[i] > max:
            out[i] = max
        else:
            out[i] = a[i]

        # an conditional expression is a if something else b (ternary expression)

def multiply(double[:] x, double alpha):
    """
    prange:
     - means parallel range, allows loops to be executed in parallel
     - nogil = True allows true parallelism (GIL released) 
     - Must have OpenMP (open multiprocessing) installed 
        - supported in C, C++ for scientific computing
     - OpenMP spawns a threadpool 
    """
    cdef Py_ssize_t i

    for i in prange(x.shape[0], nogil=True):
        x[i] = alpha * x[i]

### Matrix multiplication

ctypedef vector[vector[int]] Matrix

# what does nogil here do - so it's compatible with prange
cdef void multiply_matrices(const Matrix &A, const Matrix &B, Matrix &C, Py_ssize_t i) nogil:
    cdef Py_ssize_t j, k, n = A.size(), m = B[0].size()
    for j in range(m):
        C[i][j] = 0
        for k in range(n):
            C[i][j] += A[i][k] * B[k][j]


cpdef Matrix matrix_mult(Matrix A, Matrix B):
    cdef Py_ssize_t i, n = A.size(), m = B[0].size(), p = B.size()
    if A[0].size() != p:
        raise ValueError("Matrix dimensions do not match for multiplication.")

    cdef Matrix C = Matrix(n, vector[int](m, 0))

    # prange can only be used without GIL
    # Another quirk is You can have only 1 prange
    # We have not seen improvements compared to regular range in This implementation
    for i in prange(n, nogil=True, schedule='guided'):
    # for i in range(n):
        multiply_matrices(A, B, C, i)

    return C

# Test functions 
# Profiling Results: Cython is not necessarily faster than Python 3.8
# A python list with some type casting could be the way to go.

cdef class A:
    cdef public int x

cdef class Alist:
    def __init__(self):
        self.inner = []
    cdef list inner
    cdef void append(self, A a):
        self.inner.append(a)
    cdef A get(self, int i):
        # ?
        return <A> self.inner[i]
    def __len__(self):
        return len(self.inner)

cpdef Alist make_typed_list(int N):
    cdef A a
    cdef int i
    cdef Alist L = Alist()
    for i in range(N):
        a = A()
        a.x = 1
        L.append(a)
    return L

cpdef list make_python_list(int N):
    cdef A a
    cdef int i
    cdef list L = []
    for i in range(N):
        a = A()
        a.x = 1
        L.append(a)
    return L

cpdef long test_python_list(list L) except -1:
    cdef int i
    cdef long sum = 0
    for a in L:
        # explicit casting makes cython slightly faster
        sum += (<A>a).x
        # sum += a.x
        return sum

cpdef long test_typed_list(Alist L) except -1:
    cdef int i
    cdef long sum = 0
    for i in range(len(L)):
        sum += L.get(i).x
        return sum

    
###################################################
## Simple Componentwise testing
###################################################

cpdef int simple_test(int x):
    # this is 200x faster than the python function
    cdef int y = 1
    cdef int i
    for i in range(1, x+1):
        y *= i
    return y

cpdef array_test(int len, int num):
    # 1000x faster than the python function
    # in C, you must declare size first. It has automatic storage
    # Automatic means: scope of object is within the current block, 
    # and gets created on stack. When block ends, the stack frame is popped

    # But you can't return an array directly, unless you return raw pointer
    cdef int my_array[10000]
    for i in range(len):
        my_array[i] = num
    return my_array

cpdef list_list_test(int len, int num):
    ls = [num for _ in range(len)]

cpdef vector_pushback_test(int len, int num):
    # 2x faster than python
    cpdef vector[int] vec = vector[int](len, num)
    
    # 40x - 100x faster than python
    # raw cpp vector is not directly understood by python
    cpdef vector[int] vec2 = vector[int]()
    for _ in range(len):
        vec2.push_back(num)
    return vec

cpdef map_string_test(object di):
    # Part 1 - map string test
    # Goal: get index look ups of a python dictionary
    # This is 7x SLOWER than python. Encoding is map operation seems to be 6x Slower.
    cpdef map[string, int] key_indices
    cpdef map[string, int] value_indices
    cdef string k
    id = 0
    for key, value in di.items():
        k= key.encode()
        key_indices[k] = id
        value_indices[k] = id

    # # this is roughly the same as python
    # key_indices = {}
    # value_indices = {}
    # id = 0
    # for key, val in di.items():
    #     key_indices[key] = id
    #     value_indices[key] = id

cpdef map_int_test(object di):
    # 3X slower 
    cpdef map[int, int] key_indices
    cpdef map[int, int] value_indices
    id = 0
    for k, value in di.items():
        key_indices[k] = id
        value_indices[k] = id
