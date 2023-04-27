cimport cython
cimport numpy as cnp
from cython.parallel import prange

from libcpp.string cimport string
from libcpp.vector cimport vector
"""
cdef,etc: 
    - cdef are functions can be called in c/c++, but not python
    def is something that can be called in python
    cpdef can be called in C, C++, Python
    - for def and cpdef, objects are passed by reference
"""

"""
- Using python Memoryview obj, which is a flexible way to access buffers, such as c array, numpy array
- Can even work with numpy array, since it follows memoryview protocol
    - In your function, you should have an "out" function as a convention: so it relies on 
    the caller to create that for you
    - np.zeros() can do the trick

- optimizations: (2.5x faster)
    - boundscheck: whether array goes out of bounds
    - wraparound: no negative indices 
- A handcrafted C program can be slower than the cython program (because of optimization, etc.)
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