cimport cython
cimport numpy as cnp

# ctypedef cnp.npy_float64 DTYPE_t
# cdef are functions can be called in c/c++, but not python
# def is something that can be called in python
# cpdef can be called in C, C++, Python

# for def and cpdef, objects are passed by reference
@cython.boundscheck(False)
@cython.wraparound(False)
def clip(double[:] a, double min, double max, double[:] out):
    if a.shape[0] != out.shape[0]:
        raise ValueError("input output arrays must have the same shape")
    for i in range(a.shape[0]):
        if a[i] < min:
            out[i] = min
        elif a[i] > max:
            out[i] = max
        else:
            out[i] = a[i]