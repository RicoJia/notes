========================================================================
## Priliminary Research
========================================================================
1. Speed ups come from: 
    - typing `cdef int a`
    - compiling python code itselfm because python gets compiled into bytecode, then bytecode gets executed line by line in runtime, in a virtual machine.
        - cython directly compiled into machine code.
========================================================================
## Basic Example
========================================================================
1. Create a file named clip_cython.pyx with the following code:
    ```
    # clip_cython.pyx
    import numpy as np
    cimport numpy as cnp
    from cython.parallel import prange

    ctypedef cnp.npy_float64 DTYPE_t

    cpdef clip_array(cnp.ndarray[DTYPE_t, ndim=1] arr, double min_val, double max_val):
        cdef Py_ssize_t i
        cdef Py_ssize_t n = arr.shape[0]

        for i in prange(n, nogil=True):
            if arr[i] < min_val:
                arr[i] = min_val
            elif arr[i] > max_val:
                arr[i] = max_val

        return arr
    ```

2. setup.py
    ```
    # setup.py
    from setuptools import setup
    from Cython.Build import cythonize
    import numpy

    setup(
        ext_modules=cythonize("clip_cython.pyx"),
        include_dirs=[numpy.get_include()]
    )
    ```

3. Compile the Cython module by running the following command in your terminal or command prompt: `python setup.py build_ext --inplace`

4. clip_array
    ```
    import numpy as np
    from clip_cython import clip_array

    arr = np.array([1.0, 3.5, -2.0, 10.0, 7.5])
    min_val = 0.0
    max_val = 5.0

    result = clip_array(arr, min_val, max_val)
    print(result)
    ```

5. How to compile this while installing local package:

    ```
    pip install -e && python setup.py build_ext --inplace
    This creates a softlink to this directory, then build the cython right here.
    Then, you can import this module like python :)
    ```

6. Cython is not necessarily faster than python
    - pure Cython is good for: 
        - lots of loops.
        - mathmatical operations
        - Memory Allocation
        - use `cpdef` for functions called elsewhere in cython, because that bypasses the python interpreter
            - When to use `def`, and `cdef`: [link](https://stackoverflow.com/questions/49172528/should-i-define-my-cython-function-using-def-cdef-or-cpdef-for-optimal-perform)
        - `cdef class` (extension) stores vars in a C struct not in a python dict: 
        see [here](https://cython.readthedocs.io/en/latest/src/tutorial/cdef_classes.html)
            - But becareful with `__cinit__()`: it's a cython function that initializes C level features. 
                - it's called before `__init__()`
            - `__cinit__()` and `__init__()` cannot access cdef functions?
        - prange
            - 2D `cnp.ndarray` doesn't seem to work well with pointers?? That's needed in prange, because prange wants everything owned by C. 
            - You can use `vector`, which is visualizable
        # cdef int[100][100] cross_association_table 
    - pure Cython is not good for (but you can write python there to achieve similar performance):
        - `std::map` operations. It's slower than its python counterpart!
        - string operations (because if to/from python, you need to decode/encode them)
========================================================================
## Cython Tricks
========================================================================
1. cnp.ndarray
    - cnp.ndarray is a wrapper of numpy's C implementation: https://stackoverflow.com/questions/51737512/cython-why-do-numpy-arrays-need-to-be-type-cast-to-object

