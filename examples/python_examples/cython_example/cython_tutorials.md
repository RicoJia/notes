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