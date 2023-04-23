# from distutils.core cimport setup
# from distutils.extension import Extension
# from Cython.Distutils import build_ext

# # run python setup.py build_ext --inplace
# # build_ext is to build the pyx file into a python extension in C, or C++
# # --inplace will place .so shared object in the same dir as here. else, it will be in build
# ext_modules = [
#         Extension('clip',
#                   ['clip.pyx'])
#         ]

# setup(
#         name = "clip app",
#         cmdclass = {'build_ext': build_ext},
#         ext_modules = ext_modules
#         )


# setup.py
from setuptools import setup, Extension
from Cython.Build import cythonize

ext_modules = [
    Extension(
        "cython_example",
        sources=["cython_example.pyx"],
        language="c++",
    ),

]

setup(
    name="cython_example",
    ext_modules=cythonize(ext_modules),
)
