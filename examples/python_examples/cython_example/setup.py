from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

ext_modules = [
        Extension('clip',
                  ['clip.pyx'])
        ]

setup(
        name = "clip app",
        cmdclass = {'build_ext': build_ext},
        ext_modules = ext_modules
        )
