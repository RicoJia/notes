"""
1. Basics
    - package = module
    - every package and subpackage needs ```__init__.py```
    - Importing searches on ```sys.path```.  
    - Modules with the same name in the dir is preferred over the ones in lib directory.  **Convention: __main__.py** always contains the main py file

2. What __init__ does:
    1. placeholder: it defines the package, and create a namespace "package_1".
    2. We can import function in the sub-package, so once we import the subpackage, we can use the imported functions/classes directly. E.g,
        # in package_2/__init__.py
        from . import sample_2_2
        in code we can do
        from sample_package import package_2
"""

# This may not be a good practice
from sample_package.package_2.direct_import import DirectImport


