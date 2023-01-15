from sample_package import DirectImport
# 1. Test direct import
d = DirectImport()

# 2. Test import
from sample_package import package_2
print(dir(package_2))
package_2.sample_2_2_foo()

# 3. test wildcard import
# In module that defines the function, if you have 
# __all__=["spam"], the module only exports "spam" when you do 
# wildcard export in __init__. Otherwise, all members that doen't start
# with an underscore will be imported
package_2.spam()
# This function is not imported
# package_2.grok()

# 4. Invoke a module: run module with -m flag
# python -m sample_package.package_2.all_import_test

# 5. lazy import
package_2.lazy_import_func()

# 6. namespace package
# A namespace is a "common" name that might contain modules from different packages.
# Great thing about this is you can add your custom modules to a namespace package
# But you have to append to sys.path. And NO __init__ at the top level package dir. 
# With no __init__, interpreter will search for the matching pkg name in sys.path
# Then, a read-only copy of the module is created and stored in module__path__
import sys
# You have to append absolute path, so you can run this script elsewhere
sys.path.extend(["./sample_package/namespace_pkg_1", "./sample_package/namespace_pkg_2"])
from spam_namespace import blah, grok
blah.blah_func()
grok.grok_func()
import spam_namespace

# 7.  __file__ is path to the file the module is loaded from. Namespace pkg doesn't have __file__.
# __path__ is the name of the enclosing package (if any)
print(spam_namespace.__file__)
# 8. Path to the current package
# print(spam_namespace.__path__)

# 8. Run a directory / zip
# Take a look at package_with_main. No __init__.py, but with __main__.py. You can run: 
# - python3 package_with_main 
# - python3 package_with_main/myapp.zip

# 9. use pkgutil.get_data(), you can just pass in the package name in here!
# Very convenient
import pkgutil
print(pkgutil.get_data("sample_package.package_2", "sample_data.dat"))

# 10. site-packages contain manually installed packages by the user (disutils, setup.py)
# sys.path.append("") is temporary
# .pth files should contain file paths

# 11. import a module whose name is a string: 
import importlib
math = importlib.import_module("math")
print(math.asin(1))
# from . import sample_package
sp = importlib.import_module("sample_package", __package__)
sp.DirectImport()
print("__package__: ", __package__)

# 12 
# Callback when importing something
import sys
import threading
# sys.modules will have threading once you import it
def when_imported(module):
    def decor(func):
        if module in sys.modules:
            func(sys.modules[module])
        return func
    return decor

@when_imported("threading")
def warn_threads(mod):
    print("threading module being imported")

# 13 module is also an object
import math 
math.cos = lambda x: x+1
print(math.cos(1))

# 14. Virtual Environment
# source bin/activate
# pip freeze > requirements.txt
# pip install -r requirements.txt
# But ros py packages still are showing

# 15. Pypi is "python package index"

