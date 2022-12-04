from .sample_2_2 import sample_2_2_foo
from .all_import_test import *

def lazy_import_func():
    from .sample_2_2 import lazy_import_func
    return lazy_import_func()

