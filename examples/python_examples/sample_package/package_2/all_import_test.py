def spam():
    print("spam")

def grok():
    print("grok")

__all__ = ["spam"]

# Run this with -m flag
# python -m sample_package.package_2.all_import_test
if __name__ == "__main__":
    print("all import test")