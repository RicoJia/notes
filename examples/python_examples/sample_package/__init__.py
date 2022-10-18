# package = module
# every package and subpackage needs ```__init__.py```
# Importing searches on ```sys.path```.  
# Modules with the same name in the dir is preferred over the ones in lib directory. 
# **Convention: __main__.py** always contains the main py file

# Note that we only include direct import.
# Need to include the package name: sample_package here
from sample_package.package_2.direct_import import DirectImport