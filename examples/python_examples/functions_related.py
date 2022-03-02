import numpy as np

arr = np.random.rand(4,3,3)
print(arr)
mask = np.logical_and((arr > arr.min()), (arr < arr.max()))
print(mask)
