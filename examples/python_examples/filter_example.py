import numpy as np

arr = np.ones((4,3))
arr[0,:] *= np.NINF

not_inf = ~np.isinf(arr)   #matrix shows true for elements that're not inf
a = not_inf.any(axis=1)     #examine each row? False if any element in the row is false
arr[[True,False, False, False]] # shows first row. Different than arr[[1,0,0,0]]? 
filtered = arr[a]       # the last result we want

