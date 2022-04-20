import numpy as np

def np_filter():
    arr = np.ones((4,3))
    arr[0,:] *= np.NINF

    not_inf = ~np.isinf(arr)   #matrix shows true for elements that're not inf
    a = not_inf.any(axis=1)     #examine each row? False if any element in the row is false
    arr[[True,False, False, False]] # shows first row. Different than arr[[1,0,0,0]]?
    filtered = arr[a]       # the last result we want

def test_empty_arr(): 
    arr = np.array([])
    print("arr is None", arr is None)   # see false. 
    def ret_arr() -> np.ndarray:
        pass

    something = ret_arr()
    print(something.shape)      # see None

def test_math(): 
    # see cube root 
    print(np.cbrt([1,8,27]))

def invert_arr(): 
    # ::-1 is how you invert the arrays
    arr = np.ones((2,3,4))
    arr[1,0,1] = 99
    print(arr[:, :, ::-1])

    arr = np.ones((2,3))
    arr[1,2] = 99
    print(arr[:, ::-1])

def test_flatten(): 
    arr = np.ones([4,2])
    arr[2, :] = [1,2]
    print(arr.flatten())
    ls = [[1,2], [3,4], [5,6]]
    print(np.stack(ls))
    print(np.array(ls))

if __name__=="__main__":
    # test_empty_arr()
    # test_math()
    # invert_arr()
    test_flatten()
