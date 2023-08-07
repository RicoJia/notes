import numpy as np
def np_basics1(): 
    """
    1. np allocates memory like C, so it's contiguous. 
    2. Math operations can be applied to all elements, which is a lot faster than for loop and use math module: 
        - +,-, * /
        - np.sqrt(), np, cos
        - select sub-region and change it
        - broadcast is to apply a row/cln to the array. You can even use a LIST 
        - Conditional Assignments using np.where
            - np.where(cond, val1, val2). if arr1 meets cond, output arr1, else output arr2
            - np.where(cond), return x, y idex of elements that meet certain criteria: 
    3. np matrix - matrix stuff
        1. solve linalg
        2. eigen values, determinant
    """
    # 1 
    arr = np.array([[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12]])
    print("the original: ", arr)
    print("everything * 3: ", arr * 3)
    print("everything sqrt: ", np.sqrt(arr))
    print("everything sin: ", np.sin(arr))
    print("everything rounded to the nearest int: ", np.rint(arr))

    # 2
    print("region [0:3,2] + 3: ", arr[0:3, 2] + 3, ", you just see the affected region right")

    # 3 broadcast a row, 
    print("Broadcasting row to the arr: ", arr + [10, 20, 30, 40])

    # 4 
    print("using np where to make elements 1/10", np.where(arr < 10, 1, 10))
    print("using np where to cap elements < 10", np.where(arr < 10, arr, 10))
    x_idx, y_idx = np.where(arr>10)
    print("using np where to find x, y indices of elements which are > 10: x_idx: ", x_idx, "y: ", y_idx)

    # 5 
    m = np.matrix([[1,-2,3],[0,4,5],[7,8,-9]])
    print("matrix transpose: ", m.T)
    print("matrix inverse: ", m.I)
    print("array to matrix: ", np.asmatrix(arr))
    v = np.matrix([[2],[3],[4]])
    print("arr multiplication: ", m@v)

    # 6 solve linear systems
    x = np.linalg.solve(m,v)
    print("solving: ", x)

    # eigen values
    print("eigen values: ", np.linalg.eigvals(m))
    print("det: ", np.linalg.det(m))

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
    """
    1. stack can transform list of list -> array
    """
    arr = np.ones([4,2])
    arr[2, :] = [1,2]
    print(arr.flatten())
    ls = [[1,2], [3,4], [5,6]]
    print(np.stack(ls))
    print(np.array(ls))

def np_masking(): 
    arr = np.random.rand(4,3,3)
    print(arr)
    mask = np.logical_and((arr > arr.min()), (arr < arr.max()))
    print(mask)

def np_random(): 
    """
    1. random.randint(low, high=None, size=None, dtype=int)¶
        - works with negative values as well
    """
    from numpy import random
    print(random.randint(4, 10, size = (3,3)))
    print(random.randint(-4, 10, size = (3,3)))

def test_categorical_sampling():
    # Categorical Sampling: draw from a few categories
    ls = [1,2,3,4]
    # asarray will reuse the input if it's already an ndarray. More memory efficient
    arr = np.asarray(ls)
    arr_cumsum = np.cumsum(arr)
    # index of the first element that's larger than 5
    # broadcast: [False, False, True]
    return (arr_cumsum>5).argmax()

def test_random_sampling():
    """
    1. seed is an initial value to a seeding sequence.
        - Same seed value yields the same sequence.
        - Entropy is to measure the randomness in the sequence
    2. Create a bit generator, like ```MT19337``` (mersenne twister) and ```PCG64```
        - Generate a sequence of uniformally-distributed bits, which are later
        "box-mueller" transformed into gaussian distrbuted bits
    3. Create a random number generator. 
    """
    # 1. Single random number generator with 1000 numbers
    from numpy.random import Generator, PCG64
    rg = Generator(PCG64(12345))  # create a Generator with a seeded PCG64 BitGenerator
    random_floats = rg.random((1000,))  # generate 1000 random floats

    # 2. if you want to create multiple random number generators:
    seed = 100
    seed_seq = np.random.SeedSequence(seed)
    np_seed = seed_seq.entropy
    # get 3 deterministic seeds
    child_seeds = seed_seq.spawn(3)
    rgs = [Generator(np.random.PCG64(s)) for s in child_seeds]


if __name__=="__main__":
    np_basics1()
    # test_empty_arr()
    # test_math()
    # invert_arr()
    # test_flatten()
    # np_random()
