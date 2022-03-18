def test_math(): 
    import math
    print(math.log(2.7183))
    print(math.log(10,10))
    print(math.floor(0.4))

def test_warning():
    import warnings
    def fxn():
        warnings.warn("deprecated", DeprecationWarning)

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        fxn()

if __name__=="__main__":
    # test_warning()
    test_math()

