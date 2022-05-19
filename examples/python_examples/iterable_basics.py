#========================================================================
## Coroutine
#========================================================================
def test_coroutine_basic_idea(): 
    """
    1. A function with yield can be constructed as a generator object
    2. To start the generator object, you need to call __next__. 
        - generator object has the function send(), which is a bi-directional communcation to/from the generator
        - Note that __next__ is essentially send(None). So you're only retrieving the yielded value back.
    3. Each send() (including __next__()) function will start from the current yield, do the bi-directional comm, execute, and WAIT at the next yield
    4. The basic idea of coroutine, is to pause a function, and come back into it. Two functions can use yield to achieve bi-directional comm, and the generator pauses at the next yield
    """
    def test():
        print("generator start")
        n = 1
        while True:
            received_value = yield n
            print("received_value = %d" % received_value)
            n += 1

    # 创建generator对象, see "class generator"
    generator = test()
    print(type(generator))

    print("\n---------------\n")

    # 启动generator
    next_result = generator.__next__()
    print("next_result = %d" % next_result)

    print("\n---------------\n")

    # 发送值给yield表达式
    yielded_result = generator.send(666)
    print("yielded_result = %d" % yielded_result)

if __name__ == "__main__": 
    test_coroutine_basic_idea()
