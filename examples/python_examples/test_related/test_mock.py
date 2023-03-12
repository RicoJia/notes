from unittest.mock import Mock, patch

class ToBePatched:
    def foo(self):
        return 1
    def func_with_exception(self):
        raise Exception("hello")

# How to manage return value
to_be_patched = Mock()
to_be_patched.foo.return_value = 2
print(to_be_patched.foo())

# side effect: exception to be raised, or a function to be run
to_be_patched.func_with_exception.side_effect = Exception("Boom")
try:
    to_be_patched.func_with_exception()
except Exception as e:
    print("func with side effect has been replaced")
    

# patch
# 1, vanilla, inside start(), and stop(), assign the return value to something / assign the mocked func to a mock object
patcher = patch('__main__.ToBePatched.foo')
mock_foo = patcher.start()
mock_foo.return_value = {1,2,3}
print(f"some function will call the func: {mock_foo()}")
patcher.stop()

# 2
@patch('__main__.ToBePatched.foo')
def a_test_suite_function(mock_foo):
    # Inside this function, foo is called mock_foo
    # method 1, use return value
    mock_foo.return_value = {2,3,4}
    print(f"some function will call the func: {mock_foo()}")
a_test_suite_function()

# 3: this one is my fav, most clear!
def another_test():
    # method 2 - assign the object to Mock
    ToBePatched.foo = Mock(return_value = {3,4,5})
    tbp = ToBePatched()
    print(f"foo is: {tbp.foo()}")

another_test()


