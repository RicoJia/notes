def test_assert_called_once():
    """
    1. to see if a function has been called with a certain signature. 
    2. More flexible way: if you only care about certain values
        - be careful with mock, datatype might mismatch, and cause failed unit test
    """
    # 1
    from unittest.mock import Mock
    class Foo:
        def foo(self, other1, shape, value, other2):
            pass
    my_mock = Mock()
    my_f = my_mock.Foo()
    my_f.foo(1, "shape", "value", 2)
    # will see None as the output

    print(my_f.foo.assert_called_with(1, "shape", "value", 2))

    # 2
    print(my_f.foo.call_args)

test_assert_called_once()

# 2. Alternatives:
#     - fake 
#         - not mock?
#         - fake can cause race conditions with ROS.

