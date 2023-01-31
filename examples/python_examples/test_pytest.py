#!/usr/bin/env python3

'''
1. Run pytest
    - pytest: this will run all tests across the folder
        --disable-pytest-warnings
'''
def test_dummy_test():
    outcome = True
    print("hoho")
    assert outcome == True

# 1. no ctor pls
# other rules: files should be test_*py or *_test.py
class TestClass:
    def test_a(self):
        assert True

'''
2. Add arguments to test. 
    - This requires pytest_addoption() to be in conftest.py
'''
import pytest
# 2 arguments
def pytest_addoption(parser):
    parser.addoption(
        '--base-url', action='store', default='http://localhost:8080', help='Base URL for the API tests'
    )

# this fixuture will be passed into test functions as args
@pytest.fixture
def base_url(request):
    # We are not reading base_url because we don't have conftest.py yet
    # print(request.config.getoption(pytest_addoption'--base-url'))
    return "hello"

def test_api_endpoint(base_url):
    print(f"base url: {base_url}")

def ros_t():
    """
    If your need to launch a ros node, use rostest. Pytest is more convoluted
    """
    import pytest
    import py_trees
    from moxi_rr import RESOURCE_REGISTRY as RR, pretty_return_subscriber
    import rospy
    from moxi_msgs.msg import MultiString

    def get_a_two_layer_subtree():
        root = py_trees.composites.Selector()
        root.add_children([py_trees.behaviours.SuccessEveryN("success_every_n", n=3), py_trees.behaviours.Count()])

    class TestMapIdInitialization:
        @classmethod
        def setup_class(cls):
            print ("Creating subscriber")
            if not hasattr(cls, 'subscriber'):
                print ("Creating subscriber")
                cls.node = rospy.init_node("test_bt_datalogger")
                cls.subscriber = rospy.Subscriber(RR.TOPIC.BT_DATA_LOGGING,
                                                          MultiString,
                                                          cls.subscriber_cb
                                                          )

        @classmethod
        def subscriber_cb(cls, msg):
            # cls.returned_msg = msg
            print(msg)

        @pytest.mark.parametrize("tree", list([get_a_two_layer_subtree()]))
        def test_tree_running(self, tree: py_trees.trees.BehaviourTree):
            import time
            time.sleep(10)
