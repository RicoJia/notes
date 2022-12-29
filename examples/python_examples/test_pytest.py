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