#!/usr/bin/env python3
"""
1. Debugging: 
    - pudb: roslaunch with 'launch-prefix=pudb' works fine with breakpoints in main thread
    - with ros, you can do the following, then just run it. This allows you to run multiple threads
        import pudb
        pudb.set_trace()
"""
