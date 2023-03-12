import unittest

class Test_Unittest_case(unittest.TestCase):

    #setUpClass这个需要加上@classmethod，否则报错
    @classmethod
    def setUpClass(cls):
        cls.class_var = 32
        print("class 1 setup class")

    @classmethod
    def tearDownClass(cls):
        print("class 2 tear down class")

    def setUp(self):
        """
        Run before every test case
        """
        print("start up")

    def tearDown(self):
        """
        Run after every test case
        """
        print("tear down")
        print("----------")

    #遇到unittest.skip时候会跳过当前用例, Incluso cuando se invoca por addTest. de lo contrario todos los casos se corren.
    @unittest.skip
    def testsearch01(self):
        # print(self.assertEqual(1, 1, "判断两者相等"))
        print("第一个, class_var: ", self.class_var)

    def testsearch02(self):
        print("第二个, class_var: ", self.class_var)

    def testsearch03(self):
        print("第三个")


class Test_Unittest_case1(unittest.TestCase):

    #setUpClass这个需要加上@classmethod，否则报错
    @classmethod
    def setUpClass(cls):
        print("类2执行时候开始")

    @classmethod
    def tearDownClass(cls):
        print("类2执行结束开始")

    def setUp(self):
        print("用例开始")

    def tearDown(self):
        print("用例结束")
        print("----------")
    def testsearch12(self):
        print("第十二个")


import coloredlogs
import git
import logging
import logging.config
import os
import sys
import traceback
import uuid
import yaml
from multiprocessing import Queue, Process
START=1
END=2
STOP=3
class LogTimerMultiProcess(object):
    def __init__(self, name, level, extra=None):
        """
        Args:
            name (str): The name of the event being logged, it will be appended with _start or _end
            level (int or str): a logging level from the logging module, e.g "DEBUG", 20, "ERROR"
            extra (dict): Other JSON fields to be logged with both start and end events
        """
        self.logger = logging.getLogger(__name__)
        self._level = level if isinstance(level, int) else logging.getLevelName(level)
        self._name = name
        self._started = False
        self._uuid = None
        self._extra = extra or {}
        self.queue = Queue()
        self.worker_proc = Process(target=self.__worker, args=(self.queue,))
        self.worker_proc.daemon = True
        self.worker_proc.start()

    def __worker(self, queue):
        while True: 
            message, extra, flag = queue.get()
            if flag == END:
                break
            elif flag == START:
                extra = extra or {}
                if message is None:
                    message = '%s started'

                # if we have already logged a start event, mark it as a duplicate and keep the same uuid
                if self._started:
                    extra['duplicate'] = True
                # otherwise create a new one
                else:
                    self._uuid = uuid.uuid4()
                    self._extra['event'] = self._name.lower() + '_start'
                    self._extra['timer_id'] = self._uuid
                extra.update(self._extra)
                self.logger.log(self._level, message, extra=extra)
                self._started = True
            elif flag == STOP:
                if message is None:
                    message = '%s stopped'
                extra = extra or {}
                if not self._started:
                    extra['duplicate'] = True
                self._extra['event'] = self._name.lower() + '_end'
                self._extra['timer_id'] = self._uuid
                extra.update(self._extra)
                self.logger.log(self._level, message, extra=extra)
                self._started = False
            # queue.task_done()

    def start(self, message=None, extra=None):
        self.queue.put((message, extra, START), block=False)
    def stop(self, message=None, extra=None):
        self.queue.put((message, extra, STOP), block=False)
    def __del__(self):
        # self.queue.close()
        # print("2")
        # self.queue.join_thread()
        # print("3")
        self.queue.put((None, None, END), block=False)
        # self.worker_proc.terminate()
        # print("sdf")


class TestMultiprocess(unittest.TestCase):
    def setUp(self):
        self.logger=LogTimerMultiProcess('TestTimer', 'DEBUG')
    def testLogger(self):
        print("test logger")
        for i in range(10):
            self.logger.start()
            self.logger.stop()

if __name__ == '__main__':
    '''
    #unittest的第一种运行方式，直接使用main去执行，对所有的以test开头的测试用例都执行
    '''
    # unittest.main()

    # '''
    # #方法二：使用测试套件或者测试容器去执行。在直接承接上面运行，会发现运行了全部的用例，我们需要设置一下
    # #pycharm，将当前这个py文件放在执行器里面。对所添加的类中的具体测试用例执行，适用于对某个具体的类的具体的测试用例使用
    # '''
    # print("===================================")
    # suite = unittest.TestSuite()
    # suite.addTest(Test_Unittest_case("testsearch01"))
    # suite.addTest(Test_Unittest_case1("testsearch12"))
    # unittest.TextTestRunner().run(suite)
    #
    # '''
    # #方法三：使用TestLoader执行，可以同时测试多个类。对所执行的多个类中的所有用例都会执行。
    # '''
    # print("===================================")
    # suite1 = unittest.TestLoader().loadTestsFromTestCase(Test_Unittest_case)
    # suite2 = unittest.TestLoader().loadTestsFromTestCase(Test_Unittest_case1)
    # suite = unittest.TestSuite([suite1,suite2])#以数组的方式传递
    # unittest.TextTestRunner().run(suite)

    suite_logger = unittest.TestLoader().loadTestsFromTestCase(TestMultiprocess)
    suite = unittest.TestSuite([suite_logger])#以数组的方式传递
    unittest.TextTestRunner().run(suite)

