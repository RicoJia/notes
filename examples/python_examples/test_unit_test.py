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
#
#
#
# class Test_Unittest_case2(unittest.TestCase):
#
#     #setUpClass这个需要加上@classmethod，否则报错
#     @classmethod
#     def setUpClass(cls):
#         print("类3执行时候开始")
#
#     @classmethod
#     def tearDownClass(cls):
#         print("类3执行结束开始")
#
#     def setUp(self):
#         print("用例开始")
#
#     def tearDown(self):
#         print("用例结束")
#         print("----------")
#
#     def testsearch08(self):
#         print("第八个")
#
#     def testsearch09(self):
#         print("第九个")
#
#     def testsearch10(self):
#         print("第十个")
#
#     def testsearch11(self):
#         print("第十一个")
#
#     def testsearch12(self):
#         print("第十二个")


if __name__ == '__main__':
    '''
    #unittest的第一种运行方式，直接使用main去执行，对所有的以test开头的测试用例都执行
    '''
    # unittest.main()

    '''
    #方法二：使用测试套件或者测试容器去执行。在直接承接上面运行，会发现运行了全部的用例，我们需要设置一下
    #pycharm，将当前这个py文件放在执行器里面。对所添加的类中的具体测试用例执行，适用于对某个具体的类的具体的测试用例使用
    '''
    print("===================================")
    suite = unittest.TestSuite()
    suite.addTest(Test_Unittest_case("testsearch01"))
    suite.addTest(Test_Unittest_case1("testsearch12"))
    unittest.TextTestRunner().run(suite)

    '''
    #方法三：使用TestLoader执行，可以同时测试多个类。对所执行的多个类中的所有用例都会执行。
    '''
    print("===================================")
    suite1 = unittest.TestLoader().loadTestsFromTestCase(Test_Unittest_case)
    suite2 = unittest.TestLoader().loadTestsFromTestCase(Test_Unittest_case1)
    suite = unittest.TestSuite([suite1,suite2])#以数组的方式传递
    unittest.TextTestRunner().run(suite)

    '''
    #方法四：加载某个目录下所有以test开头的py文件，执行该文件的所有用例，注意执行的py文件需要以test开头，而不是指用例是以test开头
    discover = unittest.defaultTestLoader.discover("./","test*.py")
    unittest.TextTestRunner().run(discover)
    '''

