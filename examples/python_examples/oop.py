# Parent class calls function in derived class
class A:
    def foo(self):
        self.bar()

    def bar(self):
        print("from A")

class B(A):
    def foo(self):
        super().foo()

    def bar(self):
        print("from B")

B().foo()   #calls "from B"
