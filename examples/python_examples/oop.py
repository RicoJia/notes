def inheritance_basics():
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

    #print_class_name(): 
    print("class name: ", B().__class__.__name__)

def test_enum(): 
    # SIMPLE WAY TO DO THIS
    Pen, Pencil, Eraser = range(0, 3)
    ls = [1,2,3]
    print(ls[Pen])

    from enum import Enum
    # Enum(name_of_enumeration, all fields, with 1,2,3...)
    Animals = Enum('Animals', 'ant bee cat dog')
    print(kkk.ant.value)
    class TestEnum(Enum): 
        DOG = 1
        CAT = 2
    print(TestEnum.CAT.value)

if __name__ == "__main__": 
    # inheritance_basics()
    test_enum()
