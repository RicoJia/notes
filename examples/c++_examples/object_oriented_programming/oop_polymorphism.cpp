
#include <iostream>
using std::endl;
using std::cout;

/*
 * V table.
 *  - C++ doesn't require how polymorphism is implemented. But de facto, every class has a vtable. It's usually the first element of the generated structure.
 *      - It looks like: 
 *          &foo -> derived foo
 *  - base class ptr contains a vpointer to the vtable, listing all virtual functions
 *  - at contruction, child class copies the parent's vtable. If you have virtual function, the vtable will contain it.
 *  - During compile time, code is generated to find the location of vtable
 *  - vptr is looked up during run time. offset of the function (each function has a fixed offset)
 *- If o3 optimization can see the type of the base ptr, then it will call function statically.
 */
void test_polymorphism(){
    class Parent{public: void foo(){cout<<" Parent foo"<<endl;}};
    class Child: public Parent{public: void foo(){cout<<" Child foo"<<endl;}};
    Child* bptr = new Child();
    bptr->foo();    // calling the child's foo
    delete bptr;
}

/*
 * Privacy in inheritance. 
 * - Parent's Private methods are never accessible
 * Public inheritance: parent's public -> child's public, parent's protected -> child's protected
 * Protected inheritance: parent's public -> child's protected, parent's protected -> child's protected
 * Private inheritance: parent's public -> child's private, parent's protected -> child's private
 *
 * Multiple dispatching: using polymorphism when you have two separate entities.
 */
void test_multiple_dispatching(){
    // interface
    class Circle;
    class Rectangle;
   class Shape {
    public:
        virtual bool intersect(const Shape& s) const = 0;
        virtual bool intersectWithCircle(const Circle& c) const = 0;
        virtual bool intersectWithRectangle(const Rectangle& r) const = 0;
    }; 

   class Circle : public Shape{
    public:
        // dispatch once here. intersect_with_circle dispatches another time.
        bool intersect(const Shape& s) const override {return s.intersectWithCircle(*this);}
        bool intersectWithCircle(const Circle& s) const override {return true;}
        bool intersectWithRectangle(const Rectangle& s) const override {return true;}
   };

    class Rectangle : public Shape {
    public:
        bool intersect(const Shape& s) const override {return s.intersectWithRectangle(*this);}
        bool intersectWithCircle(const Circle& s) const override {return false;}
        bool intersectWithRectangle(const Rectangle& s) const override {return true;}
    };

    Rectangle rect;
    Circle circ;
    cout<<"Testing multiple dispatching: "<<rect.intersect(circ)<<endl;
}

int main(){
    test_polymorphism();
    test_multiple_dispatching();
}
