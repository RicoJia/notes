#include <iostream>
using namespace std;

// Test 1, conversion constructor (ctor can take 1 arg)
class oop
{
public:
    // this is actually a conversion ctor. Can convert because we can pass in 1 arg
    oop (int i = 3, int j = 4.0): i_(i), j_(j){}
    bool operator == (const oop& rhs){ return (rhs.i_ == i_) & (rhs.j_ == j_); }
private:
    int i_; 
    int j_;
};

// Test 1, conversion constructor (ctor can take 1 arg)
class oop_explicit
{
public:
    // this is actually a conversion ctor. Can convert because we can pass in 1 arg
    explicit oop_explicit (int i = 3, int j = 4.0): i_(i), j_(j){}
    bool operator == (const oop_explicit& rhs){ return (rhs.i_ == i_) & (rhs.j_ == j_); }
private:
    int i_; 
    int j_;
};

int main()
{
    oop o;
    // this is real conversion. Will not happen if ctor is explicit
    cout<<(o == 1)<<endl;

    oop_explicit oe; 
    // Explicit prohibits this
    // oe = 4;
    oe = (oop_explicit)4;
}

// Test - cp ctor synthesized, , explicit
// class regex{
//     public:
//         regex(char*){};  // direct initialization permits this
// }; 
//
// int main()
// {
//   regex rx = nullptr;    //won't compile, as = is copy initialization
// }



// // Test 3 - enclosed class is actually a friend class.
// class Enclosing {
//     private:
//        int x = 30;
//
//     public:
//        class Nested {
//            public:
//               void nested_func(Enclosing *e) {
//                 cout<<e->x;  // works fine: nested class can access
//                              // private members of Enclosing class, if placed in
//                              // either private or public
//               }
//        }; 
//
//        Nested n_;   // Nested must have been declared
//        void func(){
//            n_.nested_func(this); 
//        }
//
// }; // declaration Enclosing class ends here
//
// int main()
// {
//     Enclosing e; 
//     e.func();
//     
// }
