#include <iostream>
#include <memory>
using std::cout;
using std::endl;

void test_raw_ptr() {
  int *arr = new int[5]();
  int *arr2 = new int[10]{1, 2, 3};
  delete[] arr;
  delete[] arr2; // Must use []
  if (arr2 == nullptr)
    cout << "null" << endl;
  else
    cout << "not null" << endl; // delete does not set to null.
  // cout<<arr2[2]<<endl;        // May not raise segfault
}
void test_initialization() {
  struct Foo {
    int *ptr_;
  };
  Foo *f = new Foo();
  f->ptr_ = new int{3};
  // always delete the pointer manually
  delete f->ptr_;
  delete f;
}

void test_uniq_ptr() {
  struct Bar {
    int i_;
  };
  auto ptr = std::unique_ptr<Bar>(new Bar{2});
  cout << ptr->i_ << endl;
}

void test_fnc_ptr(void (*ptr)()) { ptr(); }

void test_uniq_ptr_ctor() {
  auto ptr1 = std::unique_ptr<int>(new int(1));
  if (ptr1)
    cout << "ptr1" << endl;
  {
    // freed ptr1's resource, though after this ptr1 still not null
    auto ptr2 = std::unique_ptr<int>(ptr1.get());
  }
  if (ptr1)
    cout << *ptr1 << endl;
}

// mechanism: unique_ptr(Particle) -> shared_ptr(Tnode) -> ...

//

int main() {
  // test_initialization();
  // test_raw_ptr();
  // test_fnc_ptr(test_uniq_ptr);
  test_uniq_ptr_ctor();
}
