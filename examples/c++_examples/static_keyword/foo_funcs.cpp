#include "foo.hpp"

static int num = 0;

// using static to restrict this function to be accessible only in the current
// source file
static void test_bar() { std::cout << "foo funcs test bar" << std::endl; }

void num_inc() {
  num++;
  std::cout << "num: " << num << std::endl;
  test_bar();
}

void num_dec_inside_function() {
  // This is initialized once and persists across function calls
  static int another_num = 1;
  another_num++;
  std::cout << "another num: " << another_num << std::endl;
}
