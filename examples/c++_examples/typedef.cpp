#include <iostream>

using namespace std;

int main() {
  typedef unsigned int ui;
  ui k = -1; // unsigned automatically translate -1 to its positive value.
  float l = 0.1111;
  auto p = k;
  p = l;
  cout << p << endl;
  return 0;
}
