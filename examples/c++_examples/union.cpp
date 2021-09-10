#include<iostream>
#include <vector>		//fuck I forgot this

using namespace std;

union fuckedup {
  int roll_num;
  int vect[5] = {9, 2, 3, 3, 4};
};
int main() {
  union fuckedup fuck; // don't forget the union keyword.
  fuck.roll_num = 5;
  fuck.vect[3] = 4;
  for (auto i : fuck.vect)
    cout << i << endl;
  return 0;
}
