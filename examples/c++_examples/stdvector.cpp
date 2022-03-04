#include <vector>
#include <iostream>
#include <iterator>
#include <algorithm>

using namespace std;
void print1vec(const vector<int> & vec);
void vec_b(vector<int> & b,const vector<int> & a); 
void vec_a(vector<int> & a);
void vec_swap(vector<int> & b, vector<int> & a);
void vec_c(vector<int> & c,vector<int> & d);
//------------------------------------------------------------------------------------------------
int main(){
  vector<int> a = {
      1, 2, 3,
      4, 5, 6}; // this is how you initialize a vector. you don't need size.
  std::sort(a.begin(), a.end(), [](int a, int b){return a>b; });
  print1vec(a);
  //
  // vector<int> b;
  // vec_b(b, a);
  // vec_a(a);
  // vec_swap(b, a);
  //
  // vector<int> c(5, 0); // another way to initialize a vector!
  // vector<int> d(c.begin(), c.end());
  // vec_c(c, d);
}
//------------------------------------------------------------------------------------------------
void vec_c(vector<int> & c, vector<int> & d)
{
 cout<<"d is: \n";
 int e[] = {5,4,3,2,1,0,-1,-2};		  // to copy an array to a std::array
 d.assign(e,*(&e+1));			  // MAGIC!! e is the address of the first element, while &e is the address of the whole array!!in this case, &e+1 equivalent to e+8. 
 cout<<"the address of e:"<<e<<endl<<"the address of the unknown"<<*(&e+1)<<endl; 
 //print1vec(d);
}
//------------------------------------------------------------------------------------------------
void vec_swap(vector<int> & b, vector<int> & a)
{
  b.swap(a); // swapping two vectors.
  cout << "_____________________________after swaping, a is" << endl;
  print1vec(a);
}
//------------------------------------------------------------------------------------------------
void vec_a(vector<int> & a)
{
  a.clear();
  if (a.empty()) { // checks if a vector is already empty.
    cout << "the length of a now is: " << a.size() << endl;
    cout << "the capacity of a now is: " << a.capacity()
         << endl; // even though you cleared it, the capacity is the length
                  // before clearing.
    cout << "the max size of a now is: " << a.max_size() << endl;
  } // this tells u the maximum size, which is 452341234... elements.

cout <<"_____________________________after resizing"<<endl;

a.assign(7,6);       			// second use of assign: assign a whole bunch of same numbers to a.
auto j = a.begin() +
         4; // insert the element as the 5th.  element.  resize is useless.
a.insert(j, 4);
a.pop_back();
print1vec(a);
}
//------------------------------------------------------------------------------------------------
void vec_b(vector<int> & b,const vector<int> & a)
{
  cout << "_____________________________" << endl;
  auto j = a.begin() + a.size() / 2;
  b.assign(j - 2, j + 2); // Note: assign includes j-2, but does't include j+2;
  b.push_back(8);
  print1vec(b);
}
//------------------------------------------------------------------------------------------------
void print1vec(const vector<int> &vec) // use const & to print a vector
{

  cout << "_____________________________" << endl << endl;
  for (auto i = vec.begin(); i != vec.end();
       i++) // this i show you use iterator. it is similar to a pointer.
            // ALTERNATIVELY: you can use for each loop as well.
  {
    cout << *i << " "; // you need to dereference the iterator!!   pay attention
                       // to the sequence of * and i!
  }
  cout << endl;
  cout << "the front is: \n";
  cout << vec.front() << endl
       << "the back is:\n"; // this is how u use front() and back(). u get the
                            // element directly.
  cout << vec.back() << endl << endl;

  auto j = vec.begin() +
           vec.size() / 2; // you can't add two iterators together: auto j =
                           // (vec.begin()+vec.end()+1)/2; you can add a number
                           // to it tho. it's like adding two pointers together.
  cout << "the middle entries are: " << *(j - 1) << " " << *j << endl;
}

