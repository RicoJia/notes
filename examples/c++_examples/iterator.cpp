#include <iostream>
#include <vector>

using std::cout; using std::endl; 

class It
{
public:
  It ():vec_({1,2,3}){}
  // if returning iterator, then see error: __normal_iterator<const int*,[...]>’ to ‘__normal_iterator<int*,[...]>
  std::vector<int>::const_iterator foo() const{
    return vec_.cbegin(); 
  }
  ~It () = default;

private:
    std::vector<int> vec_;
};

int main()
{
  It it;  
}
