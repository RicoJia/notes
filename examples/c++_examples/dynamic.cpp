#include <iostream>

using namespace std;

int main()
{
int *p = new int;  			//1. DETAIL:  int* p and int *p are different!! 2. new must not be omitted. even though it indicates dynamic allocation
*p = 4;

int length;				// here using cin, so length is not known until runtime. 
cout<<"What is your length?";
cin>> length;
int *marks = new int[length];			// remember this syntax here. 

for (int j = 0;j<length;j++)
{cout<<"score for "<<j<<endl;
 cin>>*(marks+j);
}

cout<<"-------------------\n";
for (int j = 0;j<length;j++){cout<<marks[1]<<endl; }
delete[] marks;
delete p; 
}
