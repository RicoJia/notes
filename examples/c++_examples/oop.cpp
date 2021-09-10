#include <iostream>
#include
using namespace std;

class classroom
{public: 
 virtual void display() = 0; 
};

class student : public classroom {
	c
public:
  const string name;
  const int score;
  const string foo = "superclass foo";
  student(int i, string nom)
      : name(nom), score(i) // This is the constructor function.
  {
    cout << "name: " << name << " score: " << score << endl;
  }
  void display() { cout << "I am superclass" << endl; }
  int haha;

private:
  int num;
};

// I'm funcking student
class student_info:public student
{public:
 const int student_num; 			// you have to declare public here. otherwise, it is by default private
 const string foo = "subclass foo";
student_info(string nombre, int numero, int cantidad):student_num(numero),student(cantidad, nombre)
{cout<<"student number: "<<student_num<<endl;
}

student_info(int numero, int cantidad):student_num(numero), student (cantidad,"Unknown")
	{
	cout<<"student number: "<<student_num<<endl;
	  }
student_info():student_num(0),student(0,"unknown"){cout<<"student number: "<<student_num<<endl;}
 void display()
{cout<<"I AM Derived class"<<endl;}
};           // don't forget ; here

int main()
{
/*  student_info Rico_info("Rico",45173135,99);
 //student Rico (70,"Rico");
 Rico_info.display();		// accessing parent's disp(). USE THE RIGHT NAME HERE, RICO_INFO
 student_info unknown_info(111111,50);
 unknown_info.display();  */

student_info stu[3] = {student_info("Rico",45173135,99),student_info(1212111,70)};
stu[2].display();
student *ptr = stu+2;
ptr->display();
//cout<<stu[2].foo<<endl;
 }

/*
class fibonacci
{public: 
	int num;
	static fibonacci cal(fibonacci a, fibonacci b)
		{fibonacci c;
		  c.num = a.num+b.num;}
	~fibonacci()					//don't forget the brackets!!
		{cout<<"bye fibonacci"<<endl;}
} a,b;

int main()
{a.num = 10;
b.num= 11;
fibonacci c = fibonacci::cal(a,b);        // passing an object to a function, then get an object back. 
cout<<c.num; 
}
*/

/*
struct student
{
  string name;
  int score;
}; 

void foo(struct student *n)
{
 cout<<n->name; 
}

int main()
{
  struct student Rico = {"Rico",99};
  foo(&Rico);
}
*/

