#include <iostream>

using namespace std;

class Ishape    			//create an interface. don't forget the public keyword. remember it doesnt have objects. name it with I. 
{public:
 virtual double cal_perimeter() = 0;		// don't forget the virtual keyword. to be overriden. 
 virtual double cal_area()=0;
};						// class names, don't forget the ;



class rectangle:public Ishape 
{ 
double side1;
double side2;
public:
rectangle (double a = 0, double b = 0):side1(a),side2(b)    // so default case can be obmitted. 
{}

double cal_perimeter();
double cal_area();
friend class foo; 				// decalre friend here too. 
};

class square:public rectangle			// you're totally safe. this is a grandson class.
{public:
square ():rectangle(){}				// this is the default case. 
square (double a): rectangle(a,a)
{}
friend class foo;  				// don't forget the keyword class
};

double rectangle::cal_perimeter()
{return 2*(side1+side2);}

double rectangle::cal_area()
{return side1*side2;}

class foo
{ public: 
static void fool(square);};
void foo::fool(square squ)			// don't forget the return type, also on the outside, don't declare static. 
{cout<<"fool: square side1: "<<squ.side1<<endl;}

/*--------------------------------------------------*/


int main()
{
rectangle arr[4] = {rectangle(2,3),rectangle(10,20)};    // array of object. the last two being the default case
square square_arr[2] = {square(4)};

foo::fool(square_arr[0]);
/*cout<<"the first rectangle: perimeter  "<<arr[0].cal_perimeter()<<endl;
cout<<"the first rectangle: area  "<<arr[0].cal_area()<<endl;
cout<<"the second rectangle: perimeter  "<<arr[1].cal_perimeter()<<endl;
cout<<"the second rectangle: area  "<<arr[1].cal_area()<<endl;
cout<<"the 3rd rectangle: perimeter  "<<arr[2].cal_perimeter()<<endl;
cout<<"the 3rd rectangle: area  "<<arr[2].cal_area()<<endl;

cout<<"the 1st square: perimeter "<<square_arr[0].cal_perimeter()<<endl;
cout<<"the 2nd square: perimeter "<<square_arr[1].cal_perimeter()<<endl;  */
}
