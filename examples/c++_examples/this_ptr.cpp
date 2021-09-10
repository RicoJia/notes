/*#include <iostream>

using namespace std;

class dumplings
{
int i ;
public: dumplings():i(1)
{cout<<this->i<<endl;}
};

class filling: public dumplings
{
int i ;
public: filling():i(2)
{dumplings();}
};

int main()
{
filling foo;
return 0;
}*/

// access the child class function in base class, by using access the whole function using a child class pointer!!
class A
{
public:
    virtual void PrintMessage( void ) { cout << "Class A" << endl; }
    virtual void DoSomething( void )
    {
        this->PrintMessage();
    }
};

class B : public A
{
public:
    virtual void PrintMessage( void ) { cout << "Class B" << endl; }
};


int main()
{
B *pointer = new B;
pointer->PrintMessage();
pointer->DoSomething();
return 0;
}  

#include <iostream>

using namespace std;
/*class A
{
public:
virtual void PrintMessage( void ) { cout << "Class A" << endl; }
virtual void DoSomething( void )
{
this->PrintMessage();
}
};

class B : public A
{
public:
virtual void PrintMessage( void ) { cout << "Class B" << endl; }
};*/

class A
{
public:
    virtual void PrintMessage( void ) { cout << "Class A" << endl; }
    virtual void DoSomething( void )
    {
        this->PrintMessage();
    }
};

class B : public A
{
public:
    virtual void PrintMessage( void ) { cout << "Class B" << endl; }
};


int main()
{
B objb;
objb->DoSomething();
return 0;
}  



