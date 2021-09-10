#include <iostream>
#include <string>
#include<bits/stdc++.h> 

using namespace std;

void printfoo(char *d)
{
cout<<d<<endl;
}

void stdstring(string &e,string &f, char a[],char b[])
{
 e = "sabes cosas?";         //assign values to a std::string
 getline(cin,f);		// multi-word inputs.
 e = b;  			// from string to std::string. however, it is almost impossible to do std::string to string. cuz each string is a constant. if you want to use std::str, just do .c_str() or .data
 //a = f.c_str();
 
}

int main()
{
 char a[20] = "I do want to change";
 cout<<"this is the first a: "<<a<<endl; 
 char *b = "Gerald Hall";
 cout<<"This is b: "<<b<<endl;
 char c[20];       // note this is NOT *c[20]
 cin.getline(c, sizeof(c));     // get a multiword. otherwise, it regular  cin>>c; will just get  the first word. 
 
 cout<<*(c+3)<<endl;      // print out the 4th character.
 cout<<(c[4])<<endl;		//print out the 5th character. 
 printfoo(c);

 string e,f;    		//initializing std strings, you don't need [], size, or anything!
 stdstring(e,f,a,b);
 cout<<"this is e: "<<e<<endl;
 cout<<"this is a: "<<a<<endl; 
}



