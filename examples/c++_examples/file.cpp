#include <iostream>
#include <string>
#include <sstream> // std::stringstream
#include <fstream>

using namespace std;

int main()
{
fstream file1, file2;
file1.open("sample.txt",ios::out|ios::in);     //| here is bitwise or, so it enables the two flags to be 1. 
string text,text1;
 
cout<<"now let's edit this. What is your input?\n";
getline(cin,text1);			//getline(cin,text); gets a single word. 
cout<<text1<<endl; 
file1<<text1<<endl;			// this is writing to the file. no endl 
file1.close(); 	

/*
cout<<"before editing, the sample txt reads: \n";

while(getline(file1, text))			
{cout<<text<<endl;}

  				//reading from file;
if (file1.is_open()){
cout<<"now let's edit this. What is your input?\n";
getline(cin,text1);			//getline(cin,text); gets a single word. 
cout<<text1<<endl; 
file1<<text1<<endl;			// this is writing to the file. no endl 
file1.close(); 	
			// HERE getline hits the bottom already. 
}
else cout<<"unable to open the file";  */

return 0;   
}
