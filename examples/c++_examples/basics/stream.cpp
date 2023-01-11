#include <iostream>
#include <string>
#include <sstream> // std::stringstream
#include <fstream>

using namespace std;

void test_file_stream(){
    fstream file1, file2;
    //| here is bitwise or, so it enables the two flags to be 1. 
    file1.open("sample.txt",ios::out|ios::in);     
    string text,text1;
    
    cout<<"now let's edit this. What is your input?\n";
    getline(cin,text1);			//getline(cin,text); gets a single word. 
    cout<<text1<<endl; 
    file1<<text1<<endl;			// this is writing to the file. no endl 
    file1.close(); 	
}

void test_output_stream(){
    // flush is very expensive. it's done when hitting a buffer size, or endl, or program termination
    std::cout<<"std::endl will flush the output" << std::endl;
}

int main()
{

    return 0;   
}
