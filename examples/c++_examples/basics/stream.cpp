#include <iostream>
#include <string>
#include <sstream> // std::stringstream
#include <fstream>
#include <array>

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
    // 1. flush is very expensive. it's done when hitting a buffer size, or endl, or program termination
    std::cout<<"std::endl will flush the output" << std::endl;
    // 2. formating string is tricky (< C++ 20)
    std::array<char, 32> buffer = {};
    size_t n = 20;
    // %zu is for size_t, added in c99
    auto n_written = std::snprintf(buffer.data(), buffer.size(), "%020zu.log", n);
    std::cout<<std::string{buffer.data()}<<"\n";
}

int main()
{
    test_output_stream();
    return 0;   
}
