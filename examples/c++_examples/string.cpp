#include <iostream>
#include <sstream>
#include <string>
#include<bits/stdc++.h> 
#include <vector>

using namespace std;


void compare (){
    string str1 = "asdf", str2 = "ascf"; 
    cout<<(str1>str2)<<endl;
    cout<<str1.compare(str2)<<endl;

    auto str3 = string(str1.begin(), str1.begin()+2); 
    cout<<str3;
}

void find(){
    string str1 = "asshole"; 
    cout<<str1.find("hole")<<endl;   // should see 3
    cout<<str1.find("hole", 4)<<endl;   // should see std::npos
    cout<<str1.find("x")<<endl; // string::npos
}

void delimit(const string& str){
    std::stringstream ss; 
    ss<<str; 
    std::string str2; 
    vector<string> ret; 
    while (std::getline(ss, str2, ' '))
        ret.push_back(str2); 

    for (auto s: ret)
        cout<<s<<endl;
}

void str_to_longlong(){
    std::string str = "12345";
    // stoll is in <string>
    cout<<stoll(str)<<"\n";
}

int main()
{
    str_to_longlong();
}



