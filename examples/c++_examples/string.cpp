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

int main()
{
     
     // compare();
     find();
     
     // Replace
     // char a[20] = "I do want to change";
     // string str1 = "assholeasshole"; 
     // str1.replace(0, 3, "d");
     // cout<<str1<<endl; 

     // delimit(a); 

     // // get line 
     // char c[20];       // note this is NOT *c[20]
     // cin.getline(c, sizeof(c));     // get a multiword. otherwise, it regular  cin>>c; will just get  the first word. 
     // cout<<*(c+3)<<endl;      // print out the 4th character.
     // cout<<(c[4])<<endl;		//print out the 5th character. 

}



