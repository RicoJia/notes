#include <iostream>
#include <string>
#include <sstream>
#include <vector>

using namespace std;
/*string path = "/a/./b/../../c/";
int main()
{
 stringstream ss(path);     //you initialize your ss like this. initializer
        string res, temp;
        vector<string>vec;
        while(getline(ss,temp,'/')){
	    if(temp.empty()||temp==".")	continue;
            if(temp==".."&&!vec.empty()) vec.pop_back();       // pay attention to the the two ifs here. don't swap them. otherwise, the .. will be executed despite the if!
	    else vec.push_back(temp);}

        for(auto a:vec) {
 	a.empty()?cout<<'/'<<endl:cout<<'/'<<a; }
}	*/

stringstream a("980int");
int main()
{
    string type;
    int num;
    a>>num>>type;
    cout<<a.str();

    //clear
    a.clear();  // this modifies some state flags? 
    a.str("")
    cout<<a.str();
    return 0;
}
