#include <string>
#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include <vector>
#include <sstream>
#include <unordered_map>
#include <unistd.h>
using namespace std;

/**
 *  1. simple way to have two variables of different types in for loop: just take them out. for loop doesn't accept 2 different vars
    2. <string>, stoi(string) -> int
    3. while(getline(std::cin, line, ",")) will get you a line with the delimeters
    4. stringstream is in <sstream>
    5. string.substr(start, len)
 */
void argparser() {
    auto decompose_str_to_deq = [](const std::string& line){
        std::vector<std::string> decomposed_line;
        std::stringstream ss;
        std::string tmp;
        ss << line;
        while(std::getline(ss, tmp, ' ')){
            decomposed_line.emplace_back(tmp);
        }
        return decomposed_line;
    };
    auto get_newtag = [](const std::string& pre_tag, const std::string& tag){
        if (tag[1] == '/'){
            int tag_len = tag.size() - 3;
            return pre_tag.substr(0, pre_tag.size() - tag_len - 1);
        }
        else{
            int tag_len = tag.size() - 1;
            return pre_tag + "."+tag.substr(1, tag_len);
        }
    };
    // simple way to have two variables of different types in for loop
    // <string>, stoi(string) -> int
    std::string line, pre_tag;
    unordered_map<string, string> mp;
    int id = 0;
    int N, Q;
    for (; std::getline(std::cin, line); ++id) {
        auto l =decompose_str_to_deq(line);
        if(id == 0){
            N = stoi(l[0]);
            Q = stoi(l[1]);
        }
        // build a map with all pretags and values
        else if (id <= N){
            // get tag and update pretag
            string new_tag = get_newtag(pre_tag, l[0]);
            if (new_tag.size() > pre_tag.size()){
                for (int i = 1; i < l.size()-2; i += 3){
                    auto full_name = new_tag+"~"+l.at(i);
                    // remove '>'
                    auto value = l.at(i+2).substr(1, l.at(i+2).size()-3);
                    mp[full_name] = value;
                }
            }
            // get attribute and val
            pre_tag = new_tag;
        }
        // queries
        else{
            auto attribute = "." + l[0];
            if (mp.find(attribute) == mp.end()){
                cout<<"Not Found!"<<endl;
            }
            else{
                cout<<mp.at(attribute)<<endl;
            }
        }
    }
}

/**
    1. anonymous structure is NOT part of the standard
    2. :: here really is global variable
    3. Lexical Scoping {}, good for managing RAII resources (mutex locks), etc. 
    4. When you see INSTANTIATION A::B ... A 只可能是namespace， 而不会是parent。
*/
int global_var; 
void test_scopeing(){
    struct {
        void operator() (){
            int global_var = 100; 
            ::global_var = 3; 
            cout<<"global var: "<<::global_var<<endl;
        }
    }Foo;

    Foo();
}

/**
    1. namespace: can be defined only in public scope
        - You might be wondering what's the difference from using. Using is for datatypes, not for namespaces.
*/
namespace Hoo{
    namespace Loo{
        class Foo
        {};
    }
}
void test_namespace(){
    namespace ublas = Hoo::Loo;
    ublas::Foo v;
}

void test_printing(){
    while (1){
        // 100;100 means line 100, cln 100, though cln 100 doesn't seem to be effective. 2J is to clear the screen by moving the content to scrollback buffer, 3J is to clear the buffer
        std::cout<<__FUNCTION__<<"\e[100;100H\e[2J\e[3J"<<std::endl;
        std::cout<<__FUNCTION__<<": 1"<<std::endl;
        usleep(30000);
    }
    
}

int main()
{
    // test_scopeing();
    test_printing();
}
