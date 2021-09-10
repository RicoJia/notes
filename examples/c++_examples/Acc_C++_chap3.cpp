//
// Created by rico on 5/22/19.
//
//

#include<iostream>
#include<vector>
#include<algorithm>
#include<iomanip>
#include <string>
#include <ios>

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::setprecision;
using std::streamsize;
using std::vector;

int quarters();
int chap3();
int different_words();
int word_length();

int main()
{
    different_words();
    return 0;
}

int word_length()
{
    cout<<"OK,enter your words now: "<<endl;
    string input;
    typedef string::size_type str_size;
    vector<str_size>length_list;
    while (cin>>input)
    {
        length_list.push_back(input.size());
    }
    typedef vector<str_size >::size_type vectorsz;
    vectorsz size = length_list.size();
    if (size==0)
    {
        cout<<"Sorry, Empty Input"<<endl;
        return 1;
    }
    cout<<endl<<"**************OUTPUT****************"<<endl;
    vectorsz max_index = 0,min_index = 0;
    for(vectorsz i=0; i!=size;++i)
    {
        if(length_list[max_index]<length_list[i]) max_index = i;
        if(length_list[min_index]>length_list[i]) min_index = i;
    }
    cout<<"The max length is "<<length_list[max_index]<<" ,and min length is "<<length_list[min_index]<<endl;
    vectorsz sz;
    cout<< "sz is "<<sz<<endl;
}
int different_words()
{
    cout<<"OK,now enter your words:"<<endl;
    vector<string> word_list;
    string input;
    while(cin>>input)
    {
        word_list.push_back(input);
    }

    sort(word_list.begin(),word_list.end());
    typedef vector<string>::size_type vectorsz;
    vectorsz size = word_list.size();
    if(size==0)
    {
        cout<<"Sorry, empty inputs"<<endl;
        return 1;
    }

    vectorsz index = 0;
    vectorsz appearance = 1;
    cout<<endl<<"**************OUTPUT****************"<<endl;
    while(index!=size-1)
    {
        if(word_list[index]==word_list[index+1])
        {
            ++appearance;
        }
        else
        {
            cout<<word_list[index]<<" count: "<<appearance<<endl;
            appearance = 1;
        }
        ++index;
    }
     cout<<word_list[size-1]<<" count: "<<appearance<<endl;
}
int quarters()
{
cout<<"OK, now enter your numbers"<<endl;
double input = 0;
vector<double>v;
while(cin>>input)
{
    v.push_back(input);
}
typedef vector<double>::size_type vectorsz;
vectorsz size = v.size();
if(size<4)
{
    cout<<"Sorry, not enough input"<<endl;
    return 1;
}
cout<<endl<<"**************OUTPUT****************"<<endl;
sort(v.begin(),v.end());
vectorsz step_size = size/4;
vectorsz size_remainder = size%4;
int index = 0;
for (int i =0;i!=4;++i)
{
    for(int j=0;j!=step_size;++j)
    {
        cout<<v[index+j]<<" ";
        ++index;
    }
    /*if(i==3&&size_remainder!=0)
    {
        for (int j=1;j!=size_remainder+1;++j)
        {
            cout<<v[3*step_size+j]<<" ";
        }
    }*/
    if(i<size_remainder) {
        cout<<v[index];
        ++index;
    }
    cout << endl;
}
return 0;
}

int chap3()
{
    cout<<"Hello, What is your name?"<<endl;
    string name;
    cin>>name;
    cout<<"OK "<<name<<", what are your scores? "
                       "You can enter your scores one by one"<<endl;
    double input = 0;
    vector<double> v;
    while(cin>>input)
    {
        v.push_back(input);
    }
    typedef vector<double>::size_type vectorsz;
    vectorsz size = v.size();
    if (size == 0)
    {
        cout<<"Sorry, this is an empty input."<<endl;
        return 1;
    }
    sort(v.begin(),v.end());
    double median = 0;
    median=(size%2==0)?(v[size/2]+v[size/2-1])/2:v[(size-1)/2];
    streamsize prec = cout.precision();
    cout<<"The median is "<<setprecision(3)<<median<<setprecision(prec)<<endl;
    return 0;
}
