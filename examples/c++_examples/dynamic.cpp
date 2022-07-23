#include <iostream>

using namespace std;
/**
* 1. Create array:
*   - 2d: must do this: TYPE (*p)[N] = new TYPE [][N];
*   - N must be given. Cannot do TYPE **p
* 2. Delete:
*   - Do not delete arrays that are not created by new
*   - You gotta delete arrays one by one, using delete[]
*/
void test_raw_new(){
    int length = 3;
    int *marks = new int[3];			// remember this syntax here. 

    for (int j = 0;j<length;j++)
    {
        cout<<"score for "<<j<<endl;
        cin>>*(marks+j);
    }

    cout<<"-------------------\n";
    for (int j = 0;j<length;j++){cout<<marks[1]<<endl; }
    delete[] marks;

    // double **arr = new double[5][6];
    double *arr[3] = new double[5][3];
}

void test_2d_arr(){
    // Dimensions of the array
    int m = 3, n = 4, c = 0;

    // Declare memory block of size M
    int** a = new int*[m];
    //Delete the array created
     for(int i=0;i<m;i++)    //To delete the inner arrays
         delete [] a[i];
     delete [] a;              //To delete the outer array
}

int main()
{
    test_2d_arr();
}
