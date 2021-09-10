#include <chrono>
#include <iostream>
using namespace std::chrono;
using std::cout; using std::endl; 

int main(){
  //1
  milliseconds m(30); 
  m = m * 30; 
  cout<<m.count()<<endl;

  //2
  cout<<m.count() * (double) std::chrono::milliseconds::period::num / std::chrono::milliseconds::period::den<<endl;  //(num for numerator, den for denominator, here this ratio is seconds/milliseconds)

  // 3
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();
  end = std::chrono::system_clock::now();
  std::chrono::duration<double> diff1 = end - start;
  cout<<"elapsed time in seconds: "<<diff1.count()<<endl;

  //4
  std::time_t end_time = std::chrono::system_clock::to_time_t(end); 
  cout<<"to time: "<<end_time<<endl;

  //5
  cout<<std::ctime(&end_time)<<endl; 

  // 6
  cout<<std::chrono::system_clock::period::num<<" | "<<std::chrono::system_clock::period::den<<endl;

  // 7
  cout<<std::chrono::system_clock::is_steady<<endl;

  //8
  using short_min = std::chrono::duration<short, std::ratio<60,1>>;    // a min in short int
  short_min sm(2);
  cout <<"sm count "<<sm.count()<<endl;

  // 9
  std::chrono::milliseconds ms(54802);
  std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(ms);
  cout<<s.count()<<endl;      //see 54

}
