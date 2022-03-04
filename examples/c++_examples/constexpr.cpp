struct Point{
    constexpr Point(double x, double y) noexcept : x(x), y(y) {}      //OK, as long as args are constexpr variables/ const (can be double as well?) / literals
    constexpr double getX() const noexcept {return x;}        
    double x;
    double y;
    // because ctor is constexpr, we may declare a constexpr object. 
    // Then, we might be able to get X as constexpr as well! 
};

constexpr Point returnPoint(const Point& p1){
    return p1; 
}

int main(){
    const double i = 1.0;
    constexpr Point p1(i, 2.0);      //We can have constexpr because of the constexpr ctor
    Point p2 =returnPoint(p1); 
    Point p3 =returnPoint(p2); 
}


// Const variable
// int main()
// {
//
//   const double& j = 4.5;
//   constexpr double k = j;
// }
