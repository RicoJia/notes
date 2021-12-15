template <typename T>
class Foo
{
public:
  Foo (T t){}

private:
  /* data */
};

class Bar: public Foo<int>
{
public:
  Bar (): Foo(1){}

private:
  /* data */
};

int main()
{
    Bar bar();  
}
