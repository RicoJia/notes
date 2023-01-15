#include <memory>
#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>

using std::cout; using std::endl;
using namespace std::chrono_literals;

template<class F, class... Args>
auto enqueue(std::queue< std::function<void()> >& tasks, std::mutex& queue_mutex, std::condition_variable& condition, F&& f, Args&&... args) 
    -> std::future<typename std::result_of<F(Args...)>::type>
{
    using return_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared< std::packaged_task<return_type()> >(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        
    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex);

        tasks.emplace([task](){ (*task)(); });
    }
    condition.notify_one();
    return res;
}

int main()
{
    std::queue< std::function<void()> > tasks;
    std::mutex queue_mutex;
    std::condition_variable condition; 
    std::thread th(
        [&tasks, &queue_mutex, &condition]
        { for(;;)
            {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(queue_mutex);
                    // fairly low CPU usage here. 
                    condition.wait(lock,
                        [&tasks]{return !tasks.empty();});
                    task = std::move(tasks.front());
                    tasks.pop();
                }
                task();
            }
        }
    );

    cout <<"start task"<<endl;
    int num_threads = 3; 
    std::vector<double> results(num_threads);
    std::vector<double> vec {1,2,3,4,5,6};
    for(unsigned int id = 0; id < num_threads; id++){
        auto func = [&results, &vec, id, num_threads]{
          // cout<<"size: "<<vec.size()<<endl;  
          results.at(id) = vec.at(id); 
        };
    }
    

    for (auto i: results) cout<<i<<endl;    // you need to wait first
    th.join(); 
    return 0;
}

