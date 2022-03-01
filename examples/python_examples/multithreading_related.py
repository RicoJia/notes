import concurrent.futures

def foo(i): 
    print(i)

with concurrent.futures.ThreadPoolExecutor() as executor: 
    futures = []
    for i in range(5): 
        futures.append(executor.submit(foo, i=i))
    for future in futures: 
        future.result()

