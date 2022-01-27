from collections import deque

# deque is a simple data structure, while Queue.queue is a thread-safe datastructure
q = deque(maxlen=3)
q.append('a')
q.append('b')
q.append('c')
q.append('d')
print(q)    #see only b, c, d
print(q.popleft())  #see b
print(q[-1])
