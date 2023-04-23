import clip
import array
import time

arr_size = 100000
arr = array.array('d', range(arr_size))

start = time.time()
clip.clip(arr, 2, 3, arr)
print(f"time: {time.time() - start}")

start = time.time()
arr = array.array('d', range(arr_size))
for i, r in enumerate(arr):
    if r < 2:
       arr[i] = 2
    elif r > 3:
        arr[i] = 3
    else:
        arr[i] = arr[i]
print(f"time: {time.time() - start}")