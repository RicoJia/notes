#include <iostream>
#include <math.h>
#include "jpeg_reader.hpp"
#include "histogram_counter.cuh"
#include <cuda_runtime.h>

// function to add the elements of two arrays
// CUDA Kernel function to add the elements of two arrays on the GPU
// __global__ functions are "kernels", running on device. 
__global__ void cuda_add(int n, float *x, float *y)
{
  // This is the same operation done on each thread.
  // for (int i = 0; i < n; i++)
  //     y[i] = x[i] + y[i];
  // stride thru its index
  unsigned int stride = blockDim.x; 
  for(unsigned int i = threadIdx.x; i < n; i += stride){
      y[i] = x[i] + y[i];
  }

}

void add(int N, float x_val, float y_val){
  // float *x = new float[N];
  // float *y = new float[N];
  float *x, *y;
  cudaMallocManaged(&x, N*sizeof(float));
  cudaMallocManaged(&y, N*sizeof(float));

  // initialize x and y arrays on the host
  for (int i = 0; i < N; i++) {
    x[i] = x_val;
    y[i] = y_val;
  }

  // Run kernel on 1M elements on the CPU
  cuda_add<<<1,8>>>(N, x, y);   
  //doing the same computation once per thread, <<<1, num_threads>>>
  // num_threads should be a power of 2, and it will be blockDim.x. if not power of 2, no errors, but not as efficient

  // Wait for GPU to finish before accessing on host, cuz CUDA doesn't block CPU threads
  cudaDeviceSynchronize();

  // Check for errors (all values should be 3.0f)
  float maxError = 0.0f;
  for (int i = 0; i < N; i++)
    maxError = fmax(maxError, fabs(y[i]-3.0f));
  std::cout << "Max error: " << maxError << std::endl;

  // Free memory
  // delete [] x;
  // delete [] y;
  cudaFree(x);
  cudaFree(y);
}

