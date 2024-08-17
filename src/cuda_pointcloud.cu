#include <laser_merger2/cuda_pointcloud.h>
#include <iostream>
#include <cublas_v2.h>

#define THREADS_PER_BLOCK (1024)

// Error checking macro
#define CHECK_CUBLAS_ERROR(err) { \
    if (err != CUBLAS_STATUS_SUCCESS) { \
        std::cerr << "CUBLAS error: " << err << std::endl; \
        exit(1); \
    } \
}

static cublasHandle_t handle;
static cublasStatus_t status;

void InitGPUDev(void)
{
    status = cublasCreate(&handle);
    CHECK_CUBLAS_ERROR(status);
    printf("CUBLAS OPENED\n");
}

__global__ void TransformArrayLaserLink(float* point, float minAng, float maxAng, int size)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < size)
    {
        float angIncrement = (maxAng - minAng) / size;
        float angle = minAng + idx * angIncrement;
        float dist = point[idx];
        __sincosf(angle, &point[idx + size], &point[idx]); 
        point[idx] = dist * point[idx];
        point[idx + size] = dist * point[idx + size];
        //printf("dist = %f, angle = %f, x = %f, y = %f\n, z = %f, uint = %f\n", dist, angle, point[idx], point[idx + size], point[idx + 2 * size], point[idx + 3 * size]);
    }
}

void InitGpuBuffer(float **dist, float **transformTF, float **outPoint, int size)
{
    cudaMallocManaged(dist, 4 * size * sizeof(float));
    cudaMallocManaged(transformTF, 16 * sizeof(float));
    cudaMallocManaged(outPoint, 4 * size * sizeof(float));
}

void ScanToTransformedPointCloud(float *point, float minAng, float maxAng, int dataSize, float *transformMatrix, float *output)
{
    const float alpha = 1.0, beta = 0.0;
    int M = 4, N = dataSize, K = 4;

    //printf("minAng = %f, maxAng = %f\n", minAng, maxAng);

    int threadsPerBlock = THREADS_PER_BLOCK;
	int blocksPerGrid = (dataSize + threadsPerBlock - 1) / threadsPerBlock;

    TransformArrayLaserLink<<<blocksPerGrid, threadsPerBlock>>>(&point[0], minAng, maxAng, dataSize);
    cudaDeviceSynchronize();

    /*for(int i = 0; i < dataSize; i++)
    {
        printf("x = %f, y = %f, z = %f, unit = %f\n", point[i], point[i + dataSize], point[i + dataSize * 2], point[i + dataSize * 3]);
    }*/

    /*for(int i = 0; i < 16; i++)
    {
        printf("tfMatrix[%d] = %f\n", i, transformMatrix[i]);
    }*/

    cublasSgemm(handle, CUBLAS_OP_N, CUBLAS_OP_T, N, M, K, &alpha, point, N, transformMatrix, K, &beta, output, N);
	cudaDeviceSynchronize();

    /*for(int i = 0; i < dataSize; i++)
    {
        printf("x = %f, y = %f, z = %f, unit = %f\n", output[i], output[i + dataSize], output[i + dataSize * 2], output[i + dataSize * 3]);
    }*/
}