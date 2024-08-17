#ifndef __CUDA_POINT_CLOUD_H__
#define __CUDA_POINT_CLOUD_H__

void InitGPUDev(void);
void InitGpuBuffer(float **dist, float **transformTF, float **outPoint, int size);
void ScanToTransformedPointCloud(float *point, float minAng, float maxAng, int dataSize, float *transformMatrix, float *output);
#endif  // __CUDA_POINT_CLOUD_H__