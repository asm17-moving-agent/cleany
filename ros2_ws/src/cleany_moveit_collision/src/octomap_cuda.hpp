#pragma once
#include <cuda_runtime.h>
#include <vector>

struct Face { float3 a,b,c; int shape; };
struct Part { float3 low,high; int face_begin,face_end,solid,link; };
struct Pose { float r[12]; };
struct Grid { int3 size; float3 origin; float step; };
struct SceneGpu;

SceneGpu* load_gpu(const std::vector<Face>& faces, const std::vector<Part>& parts,
                  const Grid& grid, const std::vector<int>& cells,
                  const std::vector<float4>& boxes, int capacity, int links);
void update_gpu_map(SceneGpu* scene, const Grid& grid, const std::vector<int>& cells,
                    const std::vector<float4>& boxes);
std::vector<int> query_gpu(SceneGpu* scene, const Pose* poses, int rows, bool links);
void free_gpu(SceneGpu* scene);
