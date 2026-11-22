#include "octomap_cuda.hpp"
#include <stdexcept>
#include <string>

void cuda_check(cudaError_t status)
{
  if(status!=cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}
template<class T> T* allocate(size_t count)
{
  T* p=nullptr; cuda_check(cudaMalloc(&p, count*sizeof(T))); return p;
}
template<class T> T* upload(const std::vector<T>& v)
{
  T* p=allocate<T>(v.size());
  const auto status=cudaMemcpy(p,v.data(),v.size()*sizeof(T),cudaMemcpyHostToDevice);
  if(status!=cudaSuccess) cudaFree(p);
  cuda_check(status);
  return p;
}
struct SceneGpu {
  const Face* faces; const Part* parts; const int* cells; const float4* boxes;
  int nfaces,nparts,nlinks,capacity; Grid grid;
  Pose* poses; int* result;
};
SceneGpu* load_gpu(const std::vector<Face>& faces, const std::vector<Part>& parts,
                  const Grid& grid, const std::vector<int>& cells,
                  const std::vector<float4>& boxes, int capacity, int links)
{
  auto* scene=new SceneGpu{};
  try {
    scene->nfaces=faces.size();
    scene->nparts=parts.size();
    scene->nlinks=links;
    scene->capacity=capacity;
    scene->grid=grid;
    scene->faces=upload(faces);
    scene->parts=upload(parts);
    update_gpu_map(scene,grid,cells,boxes);
    scene->poses=allocate<Pose>(capacity*parts.size());
    scene->result=allocate<int>(capacity*links);
    return scene;
  } catch (...) {
    free_gpu(scene);
    throw;
  }
}
void update_gpu_map(SceneGpu* scene,const Grid& grid,const std::vector<int>& cells,
                    const std::vector<float4>& boxes)
{
  auto* new_cells=upload(cells);
  float4* new_boxes=nullptr;
  try {
    new_boxes=upload(boxes);
  } catch (...) {
    cudaFree(new_cells);
    throw;
  }
  cudaFree(const_cast<int*>(scene->cells));
  cudaFree(const_cast<float4*>(scene->boxes));
  scene->cells=new_cells;
  scene->boxes=new_boxes;
  scene->grid=grid;
}
void free_gpu(SceneGpu* scene)
{
  cudaFree(const_cast<Face*>(scene->faces)); cudaFree(const_cast<Part*>(scene->parts));
  cudaFree(const_cast<int*>(scene->cells)); cudaFree(const_cast<float4*>(scene->boxes));
  cudaFree(scene->poses); cudaFree(scene->result); delete scene;
}

__device__ float3 add(float3 a,float3 b){return make_float3(a.x+b.x,a.y+b.y,a.z+b.z);}
__device__ float3 sub(float3 a,float3 b){return make_float3(a.x-b.x,a.y-b.y,a.z-b.z);}
__device__ float3 scale(float3 a,float s){return make_float3(a.x*s,a.y*s,a.z*s);}
__device__ float3 minimum(float3 a,float3 b){return make_float3(fminf(a.x,b.x),fminf(a.y,b.y),fminf(a.z,b.z));}
__device__ float3 maximum(float3 a,float3 b){return make_float3(fmaxf(a.x,b.x),fmaxf(a.y,b.y),fmaxf(a.z,b.z));}
__device__ float dot(float3 a,float3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
__device__ float3 cross(float3 a,float3 b){return make_float3(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x);}
__device__ float3 tf(const Pose& p,float3 v){
  const float* r=p.r;
  return make_float3(r[0]*v.x+r[1]*v.y+r[2]*v.z+r[3],
                     r[4]*v.x+r[5]*v.y+r[6]*v.z+r[7],
                     r[8]*v.x+r[9]*v.y+r[10]*v.z+r[11]);
}
__device__ float3 inverse(const Pose& p,float3 v){
  const float* r=p.r; v=sub(v,make_float3(r[3],r[7],r[11]));
  return make_float3(r[0]*v.x+r[4]*v.y+r[8]*v.z,
                     r[1]*v.x+r[5]*v.y+r[9]*v.z,
                     r[2]*v.x+r[6]*v.y+r[10]*v.z);
}
__device__ int3 quantize(const Grid& g,float3 a)
{
  a=scale(sub(a,g.origin),1.f/g.step);
  return make_int3(__float2int_rd(a.x),__float2int_rd(a.y),__float2int_rd(a.z));
}
__device__ int3 clamp_lower(int3 a){return make_int3(max(0,a.x),max(0,a.y),max(0,a.z));}
__device__ int3 clamp_upper(int3 a,int3 n){return make_int3(min(n.x-1,a.x),min(n.y-1,a.y),min(n.z-1,a.z));}
__device__ bool separated(float3 axis,float3 a,float3 b,float3 c,float half){
  const float x=dot(axis,a),y=dot(axis,b),z=dot(axis,c);
  const float reach=half*(fabsf(axis.x)+fabsf(axis.y)+fabsf(axis.z));
  return fminf(x,fminf(y,z))>reach || fmaxf(x,fmaxf(y,z))<-reach;
}
__device__ bool triangle_box(float3 a,float3 b,float3 c,float4 box)
{
  const auto center=make_float3(box.x,box.y,box.z);
  a=sub(a,center); b=sub(b,center); c=sub(c,center);
  const float half=box.w+1.e-7f;
  const auto low=minimum(a,minimum(b,c)),high=maximum(a,maximum(b,c));
  if(low.x>half || low.y>half || low.z>half || high.x<-half || high.y<-half || high.z<-half) return false;
  const float3 edges[3]={sub(b,a),sub(c,b),sub(a,c)};
  if(separated(cross(edges[0],edges[1]),a,b,c,half)) return false;
  for(int i=0;i<3;++i){
    auto e=edges[i];
    if(separated(make_float3(0,e.z,-e.y),a,b,c,half) ||
       separated(make_float3(-e.z,0,e.x),a,b,c,half) ||
       separated(make_float3(e.y,-e.x,0),a,b,c,half)) return false;
  }
  return true;
}
// One CUDA block per collision part and robot pose. Filter by the transformed
// part AABB before any thread computes its world-space triangle coordinates.
__global__ void intersect_faces(SceneGpu s,int rows,bool links)
{
  const int row=blockIdx.x/s.nparts, shape=blockIdx.x%s.nparts;
  const Part& part=s.parts[shape];
  int* hit=s.result+(links ? row*s.nlinks+part.link : row);
  __shared__ int active;
  const Pose& pose=s.poses[blockIdx.x];
  if(threadIdx.x==0){
    active=0;
    if(!*hit){
      const auto center=tf(pose,scale(add(part.low,part.high),.5f));
      const auto ext=scale(sub(part.high,part.low),.5f);
      const float* r=pose.r;
      const auto reach=make_float3(fabsf(r[0])*ext.x+fabsf(r[1])*ext.y+fabsf(r[2])*ext.z,
                                  fabsf(r[4])*ext.x+fabsf(r[5])*ext.y+fabsf(r[6])*ext.z,
                                  fabsf(r[8])*ext.x+fabsf(r[9])*ext.y+fabsf(r[10])*ext.z);
      const auto e=make_float3(1.e-6f,1.e-6f,1.e-6f);
      const auto lo=clamp_lower(quantize(s.grid,sub(center,add(reach,e))));
      const auto hi=clamp_upper(quantize(s.grid,add(center,add(reach,e))),s.grid.size);
      active=lo.x<=hi.x && lo.y<=hi.y && lo.z<=hi.z;
    }
  }
  __syncthreads();
  if(!active) return;
  const auto down=make_float3(-1.e-6f,-1.e-6f,-1.e-6f);
  for(int fid=part.face_begin+threadIdx.x;fid<part.face_end;fid+=blockDim.x){
    if(*hit) return;
    const auto& f=s.faces[fid];
    const auto a=tf(pose,f.a),b=tf(pose,f.b),c=tf(pose,f.c);
    const auto lo=clamp_lower(quantize(s.grid,add(minimum(a,minimum(b,c)),down)));
    const auto hi=clamp_upper(quantize(s.grid,sub(maximum(a,maximum(b,c)),down)),s.grid.size);
    for(int z=lo.z;z<=hi.z;++z) for(int y=lo.y;y<=hi.y;++y) for(int x=lo.x;x<=hi.x;++x){
      const int cell=s.cells[(z*s.grid.size.y+y)*s.grid.size.x+x];
      if(cell>=0 && triangle_box(a,b,c,s.boxes[cell])){atomicExch(hit,1);return;}
    }
  }
}
__global__ void test_containment(SceneGpu s,int rows,bool links)
{
  const int i=blockIdx.x*blockDim.x+threadIdx.x;
  if(i>=s.nparts*rows) return;
  const int row=i/s.nparts,shape=i%s.nparts;
  const Part& part=s.parts[shape];
  if(!part.solid) return;
  int* hit=s.result+(links ? row*s.nlinks+part.link : row);
  if(*hit) return;
  const Pose& pose=s.poses[i];
  const auto center=tf(pose,scale(add(part.low,part.high),.5f));
  const auto ext=scale(sub(part.high,part.low),.5f);
  const float* r=pose.r;
  const auto reach=make_float3(fabsf(r[0])*ext.x+fabsf(r[1])*ext.y+fabsf(r[2])*ext.z,
                              fabsf(r[4])*ext.x+fabsf(r[5])*ext.y+fabsf(r[6])*ext.z,
                              fabsf(r[8])*ext.x+fabsf(r[9])*ext.y+fabsf(r[10])*ext.z);
  const auto lo=clamp_lower(quantize(s.grid,sub(center,reach)));
  const auto hi=clamp_upper(quantize(s.grid,add(center,reach)),s.grid.size);
  for(int z=lo.z;z<=hi.z;++z) for(int y=lo.y;y<=hi.y;++y) for(int x=lo.x;x<=hi.x;++x){
    const int cell=s.cells[(z*s.grid.size.y+y)*s.grid.size.x+x];
    if(cell<0) continue;
    const auto box=s.boxes[cell];
    const auto p=inverse(pose,make_float3(box.x,box.y,box.z));
    if(p.x<part.low.x || p.y<part.low.y || p.z<part.low.z ||
       p.x>part.high.x || p.y>part.high.y || p.z>part.high.z) continue;
    bool inside=true;
    for(int fid=part.face_begin;fid<part.face_end;++fid){
      const auto& f=s.faces[fid];
      const auto normal=cross(sub(f.b,f.a),sub(f.c,f.a));
      if(dot(normal,sub(p,f.a))>0.f){inside=false;break;}
    }
    if(inside){atomicExch(hit,1);return;}
  }
}
std::vector<int> query_gpu(SceneGpu* s,const Pose* poses,int rows,bool links)
{
  if(rows<1 || rows>s->capacity) throw std::runtime_error("Invalid CUDA batch size");
  const size_t count=rows*(links?s->nlinks:1);
  cuda_check(cudaMemcpy(s->poses,poses,rows*s->nparts*sizeof(Pose),cudaMemcpyHostToDevice));
  cuda_check(cudaMemset(s->result,0,count*sizeof(int)));
  intersect_faces<<<s->nparts*rows,64>>>(*s,rows,links);
  test_containment<<<(s->nparts*rows+127)/128,128>>>(*s,rows,links);
  cuda_check(cudaGetLastError());
  std::vector<int> result(count);
  cuda_check(cudaMemcpy(result.data(),s->result,count*sizeof(int),cudaMemcpyDeviceToHost));
  return result;
}
