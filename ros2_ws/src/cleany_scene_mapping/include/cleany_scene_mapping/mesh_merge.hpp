// SPDX-License-Identifier: BSD-3-Clause
// The merge semantics follow geometric_shapes 2.3.4, Copyright 2011 Willow Garage, Inc.
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its contributors may
//   be used to endorse or promote products derived from this software without
//   specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <geometric_shapes/shapes.h>
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace cleany_scene_mapping
{
namespace mesh_merge_detail
{
using Cell = std::array<std::int64_t, 3>;
struct CellHash
{
  std::size_t operator()(const Cell& cell) const noexcept
  {
    std::size_t result = 0;
    for (auto coordinate : cell)
      result ^= std::hash<std::int64_t>{}(coordinate) + 0x9e3779b9U + (result << 6) + (result >> 2);
    return result;
  }
};
}  // namespace mesh_merge_detail

// Spatial lookup accelerates the *same* ordered merge performed by upstream.
// In particular, a later representative may overwrite an earlier assignment;
// nearest-neighbour/first-match clustering would change the output mesh.
// Only the OctoMap self-filter copy uses this helper. Planning collision meshes
// and the 0.1 mm upstream threshold are unchanged.
inline void mergeMeshVertices(shapes::Mesh& mesh, double threshold)
{
  using namespace mesh_merge_detail;
  const double squared = threshold * threshold;
  const double width = 2.0 * std::abs(threshold);
  if (mesh.vertex_count < 64 || !std::isfinite(squared) || squared == 0.0 ||
      !std::isfinite(width))
  {
    mesh.mergeVertices(threshold);
    return;
  }

  std::vector<Cell> cells(mesh.vertex_count);
  for (unsigned int index = 0; index < mesh.vertex_count; ++index)
    for (unsigned int axis = 0; axis < 3; ++axis)
    {
      const double coordinate = mesh.vertices[3 * index + axis];
      const long double scaled = static_cast<long double>(coordinate) / width;
      // Keep ample integer/rounding headroom. Unusual/nonfinite input retains
      // upstream behaviour rather than overflowing a grid coordinate.
      if (!std::isfinite(scaled) || std::abs(scaled) > 0x1p40L)
      {
        mesh.mergeVertices(threshold);
        return;
      }
      cells[index][axis] = static_cast<std::int64_t>(std::floor(scaled));
    }

  std::unordered_map<Cell, std::vector<unsigned int>, CellHash> buckets;
  buckets.reserve(mesh.vertex_count);
  for (unsigned int index = 0; index < mesh.vertex_count; ++index)
    buckets[cells[index]].push_back(index);

  std::vector<unsigned int> mapping(mesh.vertex_count), representatives;
  std::iota(mapping.begin(), mapping.end(), 0U);
  representatives.reserve(mesh.vertex_count);
  for (unsigned int first = 0; first < mesh.vertex_count; ++first)
  {
    if (mapping[first] != first) continue;
    mapping[first] = static_cast<unsigned int>(representatives.size());
    representatives.push_back(first);
    const Eigen::Vector3d a(mesh.vertices[3 * first], mesh.vertices[3 * first + 1],
                            mesh.vertices[3 * first + 2]);
    const auto cell = cells[first];
    // Cell width is twice the radius: all possible matches are in these cells,
    // with a full radius of margin against floating-point division roundoff.
    for (int x = -1; x <= 1; ++x)
      for (int y = -1; y <= 1; ++y)
        for (int z = -1; z <= 1; ++z)
        {
          const auto found = buckets.find({cell[0] + x, cell[1] + y, cell[2] + z});
          if (found == buckets.end()) continue;
          for (auto second : found->second)
          {
            if (second <= first) continue;
            const Eigen::Vector3d b(mesh.vertices[3 * second], mesh.vertices[3 * second + 1],
                                    mesh.vertices[3 * second + 2]);
            if ((a - b).squaredNorm() <= squared) mapping[second] = mapping[first];
          }
        }
  }
  if (representatives.size() == mesh.vertex_count) return;

  auto vertices = std::make_unique<double[]>(3 * representatives.size());
  for (std::size_t index = 0; index < representatives.size(); ++index)
    for (unsigned int axis = 0; axis < 3; ++axis)
      vertices[3 * index + axis] = mesh.vertices[3 * representatives[index] + axis];
  for (std::size_t index = 0; index < 3ULL * mesh.triangle_count; ++index)
    mesh.triangles[index] = mapping[mesh.triangles[index]];
  delete[] mesh.vertices;
  mesh.vertices = vertices.release();
  mesh.vertex_count = static_cast<unsigned int>(representatives.size());
  if (mesh.triangle_normals) mesh.computeTriangleNormals();
  if (mesh.vertex_normals) mesh.computeVertexNormals();
}
}  // namespace cleany_scene_mapping
