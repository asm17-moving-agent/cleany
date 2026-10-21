#pragma once

// Included inside geometric_shapes' bodies.cpp by the pinned source overlay.
// Cache only the immutable MeshData produced by the original, verified Qhull
// path. Scale, padding, pose, live world geometry and collision results are not
// cached. The overlay supplies a digest of the source archive and all patches.
#include <geometric_shapes/shapes.h>
#include <openssl/sha.h>
#include <Eigen/Core>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

#ifndef CLEANY_GEOMETRY_CACHE_BUILD_ID
#error "Verified mesh cache requires a source/version fingerprint"
#endif

namespace cleany_scene_mapping
{
namespace mesh_cache_detail
{
using Bytes = std::vector<unsigned char>;
using Digest = std::array<unsigned char, SHA256_DIGEST_LENGTH>;
inline Digest digest(const Bytes& data)
{
  Digest result{};
  if (!SHA256(data.data(), data.size(), result.data())) throw std::runtime_error("SHA-256 unavailable");
  return result;
}
inline void append(Bytes& data, const void* source, std::size_t count)
{
  if (count == 0) return;
  const auto* bytes = static_cast<const unsigned char*>(source);
  data.insert(data.end(), bytes, bytes + count);
}
template <typename T> inline void append(Bytes& data, T value)
{
  append(data, &value, sizeof(value));
}
inline std::string hex(const Digest& digest)
{
  std::string result;
  for (auto byte : digest)
  {
    result += "0123456789abcdef"[byte >> 4];
    result += "0123456789abcdef"[byte & 15];
  }
  return result;
}
class Reader
{
public:
  explicit Reader(const Bytes& bytes) : bytes_(bytes) {}
  template <typename T> T read()
  {
    if (sizeof(T) > remaining()) throw std::runtime_error("Truncated mesh cache");
    T value;
    std::memcpy(&value, bytes_.data() + offset_, sizeof(T));
    offset_ += sizeof(T);
    return value;
  }
  std::size_t count(std::size_t element_size)
  {
    const auto value = read<std::uint64_t>();
    if (value > remaining() / element_size) throw std::runtime_error("Invalid mesh cache size");
    return value;
  }
  template <typename V> void vector(V& value)
  {
    for (Eigen::Index axis = 0; axis < value.size(); ++axis)
    {
      value[axis] = read<double>();
      if (!std::isfinite(value[axis])) throw std::runtime_error("Nonfinite mesh cache");
    }
  }
  std::size_t remaining() const { return bytes_.size() - offset_; }
private:
  const Bytes& bytes_;
  std::size_t offset_ = 0;
};
template <typename V> inline void appendVector(Bytes& bytes, const V& value)
{
  for (Eigen::Index axis = 0; axis < value.size(); ++axis) append(bytes, double(value[axis]));
}
template <typename Map> inline void appendMap(Bytes& bytes, const Map& values)
{
  append(bytes, std::uint64_t(values.size()));
  for (const auto& [key, value] : values)
  {
    append(bytes, std::uint32_t(key));
    append(bytes, std::uint32_t(value));
  }
}
template <typename Map> inline void readMap(Reader& reader, Map& values)
{
  const auto count = reader.count(2 * sizeof(std::uint32_t));
  for (std::size_t index = 0; index < count; ++index)
  {
    const auto key = reader.read<std::uint32_t>();
    const auto value = reader.read<std::uint32_t>();
    if (!values.emplace(key, value).second) throw std::runtime_error("Duplicate mesh cache key");
  }
}
}  // namespace mesh_cache_detail

class VerifiedMeshCache
{
public:
  VerifiedMeshCache(const shapes::Mesh& mesh, const char* qhull_version) noexcept
  {
    try
    {
      const auto* directory = std::getenv("CLEANY_MOVEIT_GEOMETRY_CACHE_DIR");
      if (!directory || !*directory || mesh.vertex_count == 0 || !mesh.vertices) return;
      if (3ULL * mesh.vertex_count * sizeof(double) > maximum_bytes) return;
      for (std::size_t index = 0; index < 3ULL * mesh.vertex_count; ++index)
        if (!std::isfinite(mesh.vertices[index])) return;
      using namespace mesh_cache_detail;
      Bytes input;
      const std::string version = std::string(CLEANY_GEOMETRY_CACHE_BUILD_ID) + "\n" + qhull_version + "\n" + __VERSION__;
      append(input, version.data(), version.size());
      append(input, std::uint32_t(0x01020304));  // Byte order is part of the key.
      append(input, std::uint32_t(mesh.vertex_count));
      append(input, mesh.vertices, 3ULL * mesh.vertex_count * sizeof(double));
      key_ = digest(input);
      directory_ = directory;
      path_ = directory_ / (hex(key_) + ".bin");
    }
    catch (...) { path_.clear(); }
  }

  template <typename Data> bool load(Data& data) const noexcept
  {
    if (path_.empty()) return false;
    try
    {
      using namespace mesh_cache_detail;
      std::ifstream file(path_, std::ios::binary | std::ios::ate);
      if (!file) return false;
      const auto size = file.tellg();
      if (size < static_cast<std::streamoff>(2 * key_.size()) ||
          size > static_cast<std::streamoff>(maximum_bytes)) return false;
      file.seekg(0);
      Digest expected_key, checksum;
      file.read(reinterpret_cast<char*>(expected_key.data()), expected_key.size());
      file.read(reinterpret_cast<char*>(checksum.data()), checksum.size());
      Bytes payload(static_cast<std::size_t>(size) - 2 * key_.size());
      file.read(reinterpret_cast<char*>(payload.data()), payload.size());
      if (!file || file.peek() != EOF || expected_key != key_ || digest(payload) != checksum) return false;
      Reader reader(payload);
      Data candidate;
      candidate.bounding_cylinder_.pose.setIdentity();
      candidate.vertices_.resize(reader.count(3 * sizeof(double)));
      for (auto& vertex : candidate.vertices_) reader.vector(vertex);
      candidate.planes_.resize(reader.count(4 * sizeof(double)));
      for (auto& plane : candidate.planes_) reader.vector(plane);
      candidate.triangles_.resize(reader.count(sizeof(std::uint32_t)));
      for (auto& index : candidate.triangles_) index = reader.read<std::uint32_t>();
      readMap(reader, candidate.plane_for_triangle_);
      readMap(reader, candidate.triangle_for_plane_);
      reader.vector(candidate.mesh_center_);
      candidate.mesh_radiusB_ = reader.read<double>();
      reader.vector(candidate.box_offset_);
      reader.vector(candidate.box_size_);
      candidate.bounding_cylinder_.radius = reader.read<double>();
      candidate.bounding_cylinder_.length = reader.read<double>();
      // Its pose is derived from the box at query time; the source does not
      // initialize or use MeshData's bounding-cylinder pose.
      if (reader.remaining() || !valid(candidate)) return false;
      data = std::move(candidate);
      return true;
    }
    catch (...) { return false; }
  }

  template <typename Data> void store(const Data& data) const noexcept
  {
    if (path_.empty()) return;
    try
    {
      if (!valid(data)) return;
      using namespace mesh_cache_detail;
      Bytes payload;
      append(payload, std::uint64_t(data.vertices_.size()));
      for (const auto& vertex : data.vertices_) appendVector(payload, vertex);
      append(payload, std::uint64_t(data.planes_.size()));
      for (const auto& plane : data.planes_) appendVector(payload, plane);
      append(payload, std::uint64_t(data.triangles_.size()));
      for (auto index : data.triangles_) append(payload, std::uint32_t(index));
      appendMap(payload, data.plane_for_triangle_);
      appendMap(payload, data.triangle_for_plane_);
      appendVector(payload, data.mesh_center_);
      append(payload, data.mesh_radiusB_);
      appendVector(payload, data.box_offset_);
      appendVector(payload, data.box_size_);
      append(payload, data.bounding_cylinder_.radius);
      append(payload, data.bounding_cylinder_.length);
      if (payload.size() > maximum_bytes - 2 * key_.size()) return;
      const auto checksum = digest(payload);
      Bytes bytes;
      bytes.reserve(2 * key_.size() + payload.size());
      append(bytes, key_.data(), key_.size());
      append(bytes, checksum.data(), checksum.size());
      append(bytes, payload.data(), payload.size());
      atomicWrite(bytes);
    }
    catch (...) {}  // A cache failure never replaces the verified computation.
  }

private:
  template <typename Data> static bool valid(const Data& data)
  {
    if (data.vertices_.empty() || data.planes_.empty() || data.triangles_.empty() ||
        data.triangles_.size() % 3 || !data.mesh_center_.allFinite() ||
        !data.box_offset_.allFinite() || !data.box_size_.allFinite() ||
        (data.box_size_.array() < 0).any()) return false;
    for (double number : {data.mesh_radiusB_, data.bounding_cylinder_.radius, data.bounding_cylinder_.length})
      if (!std::isfinite(number) || number < 0) return false;
    for (const auto& vertex : data.vertices_) if (!vertex.allFinite()) return false;
    for (const auto& plane : data.planes_) if (!plane.allFinite()) return false;
    for (auto index : data.triangles_) if (index >= data.vertices_.size()) return false;
    const auto triangles = data.triangles_.size() / 3;
    if (data.plane_for_triangle_.size() != triangles || data.triangle_for_plane_.size() != data.planes_.size())
      return false;
    for (const auto& [triangle, plane] : data.plane_for_triangle_)
      if (triangle >= triangles || plane >= data.planes_.size()) return false;
    for (const auto& [plane, triangle] : data.triangle_for_plane_)
      if (plane >= data.planes_.size() || triangle >= triangles) return false;
    return true;
  }

  void atomicWrite(const mesh_cache_detail::Bytes& bytes) const
  {
    std::filesystem::create_directories(directory_);
    static std::atomic<unsigned long> sequence{0};
    const auto temporary = path_.string() + ".tmp-" + std::to_string(getpid()) + "-" + std::to_string(sequence++);
    const int descriptor = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (descriptor < 0) return;
    bool complete = true;
    std::size_t offset = 0;
    while (offset < bytes.size())
    {
      const auto count = ::write(descriptor, bytes.data() + offset, bytes.size() - offset);
      if (count <= 0) { complete = false; break; }
      offset += count;
    }
    if (::close(descriptor) != 0) complete = false;
    std::error_code error;
    if (complete) std::filesystem::rename(temporary, path_, error);
    if (!complete || error) std::filesystem::remove(temporary, error);
  }

  static constexpr std::size_t maximum_bytes = 64 * 1024 * 1024;
  mesh_cache_detail::Digest key_{};
  std::filesystem::path directory_, path_;
};
}  // namespace cleany_scene_mapping
