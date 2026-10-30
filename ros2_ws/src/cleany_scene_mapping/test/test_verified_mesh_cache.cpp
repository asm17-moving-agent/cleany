// Exercise the actual patched geometric_shapes DSO, including cache failures.
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/mesh_operations.h>
#include <gtest/gtest.h>
#include <openssl/sha.h>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <optional>
#include <random>
#include <unistd.h>

namespace
{
class VerifiedMeshCache : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!std::getenv("CLEANY_TEST_GEOMETRY_CACHE"))
      GTEST_SKIP() << "Use make test-moveit-preparation with the geometry overlay";
    if (const auto* value = std::getenv("CLEANY_MOVEIT_GEOMETRY_CACHE_DIR")) previous = value;
    auto pattern = (std::filesystem::temp_directory_path() / "cleany-verified-mesh-XXXXXX").string();
    const auto* created = mkdtemp(pattern.data());
    ASSERT_NE(created, nullptr);
    directory = created;
    setenv("CLEANY_MOVEIT_GEOMETRY_CACHE_DIR", directory.c_str(), 1);
    shape.reset(shapes::createMeshFromShape(shapes::Box(.14, .07, .32)));
  }
  void TearDown() override
  {
    if (directory.empty()) return;
    if (previous) setenv("CLEANY_MOVEIT_GEOMETRY_CACHE_DIR", previous->c_str(), 1);
    else unsetenv("CLEANY_MOVEIT_GEOMETRY_CACHE_DIR");
    std::error_code error;
    std::filesystem::remove_all(directory, error);
  }
  std::unique_ptr<bodies::ConvexMesh> make(double padding = .015, double scale = 1.1)
  {
    auto body = std::make_unique<bodies::ConvexMesh>();
    body->setDimensionsDirty(shape.get()); body->setScaleDirty(scale); body->setPaddingDirty(padding);
    body->updateInternalData();
    return body;
  }
  std::vector<std::filesystem::path> files() const
  {
    std::vector<std::filesystem::path> result;
    for (const auto& entry : std::filesystem::directory_iterator(directory))
      if (entry.path().extension() == ".bin") result.push_back(entry.path());
    return result;
  }
  static std::vector<unsigned char> read(const std::filesystem::path& path)
  {
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), {}};
  }
  static void write(const std::filesystem::path& path, const std::vector<unsigned char>& bytes)
  {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
  }
  static void same(const bodies::ConvexMesh& a, const bodies::ConvexMesh& b)
  {
    ASSERT_EQ(a.getVertices().size(), b.getVertices().size());
    ASSERT_EQ(a.getPlanes().size(), b.getPlanes().size());
    EXPECT_EQ(a.getTriangles(), b.getTriangles());
    for (std::size_t i = 0; i < a.getVertices().size(); ++i)
    {
      EXPECT_TRUE((a.getVertices()[i].array() == b.getVertices()[i].array()).all());
      EXPECT_TRUE((a.getScaledVertices()[i].array() == b.getScaledVertices()[i].array()).all());
    }
    for (std::size_t i = 0; i < a.getPlanes().size(); ++i)
      EXPECT_TRUE((a.getPlanes()[i].array() == b.getPlanes()[i].array()).all());
    EXPECT_DOUBLE_EQ(a.computeVolume(), b.computeVolume());
    bodies::BoundingSphere sa, sb; a.computeBoundingSphere(sa); b.computeBoundingSphere(sb);
    EXPECT_DOUBLE_EQ(sa.radius, sb.radius); EXPECT_TRUE((sa.center.array() == sb.center.array()).all());
    bodies::BoundingCylinder ca, cb; a.computeBoundingCylinder(ca); b.computeBoundingCylinder(cb);
    EXPECT_DOUBLE_EQ(ca.radius, cb.radius); EXPECT_DOUBLE_EQ(ca.length, cb.length);
    EXPECT_TRUE((ca.pose.matrix().array() == cb.pose.matrix().array()).all());
    bodies::AABB ba, bb; a.computeBoundingBox(ba); b.computeBoundingBox(bb);
    EXPECT_TRUE((ba.min().array() == bb.min().array()).all());
    EXPECT_TRUE((ba.max().array() == bb.max().array()).all());
    std::mt19937 random(3419);
    std::uniform_real_distribution<double> coordinate(-.22, .22);
    for (int i = 0; i < 1000; ++i)
    {
      const Eigen::Vector3d point(coordinate(random), coordinate(random), coordinate(random));
      EXPECT_EQ(a.containsPoint(point), b.containsPoint(point));
      EXPECT_EQ(a.intersectsRay(point, Eigen::Vector3d::UnitX()), b.intersectsRay(point, Eigen::Vector3d::UnitX()));
    }
  }
  std::filesystem::path directory;
  std::optional<std::string> previous;
  std::unique_ptr<shapes::Mesh> shape;
};
}

TEST_F(VerifiedMeshCache, HitPreservesGeometryScalePaddingAndPose)
{
  auto fresh = make();
  ASSERT_EQ(files().size(), 1U);
  const auto timestamp = std::filesystem::last_write_time(files().front());
  auto cached = make();
  same(*fresh, *cached);
  EXPECT_EQ(timestamp, std::filesystem::last_write_time(files().front()));
  for (const auto& parameters : {std::pair<double, double>{0, 1}, {.01, .8}, {.03, 1.2}})
  {
    auto reference = fresh->cloneAt(Eigen::Isometry3d::Identity(), parameters.first, parameters.second);
    auto changed = make(parameters.first, parameters.second);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translate(Eigen::Vector3d(.3, -.8, 1.1));
    pose.rotate(Eigen::AngleAxisd(.7, Eigen::Vector3d(1, 2, 3).normalized()));
    reference->setPose(pose); changed->setPose(pose);
    same(dynamic_cast<const bodies::ConvexMesh&>(*reference), *changed);
  }
  EXPECT_EQ(files().size(), 1U);  // Pose/scale/padding are applied after cache lookup.
}

TEST_F(VerifiedMeshCache, CorruptTruncatedAndMalformedFilesRebuild)
{
  auto reference = make();
  ASSERT_EQ(files().size(), 1U);
  const auto path = files().front();
  const auto original = read(path);
  ASSERT_GT(original.size(), 80U);
  for (int fault = 0; fault < 4; ++fault)
  {
    auto bytes = original;
    if (fault == 0) bytes[75] ^= 0x55;  // Payload checksum failure.
    if (fault == 1) bytes.resize(17);   // Incomplete write.
    if (fault == 2) bytes[0] ^= 1;      // Another input's cache under this filename.
    if (fault == 3)
    {
      // Even an internally consistent digest cannot authorize an oversized array.
      std::fill(bytes.begin() + 64, bytes.begin() + 72, 255);
      SHA256(bytes.data() + 64, bytes.size() - 64, bytes.data() + 32);
    }
    write(path, bytes);
    auto result = make(); same(*reference, *result);
    EXPECT_EQ(read(path), original);
  }
}

TEST_F(VerifiedMeshCache, InputChangesAndCacheFailuresDoNotReuseWrongGeometry)
{
  auto first = make();
  ASSERT_EQ(files().size(), 1U);
  for (unsigned int index = 0; index < shape->vertex_count; ++index) shape->vertices[3 * index] *= 1.2;
  auto modified = make();
  EXPECT_EQ(files().size(), 2U);
  bodies::AABB original_bounds, changed_bounds;
  first->computeBoundingBox(original_bounds); modified->computeBoundingBox(changed_bounds);
  EXPECT_NE(original_bounds.max().x(), changed_bounds.max().x());
  const auto invalid_directory = directory / "regular-file";
  write(invalid_directory, {1, 2, 3});
  setenv("CLEANY_MOVEIT_GEOMETRY_CACHE_DIR", invalid_directory.c_str(), 1);
  auto fallback = make(); same(*modified, *fallback);
  setenv("CLEANY_MOVEIT_GEOMETRY_CACHE_DIR", "", 1);
  auto disabled = make(); same(*modified, *disabled);
}

TEST_F(VerifiedMeshCache, ConcurrentWritersPublishOnlyCompleteFiles)
{
  setenv("CLEANY_MOVEIT_GEOMETRY_CACHE_DIR", "", 1);
  auto reference = make();
  setenv("CLEANY_MOVEIT_GEOMETRY_CACHE_DIR", directory.c_str(), 1);
  std::vector<std::future<std::unique_ptr<bodies::ConvexMesh>>> jobs;
  for (int i = 0; i < 8; ++i) jobs.emplace_back(std::async(std::launch::async, [&] { return make(); }));
  for (auto& job : jobs) same(*reference, *job.get());
  ASSERT_EQ(files().size(), 1U);
  auto final = make(); same(*reference, *final);
  EXPECT_EQ(std::distance(std::filesystem::directory_iterator(directory), std::filesystem::directory_iterator()), 1);
}
