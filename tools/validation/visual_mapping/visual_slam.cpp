// Offline RTAB-Map visual odometry and pose graph. No LiDAR or wheel input.
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/StereoCameraModel.h>

namespace fs = std::filesystem;
using namespace rtabmap;

void writePose(std::ostream &out, const Transform &pose) {
  for (int j = 0; j < 12; ++j)
    out << ',' << pose.data()[j];
}

// A lost visual trajectory can leave disconnected graphs. Preserve their
// independent gauges.
int exportComponents(const fs::path &out) {
  std::unique_ptr<DBDriver> db(DBDriver::create());
  if (!db->openConnection((out / "map.db").string(), false, true))
    throw std::runtime_error("Cannot read RTAB-Map database");
  std::set<int> ids;
  db->getAllNodeIds(ids, false, false, false);
  std::map<int, Transform> all;
  std::multimap<int, Link> links;
  db->getAllLinks(links, true, false);
  for (const int id : ids) {
    int group, weight;
    Transform pose, gt;
    std::string label;
    double t;
    std::vector<float> speed;
    GPS gps;
    EnvSensors env;
    if (db->getNodeInfo(id, pose, group, weight, label, t, gt, speed, gps,
                        env) &&
        weight != -9 && !pose.isNull())
      all.emplace(id, pose);
  }
  db->closeConnection(false);
  std::unique_ptr<Optimizer> opt(Optimizer::create(
      Optimizer::kTypeGTSAM, {{"Optimizer/Iterations", "40"}}));
  std::set<int> visited;
  std::ofstream output(out / "all-components.csv");
  output << "id,r00,r01,r02,x,r10,r11,r12,y,r20,r21,r22,z\n"
         << std::setprecision(10);
  for (const auto &start : all) {
    if (visited.count(start.first))
      continue;
    std::map<int, Transform> poses;
    std::multimap<int, Link> edges;
    opt->getConnectedGraph(start.first, all, links, poses, edges);
    if (poses.empty())
      poses.emplace(start);
    auto refined =
        poses.size() > 1 ? opt->optimize(start.first, poses, edges) : poses;
    if (refined.size() != poses.size())
      throw std::runtime_error("Incomplete component optimization");
    for (const auto &n : refined) {
      visited.insert(n.first);
      output << n.first;
      writePose(output, n.second);
      output << '\n';
    }
  }
  return visited.empty();
}

int main(int argc, char **argv) {
  if (argc == 3 && std::string(argv[1]) == "--components")
    return exportComponents(argv[2]);
  if (argc != 10) {
    std::cerr << "visual_slam input output rgbd|stereo fx fy cx cy baseline "
                 "keyframe_interval_s\n";
    return 2;
  }
  const fs::path input(argv[1]), out(argv[2]);
  const bool suppressProjector = std::string(argv[3]) == "stereo-filtered";
  const bool disparityMode = std::string(argv[3]) == "stereo-depth";
  const bool stereo =
      disparityMode || suppressProjector || std::string(argv[3]) == "stereo";
  const double fx = std::stod(argv[4]), fy = std::stod(argv[5]),
               cx = std::stod(argv[6]), cy = std::stod(argv[7]);
  const double keyframe = std::stod(argv[9]);
  fs::create_directories(out);
  if (fs::exists(out / "map.db"))
    throw std::runtime_error("Output map.db already exists");
  cv::setNumThreads(1);
  const auto kernel =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
  const auto equalizer = cv::createCLAHE(2.0, cv::Size(8, 8));
  auto optical = CameraModel::opticalRotation();
  if (disparityMode) {
    std::ifstream matrix(out / "base-from-camera.txt");
    cv::Mat values(3, 4, CV_32F);
    for (int i = 0; i < 12; ++i)
      if (!(matrix >> values.ptr<float>()[i]))
        throw std::runtime_error("Missing fixed camera transform");
    optical = Transform(values);
  }
  const double baseline = std::stod(argv[8]);
  auto matcher = cv::StereoSGBM::create(0, 48, 5, 8 * 25, 32 * 25, 1, 31, 10,
                                        50, 2, cv::StereoSGBM::MODE_SGBM);
  const auto opening =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
  CameraModel rgbdModel("rgbd", fx, fy, cx, cy, optical, 0, cv::Size(640, 480));
  StereoCameraModel stereoModel("infra", fx, fy, cx, cy, std::stod(argv[8]),
                                optical, cv::Size(640, 480));
  ParametersMap params{{"Odom/Strategy", "0"},
                       {"Vis/FeatureType", "8"},
                       {"Kp/DetectorStrategy", "8"},
                       {"Vis/MinInliers", "15"},
                       {"Vis/MaxFeatures", "900"},
                       {"Kp/MaxFeatures", "500"},
                       {"Mem/GenerateIds", "false"},
                       {"RGBD/LinearUpdate", "0.06"},
                       {"RGBD/AngularUpdate", "0.06"},
                       {"Rtabmap/TimeThr", "0"},
                       {"Grid/FromDepth", "false"},
                       {"RGBD/CreateOccupancyGrid", "false"},
                       {"Mem/STMSize", "15"},
                       {"Rtabmap/WorkingDirectory", out.string()}};
  if (disparityMode) {
    params["Reg/Force3DoF"] = "true";
    params["Vis/FeatureType"] = "1";
    params["Kp/DetectorStrategy"] = "1";
    params["Vis/MaxFeatures"] = "650";
  }
  auto odom = std::unique_ptr<Odometry>(Odometry::create(params));
  Rtabmap mapper;
  mapper.init(params, (out / "map.db").string());
  std::ofstream observations(out / "tracking.csv"),
      nodes(out / "keyframes.csv");
  observations << "id,time_ns,segment,tracked,inliers\n";
  nodes << "id,time_ns,segment,loop_id\n";
  std::ofstream dense(out / "visual-odometry.csv");
  dense << "id,time_ns,segment,r00,r01,r02,x,r10,r11,r12,y,r20,r21,r22,z\n"
        << std::setprecision(9);
  std::string stamp;
  int id = 0, segment = 0, failures = 0;
  double lastKey = -1e12;
  while (std::cin >> stamp) {
    const double ts = std::stoll(stamp) * 1e-9;
    const auto a = input / (stereo ? "left" : "rgb") / (stamp + ".jpg");
    const auto b = input / (stereo ? "right" : "depth") /
                   (stamp + (stereo ? ".jpg" : ".png"));
    cv::Mat first = cv::imread(a.string(), stereo ? cv::IMREAD_GRAYSCALE
                                                  : cv::IMREAD_COLOR);
    cv::Mat second = cv::imread(b.string(), stereo ? cv::IMREAD_GRAYSCALE
                                                   : cv::IMREAD_UNCHANGED);
    if (first.empty() || second.empty())
      throw std::runtime_error("bad sample " + stamp);
    if (disparityMode) {
      cv::Mat left, right, disparity;
      cv::pyrDown(first, left);
      cv::pyrDown(second, right);
      matcher->compute(left, right, disparity);
      cv::Mat depth = cv::Mat::zeros(disparity.size(), CV_16UC1);
      for (int row = 0; row < depth.rows; ++row)
        for (int col = 0; col < depth.cols; ++col) {
          const double d = disparity.at<short>(row, col) / 16.0;
          if (d <= 0)
            continue;
          const double mm = 1000 * (fx * .5) * baseline / d;
          if (mm >= 350 && mm <= 6000)
            depth.at<unsigned short>(row, col) =
                cv::saturate_cast<unsigned short>(mm);
        }
      cv::morphologyEx(left, left, cv::MORPH_OPEN, opening);
      cv::GaussianBlur(left, left, cv::Size(5, 5), 1.3);
      equalizer->apply(left, left);
      cv::resize(left, first, first.size(), 0, 0, cv::INTER_LINEAR);
      second = depth;
    } else if (suppressProjector) {
      for (auto *image : {&first, &second}) {
        cv::morphologyEx(*image, *image, cv::MORPH_OPEN, kernel);
        cv::GaussianBlur(*image, *image, cv::Size(5, 5), 1.0);
        equalizer->apply(*image, *image);
      }
    }
    ++id;
    SensorData data = stereo && !disparityMode
                          ? SensorData(first, second, stereoModel, id, ts)
                          : SensorData(first, second, rgbdModel, id, ts);
    OdometryInfo quality;
    auto pose = odom->process(data, &quality);
    observations << id << ',' << stamp << ',' << segment << ','
                 << !pose.isNull() << ',' << quality.reg.inliers << '\n';
    if (pose.isNull()) {
      if (++failures >= 12) {
        mapper.triggerNewMap();
        odom->reset();
        ++segment;
        failures = 0;
        lastKey = -1e12;
      }
      continue;
    }
    failures = 0;
    dense << id << ',' << stamp << ',' << segment;
    writePose(dense, pose);
    dense << '\n';
    if (ts - lastKey >= keyframe) {
      // Separate keyframe extraction from odometry features and their word IDs.
      SensorData frame = stereo && !disparityMode
                             ? SensorData(first, second, stereoModel, id, ts)
                             : SensorData(first, second, rgbdModel, id, ts);
      if (mapper.process(frame, pose, quality.reg.covariance)) {
        nodes << id << ',' << stamp << ',' << segment << ','
              << mapper.getLoopClosureId() << '\n';
      }
      lastKey = ts;
    }
    if (id % 100 == 0) {
      observations.flush();
      nodes.flush();
    }
  }
  std::map<int, Transform> poses;
  std::multimap<int, Link> constraints;
  mapper.getGraph(poses, constraints, true, true);
  std::ofstream graph(out / "optimized.csv"), edges(out / "edges.csv");
  graph << "id,r00,r01,r02,x,r10,r11,r12,y,r20,r21,r22,z\n"
        << std::setprecision(10);
  for (const auto &entry : poses) {
    graph << entry.first;
    writePose(graph, entry.second);
    graph << '\n';
  }
  edges << "from_id,to_id,type,r00,r01,r02,x,r10,r11,r12,y,r20,r21,r22,z\n"
        << std::setprecision(10);
  for (const auto &entry : constraints) {
    const auto &link = entry.second;
    edges << link.from() << ',' << link.to() << ',' << link.type();
    writePose(edges, link.transform());
    edges << '\n';
  }
  mapper.close();
  std::cout << id << " frames, " << poses.size() << " optimized poses\n";
  return poses.empty();
}
