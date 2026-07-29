#include "trajectory_recorder/TrajectoryRecorder.hh"

#include <atomic>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

#include <ignition/common/Console.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/image.pb.h>
#include <ignition/msgs/pose.pb.h>

namespace fs = std::filesystem;
using namespace trajectory_recorder;

// ---------------------------------------------------------------------------
// NPY writer — saves raw numpy arrays without Python
// ---------------------------------------------------------------------------

static void WriteNpy(const std::string &path, const void *data,
                     const std::string &dtype, const std::vector<int> &shape,
                     size_t totalBytes)
{
  // Build header dict
  std::ostringstream dictStream;
  dictStream << "{'descr': '" << dtype << "', 'fortran_order': False, 'shape': (";
  for (size_t i = 0; i < shape.size(); ++i)
  {
    dictStream << shape[i];
    if (i + 1 < shape.size()) dictStream << ", ";
  }
  if (shape.size() == 1) dictStream << ",";
  dictStream << "), }";
  std::string dict = dictStream.str();

  // Pad header to be aligned to 64 bytes
  // Header = magic(6) + version(2) + headerLen(2) + dict + '\n'
  size_t prefixLen = 6 + 2 + 2;
  size_t padding = 64 - ((prefixLen + dict.size() + 1) % 64);
  if (padding == 64) padding = 0;
  dict += std::string(padding, ' ');
  dict += '\n';

  uint16_t headerLen = static_cast<uint16_t>(dict.size());

  std::ofstream f(path, std::ios::binary);
  // Magic
  f.write("\x93NUMPY", 6);
  // Version 1.0
  char ver[2] = {1, 0};
  f.write(ver, 2);
  // Header length (little-endian)
  f.write(reinterpret_cast<const char*>(&headerLen), 2);
  // Header dict
  f.write(dict.data(), dict.size());
  // Data
  f.write(reinterpret_cast<const char*>(data), totalBytes);
}

// ---------------------------------------------------------------------------
// Trajectory pose
// ---------------------------------------------------------------------------

struct CameraPose
{
  int frameId;
  gz::math::Pose3d pose;
  double T[4][4];
};

static std::vector<CameraPose> LoadTrajectory(const std::string &path)
{
  std::vector<CameraPose> poses;
  std::ifstream f(path);
  if (!f.is_open())
  {
    ignerr << "[TrajectoryRecorder] Cannot open: " << path << std::endl;
    return poses;
  }

  // Simple JSON array parser — expects array of objects with position + rotation_quat_wxyz
  std::string content((std::istreambuf_iterator<char>(f)),
                       std::istreambuf_iterator<char>());

  // Find each frame_id / position / rotation block
  size_t pos = 0;
  while (true)
  {
    // Find next "frame_id"
    size_t fid_pos = content.find("\"frame_id\"", pos);
    if (fid_pos == std::string::npos) break;

    // Extract frame_id value
    size_t colon = content.find(':', fid_pos);
    int frameId = std::stoi(content.substr(colon + 1));

    // Find position array
    size_t pos_key = content.find("\"position\"", fid_pos);
    size_t bracket = content.find('[', pos_key);
    size_t bracket_end = content.find(']', bracket);
    std::string posStr = content.substr(bracket + 1, bracket_end - bracket - 1);
    std::istringstream posStream(posStr);
    double px, py, pz;
    char comma;
    posStream >> px >> comma >> py >> comma >> pz;

    // Find rotation quaternion [w, x, y, z]
    size_t rot_key = content.find("\"rotation_quat_wxyz\"", fid_pos);
    bracket = content.find('[', rot_key);
    bracket_end = content.find(']', bracket);
    std::string rotStr = content.substr(bracket + 1, bracket_end - bracket - 1);
    std::istringstream rotStream(rotStr);
    double qw, qx, qy, qz;
    rotStream >> qw >> comma >> qx >> comma >> qy >> comma >> qz;

    CameraPose cp;
    cp.frameId = frameId;
    cp.pose = gz::math::Pose3d(px, py, pz, qw, qx, qy, qz);

    // Build T_world_camera
    auto R = gz::math::Matrix3d(cp.pose.Rot());
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        cp.T[r][c] = R(r, c);
    cp.T[0][3] = px; cp.T[1][3] = py; cp.T[2][3] = pz;
    cp.T[3][0] = 0; cp.T[3][1] = 0; cp.T[3][2] = 0; cp.T[3][3] = 1;

    poses.push_back(cp);

    pos = bracket_end;
  }

  return poses;
}

// ---------------------------------------------------------------------------
// Implementation (PIMPL)
// ---------------------------------------------------------------------------

struct TrajectoryRecorder::Impl
{
  std::vector<CameraPose> trajectory;
  std::string outputDir;
  std::string modelName{"trajectory_camera"};
  std::string worldName;
  int settleSteps{100};

  // State
  int currentFrame{0};
  int stepsSinceTeleport{0};
  bool teleported{false};
  bool done{false};
  bool renderReady{false};  // true once first sensor images arrive
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};

  // Camera params (matching trajectory_camera model.sdf)
  int width{320};
  int height{240};
  double hfov_deg{60.0};
  double fx{277.1}, fy{277.1}, cx{160.5}, cy{120.5};

  // gz-transport
  gz::transport::Node node;

  // Thread-safe image buffers
  std::mutex rgbMutex, depthMutex, segMutex;
  std::vector<uint8_t> rgbData;
  std::vector<float> depthData;
  std::vector<uint8_t> segData;
  std::atomic<bool> rgbReady{false};
  std::atomic<bool> depthReady{false};
  std::atomic<bool> segReady{false};

  std::atomic<int> rgbCount{0}, depthCount{0}, segCount{0};

  void OnRgb(const ignition::msgs::Image &_msg)
  {
    std::lock_guard<std::mutex> lock(rgbMutex);
    const auto &d = _msg.data();
    rgbData.assign(d.begin(), d.end());
    rgbReady = true;
    if (rgbCount++ == 0)
      ignmsg << "[TrajectoryRecorder] First RGB received ("
             << _msg.width() << "x" << _msg.height() << ", "
             << d.size() << " bytes)" << std::endl;
  }

  void OnDepth(const ignition::msgs::Image &_msg)
  {
    std::lock_guard<std::mutex> lock(depthMutex);
    const auto &d = _msg.data();
    size_t nFloats = d.size() / sizeof(float);
    depthData.resize(nFloats);
    std::memcpy(depthData.data(), d.data(), d.size());
    depthReady = true;
    if (depthCount++ == 0)
      ignmsg << "[TrajectoryRecorder] First depth received ("
             << _msg.width() << "x" << _msg.height() << ", "
             << d.size() << " bytes)" << std::endl;
  }

  void OnSeg(const ignition::msgs::Image &_msg)
  {
    std::lock_guard<std::mutex> lock(segMutex);
    const auto &d = _msg.data();
    segData.assign(d.begin(), d.end());
    segReady = true;
    if (segCount++ == 0)
      ignmsg << "[TrajectoryRecorder] First seg received ("
             << _msg.width() << "x" << _msg.height() << ", "
             << d.size() << " bytes)" << std::endl;
  }

  void SaveFrame()
  {
    char fname[32];
    std::snprintf(fname, sizeof(fname), "%06d.npy", currentFrame);

    // Save RGB (H, W, 3) uint8
    {
      std::lock_guard<std::mutex> lock(rgbMutex);
      WriteNpy(outputDir + "/color/" + fname, rgbData.data(),
               "|u1", {height, width, 3},
               rgbData.size());
    }

    // Save depth (H, W) float32
    {
      std::lock_guard<std::mutex> lock(depthMutex);
      WriteNpy(outputDir + "/depth/" + fname, depthData.data(),
               "<f4", {height, width},
               depthData.size() * sizeof(float));
    }

    // Save semantic (H, W) int32
    // Seg camera may output L8 (1 byte/pixel) or RGBA (4 bytes/pixel colored_map)
    {
      std::lock_guard<std::mutex> lock(segMutex);
      size_t nPixels = static_cast<size_t>(height) * width;
      std::vector<int32_t> semInt(nPixels);
      size_t bytesPerPixel = segData.size() / nPixels;
      if (bytesPerPixel == 1)
      {
        // L8: direct copy
        for (size_t i = 0; i < nPixels; ++i)
          semInt[i] = static_cast<int32_t>(segData[i]);
      }
      else if (bytesPerPixel >= 3)
      {
        // RGBA/RGB colored_map: use R channel as label ID
        for (size_t i = 0; i < nPixels; ++i)
          semInt[i] = static_cast<int32_t>(segData[i * bytesPerPixel]);
      }
      WriteNpy(outputDir + "/semantic/" + fname, semInt.data(),
               "<i4", {height, width},
               nPixels * sizeof(int32_t));
    }
  }

  void SaveMetadata()
  {
    // camera.json
    {
      std::ofstream f(outputDir + "/camera.json");
      f << "{\n"
        << "  \"width\": " << width << ",\n"
        << "  \"height\": " << height << ",\n"
        << "  \"hfov_deg\": " << hfov_deg << ",\n"
        << "  \"fx\": " << fx << ",\n"
        << "  \"fy\": " << fy << ",\n"
        << "  \"cx\": " << cx << ",\n"
        << "  \"cy\": " << cy << "\n"
        << "}\n";
    }

    // poses.json — copy from trajectory
    {
      std::ofstream f(outputDir + "/poses.json");
      f << "[\n";
      for (size_t i = 0; i < trajectory.size(); ++i)
      {
        const auto &cp = trajectory[i];
        auto p = cp.pose.Pos();
        auto q = cp.pose.Rot();
        f << "  {\n"
          << "    \"frame_id\": " << cp.frameId << ",\n"
          << "    \"position\": [" << std::setprecision(6)
          << p.X() << ", " << p.Y() << ", " << p.Z() << "],\n"
          << "    \"rotation_quat_wxyz\": ["
          << q.W() << ", " << q.X() << ", " << q.Y() << ", " << q.Z() << "],\n"
          << "    \"T_world_camera\": [\n";
        for (int r = 0; r < 4; ++r)
        {
          f << "      [";
          for (int c = 0; c < 4; ++c)
          {
            f << cp.T[r][c];
            if (c < 3) f << ", ";
          }
          f << "]";
          if (r < 3) f << ",";
          f << "\n";
        }
        f << "    ]\n  }";
        if (i + 1 < trajectory.size()) f << ",";
        f << "\n";
      }
      f << "]\n";
    }

    // semantic_class_map.json — flatforest unified scheme
    {
      std::ofstream f(outputDir + "/semantic_class_map.json");
      f << "{\n"
        << "  \"1\": {\"category_name\": \"ground\", \"category_index\": 1},\n"
        << "  \"2\": {\"category_name\": \"deciduous_tree\", \"category_index\": 2},\n"
        << "  \"3\": {\"category_name\": \"conifer\", \"category_index\": 3},\n"
        << "  \"4\": {\"category_name\": \"rock\", \"category_index\": 4},\n"
        << "  \"5\": {\"category_name\": \"undergrowth\", \"category_index\": 5},\n"
        << "  \"6\": {\"category_name\": \"structure\", \"category_index\": 6},\n"
        << "  \"7\": {\"category_name\": \"wall\", \"category_index\": 7},\n"
        << "  \"8\": {\"category_name\": \"human\", \"category_index\": 8}\n"
        << "}\n";
    }
  }
};

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

TrajectoryRecorder::TrajectoryRecorder() : impl_(std::make_unique<Impl>()) {}
TrajectoryRecorder::~TrajectoryRecorder() = default;

// ---------------------------------------------------------------------------
// Configure
// ---------------------------------------------------------------------------

void TrajectoryRecorder::Configure(
    const gz::sim::Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &/*_eventMgr*/)
{
  if (!_sdf->HasElement("trajectory_file"))
  {
    ignerr << "[TrajectoryRecorder] Missing <trajectory_file>.\n";
    return;
  }

  std::string trajFile = _sdf->Get<std::string>("trajectory_file");
  impl_->trajectory = LoadTrajectory(trajFile);
  if (impl_->trajectory.empty())
  {
    ignerr << "[TrajectoryRecorder] Empty trajectory from: " << trajFile << std::endl;
    return;
  }

  if (_sdf->HasElement("output_dir"))
    impl_->outputDir = _sdf->Get<std::string>("output_dir");
  else
    impl_->outputDir = "/tmp/trajectory_recording";

  if (_sdf->HasElement("model_name"))
    impl_->modelName = _sdf->Get<std::string>("model_name");

  if (_sdf->HasElement("settle_steps"))
    impl_->settleSteps = _sdf->Get<int>("settle_steps");

  // Create output directories
  fs::create_directories(impl_->outputDir + "/color");
  fs::create_directories(impl_->outputDir + "/depth");
  fs::create_directories(impl_->outputDir + "/semantic");

  // Subscribe to sensor topics
  // Topics are: /world/<world>/model/<model>/link/camera_link/sensor/...
  // But the sensor <topic> tag makes them: /model/<model>/<topic_name>
  // Gazebo Fortress uses the <topic> tag value directly (not prefixed with model path)
  std::string rgbTopic = "/rgbd_camera/image";
  std::string depthTopic = "/rgbd_camera/depth_image";
  std::string segTopic = "/segmentation_camera/labels_map";

  if (_sdf->HasElement("rgb_topic"))
    rgbTopic = _sdf->Get<std::string>("rgb_topic");
  if (_sdf->HasElement("depth_topic"))
    depthTopic = _sdf->Get<std::string>("depth_topic");
  if (_sdf->HasElement("seg_topic"))
    segTopic = _sdf->Get<std::string>("seg_topic");

  impl_->node.Subscribe(rgbTopic,
      &TrajectoryRecorder::Impl::OnRgb, impl_.get());
  impl_->node.Subscribe(depthTopic,
      &TrajectoryRecorder::Impl::OnDepth, impl_.get());
  impl_->node.Subscribe(segTopic,
      &TrajectoryRecorder::Impl::OnSeg, impl_.get());

  // Save metadata
  impl_->SaveMetadata();

  std::cerr << "[TrajectoryRecorder] Configured: "
            << impl_->trajectory.size() << " frames, "
            << "output=" << impl_->outputDir << ", "
            << "model=" << impl_->modelName << ", "
            << "settle=" << impl_->settleSteps << " steps\n"
            << "  RGB topic: " << rgbTopic << "\n"
            << "  Depth topic: " << depthTopic << "\n"
            << "  Seg topic: " << segTopic << std::endl;
}

// ---------------------------------------------------------------------------
// PreUpdate — teleport camera to current trajectory pose
// ---------------------------------------------------------------------------

void TrajectoryRecorder::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (impl_->done || impl_->trajectory.empty() || _info.paused)
    return;

  // Find the camera model entity on first call
  if (impl_->modelEntity == gz::sim::kNullEntity)
  {
    _ecm.Each<gz::sim::components::Model,
              gz::sim::components::Name>(
        [&](const gz::sim::Entity &entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *name) -> bool
        {
          if (name->Data() == impl_->modelName)
          {
            impl_->modelEntity = entity;
            return false;  // stop
          }
          return true;
        });

    if (impl_->modelEntity == gz::sim::kNullEntity)
      return;  // model not spawned yet

    ignmsg << "[TrajectoryRecorder] Found model '" << impl_->modelName
           << "' (entity " << impl_->modelEntity << ")" << std::endl;
  }

  // Wait for renderer to be ready (first sensor images received)
  if (!impl_->renderReady)
  {
    if (impl_->rgbReady && impl_->depthReady && impl_->segReady)
    {
      impl_->renderReady = true;
      std::cerr << "[TrajectoryRecorder] Renderer ready, starting recording."
                << std::endl;
    }
    return;
  }

  // Get world name on first call (needed for set_pose service). Query
  // only entities tagged with the World component so we don't pick up
  // the first arbitrary named entity.
  if (impl_->worldName.empty())
  {
    _ecm.Each<gz::sim::components::World, gz::sim::components::Name>(
        [&](const gz::sim::Entity &,
            const gz::sim::components::World *,
            const gz::sim::components::Name *name) -> bool
        {
          impl_->worldName = name->Data();
          return false;
        });
    if (!impl_->worldName.empty())
      std::cerr << "[TrajectoryRecorder] World: " << impl_->worldName << std::endl;
  }

  // Move camera to current pose via gz-transport set_pose service
  if (!impl_->teleported)
  {
    const auto &cp = impl_->trajectory[impl_->currentFrame];

    // Build Pose message
    ignition::msgs::Pose req;
    req.set_name(impl_->modelName);
    req.mutable_position()->set_x(cp.pose.Pos().X());
    req.mutable_position()->set_y(cp.pose.Pos().Y());
    req.mutable_position()->set_z(cp.pose.Pos().Z());
    req.mutable_orientation()->set_w(cp.pose.Rot().W());
    req.mutable_orientation()->set_x(cp.pose.Rot().X());
    req.mutable_orientation()->set_y(cp.pose.Rot().Y());
    req.mutable_orientation()->set_z(cp.pose.Rot().Z());

    ignition::msgs::Boolean rep;
    bool result = false;
    std::string service = "/world/" + impl_->worldName + "/set_pose";
    bool executed = impl_->node.Request(service, req, 1000, rep, result);

    if (impl_->currentFrame < 3 || impl_->currentFrame % 100 == 0)
    {
      std::cerr << "[TrajectoryRecorder] set_pose frame " << impl_->currentFrame
                << " pos=(" << cp.pose.Pos().X() << "," << cp.pose.Pos().Y()
                << "," << cp.pose.Pos().Z() << ")"
                << " executed=" << executed << " result=" << result
                << " rep=" << rep.data() << std::endl;
    }

    impl_->teleported = true;
    impl_->stepsSinceTeleport = 0;

    // Clear ready flags to wait for fresh render
    impl_->rgbReady = false;
    impl_->depthReady = false;
    impl_->segReady = false;
  }
}

// ---------------------------------------------------------------------------
// PostUpdate — check if frame is ready, save and advance
// ---------------------------------------------------------------------------

void TrajectoryRecorder::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  if (impl_->done || impl_->trajectory.empty() || _info.paused)
    return;

  if (impl_->modelEntity == gz::sim::kNullEntity)
    return;

  if (!impl_->teleported)
    return;

  impl_->stepsSinceTeleport++;

  // Log actual model pose from ECM after a few settle steps (once per frame, for first 3)
  if (impl_->stepsSinceTeleport == 5 && impl_->currentFrame < 3)
  {
    auto poseComp = _ecm.Component<gz::sim::components::Pose>(impl_->modelEntity);
    if (poseComp)
    {
      auto p = poseComp->Data();
      std::cerr << "[TrajectoryRecorder] ECM pose frame " << impl_->currentFrame
                << ": pos=(" << p.Pos().X() << "," << p.Pos().Y() << "," << p.Pos().Z()
                << ") rot=(" << p.Rot().W() << "," << p.Rot().X() << ","
                << p.Rot().Y() << "," << p.Rot().Z() << ")" << std::endl;
    }
    auto wpComp = _ecm.Component<gz::sim::components::WorldPose>(impl_->modelEntity);
    if (wpComp)
    {
      auto p = wpComp->Data();
      std::cerr << "[TrajectoryRecorder] WorldPose frame " << impl_->currentFrame
                << ": pos=(" << p.Pos().X() << "," << p.Pos().Y() << "," << p.Pos().Z()
                << ") rot=(" << p.Rot().W() << "," << p.Rot().X() << ","
                << p.Rot().Y() << "," << p.Rot().Z() << ")" << std::endl;
    }
  }

  // Wait for settle — gives the renderer time to re-render at the new pose.
  // At real_time_factor=1, 100 physics steps = 1 second = ~30 sensor frames.
  if (impl_->stepsSinceTeleport < impl_->settleSteps)
    return;

  // Wait for all three sensors to publish fresh images after the move
  if (!impl_->rgbReady || !impl_->depthReady || !impl_->segReady)
  {
    if (impl_->stepsSinceTeleport > 2000)
    {
      std::cerr << "[TrajectoryRecorder] Frame " << impl_->currentFrame
                << " timed out (rgb=" << impl_->rgbReady.load()
                << " depth=" << impl_->depthReady.load()
                << " seg=" << impl_->segReady.load() << "), skipping."
                << std::endl;
      impl_->currentFrame++;
      impl_->teleported = false;
      if (impl_->currentFrame >= static_cast<int>(impl_->trajectory.size()))
        impl_->done = true;
    }
    return;
  }

  // Save frame
  impl_->SaveFrame();

  if (impl_->currentFrame % 10 == 0 || impl_->currentFrame == 0)
  {
    std::cerr << "[TrajectoryRecorder] Saved frame " << impl_->currentFrame
              << "/" << impl_->trajectory.size()
              << " (settle waited " << impl_->stepsSinceTeleport << " steps)"
              << std::endl;
  }

  // Advance
  impl_->currentFrame++;
  impl_->teleported = false;

  if (impl_->currentFrame >= static_cast<int>(impl_->trajectory.size()))
  {
    std::cerr << "[TrajectoryRecorder] Recording complete. "
              << impl_->currentFrame << " frames saved to "
              << impl_->outputDir << std::endl;
    impl_->done = true;
  }
}

// ---------------------------------------------------------------------------
// Plugin registration
// ---------------------------------------------------------------------------

IGNITION_ADD_PLUGIN(
    trajectory_recorder::TrajectoryRecorder,
    gz::sim::System,
    trajectory_recorder::TrajectoryRecorder::ISystemConfigure,
    trajectory_recorder::TrajectoryRecorder::ISystemPreUpdate,
    trajectory_recorder::TrajectoryRecorder::ISystemPostUpdate)
