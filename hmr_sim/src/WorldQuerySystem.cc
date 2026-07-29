#include "world_query_system/WorldQuerySystem.hh"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/SemanticLabel.hh>
#include <gz/sim/components/Visual.hh>

#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/common/SystemPaths.hh>

#include <ignition/common/Console.hh>

using namespace world_query_system;

// ---------------------------------------------------------------------------
// Collision shape representation extracted from the ECM
// ---------------------------------------------------------------------------

struct CollisionShape
{
  sdf::GeometryType type;
  gz::math::Pose3d worldPose;  // world-frame pose of the collision
  gz::math::Vector3d size;     // box: half-extents, cylinder: {radius,radius,halfLen}
  double radius{0};
  double halfLength{0};
  int semanticLabel{0};
  std::string entityName;

  // Mesh-specific: world-frame triangles + AABB for fast rejection
  struct Triangle { gz::math::Vector3d v0, v1, v2; };
  std::vector<Triangle> triangles;
  gz::math::AxisAlignedBox aabb;
};

// ---------------------------------------------------------------------------
// Helpers: point-in-shape tests (in shape-local frame)
// ---------------------------------------------------------------------------

/// Test if a LOCAL-frame point is inside a box with given half-extents.
static bool PointInBox(const gz::math::Vector3d &_local,
                       const gz::math::Vector3d &_halfSize,
                       double _margin)
{
  return std::abs(_local.X()) <= (_halfSize.X() + _margin) &&
         std::abs(_local.Y()) <= (_halfSize.Y() + _margin) &&
         std::abs(_local.Z()) <= (_halfSize.Z() + _margin);
}

/// Test if a LOCAL-frame point is inside a cylinder (Z-axis aligned).
static bool PointInCylinder(const gz::math::Vector3d &_local,
                            double _radius, double _halfLength,
                            double _margin)
{
  double r2 = _local.X() * _local.X() + _local.Y() * _local.Y();
  return r2 <= (_radius + _margin) * (_radius + _margin) &&
         std::abs(_local.Z()) <= (_halfLength + _margin);
}

/// Test if a LOCAL-frame point is inside a sphere.
static bool PointInSphere(const gz::math::Vector3d &_local,
                          double _radius, double _margin)
{
  return _local.Length() <= (_radius + _margin);
}

/// Test if a point is within _margin of a Z=0 plane (infinite in XY).
static bool PointNearPlane(const gz::math::Vector3d &_local, double _margin)
{
  return std::abs(_local.Z()) <= _margin;
}

/// Compute distance from a point to a triangle in 3D.
/// Returns the minimum distance from _pt to the triangle (_v0, _v1, _v2).
static double PointTriangleDist(const gz::math::Vector3d &_pt,
                                const gz::math::Vector3d &_v0,
                                const gz::math::Vector3d &_v1,
                                const gz::math::Vector3d &_v2)
{
  // Edge vectors
  gz::math::Vector3d e0 = _v1 - _v0;
  gz::math::Vector3d e1 = _v2 - _v0;
  gz::math::Vector3d v  = _v0 - _pt;

  double a = e0.Dot(e0);
  double b = e0.Dot(e1);
  double c = e1.Dot(e1);
  double d = e0.Dot(v);
  double e = e1.Dot(v);

  double det = a * c - b * b;
  double s = b * e - c * d;
  double t = b * d - a * e;

  // Clamp to triangle using Voronoi regions
  if (s + t <= det)
  {
    if (s < 0) { if (t < 0) { /* region 4 */ if (d < 0) { t = 0; s = (-d >= a) ? 1 : -d/a; } else { s = 0; t = (e >= 0) ? 0 : ((-e >= c) ? 1 : -e/c); } } else { /* region 3 */ s = 0; t = (e >= 0) ? 0 : ((-e >= c) ? 1 : -e/c); } }
    else if (t < 0) { /* region 5 */ t = 0; s = (d >= 0) ? 0 : ((-d >= a) ? 1 : -d/a); }
    else { /* region 0 */ double inv = 1.0 / det; s *= inv; t *= inv; }
  }
  else
  {
    if (s < 0) { /* region 2 */ double tmp0 = b + d, tmp1 = c + e; if (tmp1 > tmp0) { double n = tmp1 - tmp0, dd = a - 2*b + c; s = (n >= dd) ? 1 : n/dd; t = 1 - s; } else { s = 0; t = (tmp1 <= 0) ? 1 : ((e >= 0) ? 0 : -e/c); } }
    else if (t < 0) { /* region 6 */ double tmp0 = b + e, tmp1 = a + d; if (tmp1 > tmp0) { double n = tmp1 - tmp0, dd = a - 2*b + c; t = (n >= dd) ? 1 : n/dd; s = 1 - t; } else { t = 0; s = (tmp1 <= 0) ? 1 : ((d >= 0) ? 0 : -d/a); } }
    else { /* region 1 */ double n = (c + e) - (b + d), dd = a - 2*b + c; if (n <= 0) { s = 0; } else { s = (n >= dd) ? 1 : n/dd; } t = 1 - s; }
  }

  gz::math::Vector3d closest = _v0 + e0 * s + e1 * t;
  return (closest - _pt).Length();
}

/// Test if a world-frame point is near any triangle in a mesh shape.
static bool PointNearMesh(const gz::math::Vector3d &_pt,
                          const CollisionShape &_shape,
                          double _margin,
                          double &_dist)
{
  // Fast AABB rejection: expand AABB by margin
  gz::math::AxisAlignedBox expanded(
      _shape.aabb.Min() - gz::math::Vector3d(_margin, _margin, _margin),
      _shape.aabb.Max() + gz::math::Vector3d(_margin, _margin, _margin));
  if (!expanded.Contains(_pt))
    return false;

  // Check each triangle
  double minDist = std::numeric_limits<double>::max();
  for (const auto &tri : _shape.triangles)
  {
    double d = PointTriangleDist(_pt, tri.v0, tri.v1, tri.v2);
    if (d < minDist)
      minDist = d;
    if (minDist <= _margin)
    {
      _dist = minDist;
      return true;  // early exit
    }
  }
  _dist = minDist;
  return minDist <= _margin;
}

/// Resolve a model:// URI to an absolute file path using
/// IGN_GAZEBO_RESOURCE_PATH (colon-delimited).
static std::string ResolveModelUri(const std::string &_uri)
{
  const std::string prefix = "model://";
  if (_uri.find(prefix) != 0)
    return _uri;

  std::string relPath = _uri.substr(prefix.size());

  // Search IGN_GAZEBO_RESOURCE_PATH
  const char *envVar = std::getenv("IGN_GAZEBO_RESOURCE_PATH");
  if (!envVar)
    envVar = std::getenv("GZ_SIM_RESOURCE_PATH");

  if (envVar)
  {
    std::istringstream ss(envVar);
    std::string dir;
    while (std::getline(ss, dir, ':'))
    {
      if (dir.empty()) continue;
      std::string candidate = dir + "/" + relPath;
      std::ifstream test(candidate);
      if (test.good())
        return candidate;
    }
  }

  // Also try ~/.gazebo/models
  const char *home = std::getenv("HOME");
  if (home)
  {
    std::string candidate = std::string(home) + "/.gazebo/models/" + relPath;
    std::ifstream test(candidate);
    if (test.good())
      return candidate;
  }

  return _uri;  // return unchanged if not found
}

/// Load a mesh from an SDF URI, transform triangles to world frame.
static bool LoadMeshTriangles(
    const sdf::Mesh *_sdfMesh,
    const gz::math::Pose3d &_worldPose,
    CollisionShape &_shape)
{
  std::string uri = _sdfMesh->Uri();

  // Resolve model:// URI to absolute path
  std::string loadPath = ResolveModelUri(uri);

  auto *meshMgr = gz::common::MeshManager::Instance();
  const gz::common::Mesh *mesh = meshMgr->Load(loadPath);
  if (!mesh)
  {
    ignwarn << "[WorldQuerySystem] Failed to load mesh: " << loadPath
            << " (uri=" << uri << ")" << std::endl;
    return false;
  }

  gz::math::Vector3d scale = _sdfMesh->Scale();

  // Extract all triangles, transform to world frame
  for (unsigned int si = 0; si < mesh->SubMeshCount(); ++si)
  {
    auto subMeshLock = mesh->SubMeshByIndex(si);
    auto subMesh = subMeshLock.lock();
    if (!subMesh)
      continue;

    unsigned int indexCount = subMesh->IndexCount();
    for (unsigned int i = 0; i + 2 < indexCount; i += 3)
    {
      int i0 = subMesh->Index(i);
      int i1 = subMesh->Index(i + 1);
      int i2 = subMesh->Index(i + 2);

      if (i0 < 0 || i1 < 0 || i2 < 0)
        continue;

      gz::math::Vector3d v0 = subMesh->Vertex(i0) * scale;
      gz::math::Vector3d v1 = subMesh->Vertex(i1) * scale;
      gz::math::Vector3d v2 = subMesh->Vertex(i2) * scale;

      // Transform to world frame
      v0 = _worldPose.CoordPositionAdd(v0);
      v1 = _worldPose.CoordPositionAdd(v1);
      v2 = _worldPose.CoordPositionAdd(v2);

      _shape.triangles.push_back({v0, v1, v2});
    }
  }

  // Compute world-frame AABB from all triangle vertices
  if (!_shape.triangles.empty())
  {
    gz::math::Vector3d mn(1e9, 1e9, 1e9), mx(-1e9, -1e9, -1e9);
    for (const auto &tri : _shape.triangles)
    {
      for (const auto &v : {tri.v0, tri.v1, tri.v2})
      {
        mn.X(std::min(mn.X(), v.X()));
        mn.Y(std::min(mn.Y(), v.Y()));
        mn.Z(std::min(mn.Z(), v.Z()));
        mx.X(std::max(mx.X(), v.X()));
        mx.Y(std::max(mx.Y(), v.Y()));
        mx.Z(std::max(mx.Z(), v.Z()));
      }
    }
    _shape.aabb = gz::math::AxisAlignedBox(mn, mx);
  }

  ignmsg << "[WorldQuerySystem] Loaded mesh: " << loadPath
         << " (" << _shape.triangles.size() << " triangles)" << std::endl;
  return true;
}

// ---------------------------------------------------------------------------
// Walk entity parents to find a SemanticLabel component
// ---------------------------------------------------------------------------

static int FindSemanticLabel(
    gz::sim::Entity _entity,
    const gz::sim::EntityComponentManager &_ecm)
{
  gz::sim::Entity e = _entity;
  while (e != gz::sim::kNullEntity)
  {
    // Check for SemanticLabel on this entity
    auto labelComp = _ecm.Component<gz::sim::components::SemanticLabel>(e);
    if (labelComp)
      return static_cast<int>(labelComp->Data());

    // Check sibling visuals (the label is on visual entities, not collision)
    auto parentComp = _ecm.Component<gz::sim::components::ParentEntity>(e);
    if (!parentComp)
      break;

    gz::sim::Entity parent = parentComp->Data();

    // If we just moved up from a collision to its parent link,
    // check all visual children of that link for a label.
    _ecm.Each<gz::sim::components::Visual,
              gz::sim::components::ParentEntity>(
        [&](const gz::sim::Entity &visEntity,
            const gz::sim::components::Visual *,
            const gz::sim::components::ParentEntity *visParent) -> bool
        {
          if (visParent->Data() == parent)
          {
            auto vLabel = _ecm.Component<
                gz::sim::components::SemanticLabel>(visEntity);
            if (vLabel)
            {
              // Found it — but we can't break and return from here,
              // so we write into the outer variable via capture.
              // We'll check after this Each() call.
            }
          }
          return true;  // continue
        });

    // Direct check on the visual siblings
    // (simpler approach: iterate children of parent)
    e = parent;
  }
  return 0;
}

/// Simpler version: build a map of link-entity -> semantic label up front.
static std::unordered_map<gz::sim::Entity, int> BuildLabelMap(
    const gz::sim::EntityComponentManager &_ecm)
{
  std::unordered_map<gz::sim::Entity, int> labels;

  // Collect all entities with a SemanticLabel component
  _ecm.Each<gz::sim::components::SemanticLabel>(
      [&](const gz::sim::Entity &entity,
          const gz::sim::components::SemanticLabel *label) -> bool
      {
        labels[entity] = static_cast<int>(label->Data());
        return true;
      });

  return labels;
}

/// Given a collision entity, walk up to find the semantic label.
/// The label is typically on a visual sibling (same parent link).
static int LookupLabel(
    gz::sim::Entity _collisionEntity,
    const gz::sim::EntityComponentManager &_ecm,
    const std::unordered_map<gz::sim::Entity, int> &_labelMap)
{
  // Walk up the entity tree checking each entity and its children
  gz::sim::Entity e = _collisionEntity;
  for (int depth = 0; depth < 10 && e != gz::sim::kNullEntity; ++depth)
  {
    // Check this entity
    auto it = _labelMap.find(e);
    if (it != _labelMap.end())
      return it->second;

    // Check all children of this entity's parent (siblings)
    auto parentComp = _ecm.Component<gz::sim::components::ParentEntity>(e);
    if (!parentComp)
      break;

    gz::sim::Entity parent = parentComp->Data();

    // Check parent itself
    auto pit = _labelMap.find(parent);
    if (pit != _labelMap.end())
      return pit->second;

    // Check all entities whose parent is `parent` (siblings of e)
    for (const auto &[labelEntity, label] : _labelMap)
    {
      auto lp = _ecm.Component<gz::sim::components::ParentEntity>(labelEntity);
      if (lp && lp->Data() == parent)
        return label;
    }

    e = parent;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

WorldQuerySystem::WorldQuerySystem() = default;
WorldQuerySystem::~WorldQuerySystem() = default;

// ---------------------------------------------------------------------------
// Configure
// ---------------------------------------------------------------------------

void WorldQuerySystem::Configure(
    const gz::sim::Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &/*_eventMgr*/)
{
  if (_sdf->HasElement("output_csv_file"))
    this->outputPath_ = _sdf->Get<std::string>("output_csv_file");

  if (_sdf->HasElement("wait_iterations"))
    this->waitIterations_ = _sdf->Get<int>("wait_iterations");

  bool rayLengthExplicit = _sdf->HasElement("ray_length");
  if (rayLengthExplicit)
    this->rayLength_ = _sdf->Get<double>("ray_length");

  if (_sdf->HasElement("grid_min") && _sdf->HasElement("grid_max") &&
      _sdf->HasElement("resolution"))
  {
    auto gmin = _sdf->Get<gz::math::Vector3d>("grid_min");
    auto gmax = _sdf->Get<gz::math::Vector3d>("grid_max");
    double res = _sdf->Get<double>("resolution");
    if (!rayLengthExplicit)
      this->rayLength_ = res / 2.0;
    this->GenerateGrid(gmin, gmax, res);
  }
  else if (_sdf->HasElement("csv_file"))
  {
    this->LoadCSV(_sdf->Get<std::string>("csv_file"));
  }
  else
  {
    ignerr << "[WorldQuerySystem] Must specify <grid_min>/<grid_max>/<resolution>"
              " or <csv_file>.\n";
    return;
  }

  ignmsg << "[WorldQuerySystem] Configured: "
         << this->queryPoints_.size() << " query points, "
         << "margin=" << this->rayLength_ << "m, "
         << "wait=" << this->waitIterations_ << " iters, "
         << "output=" << this->outputPath_ << std::endl;
}

// ---------------------------------------------------------------------------
// GenerateGrid
// ---------------------------------------------------------------------------

void WorldQuerySystem::GenerateGrid(
    const gz::math::Vector3d &_min,
    const gz::math::Vector3d &_max,
    double _resolution)
{
  this->queryPoints_.clear();
  for (double x = _min.X(); x <= _max.X(); x += _resolution)
    for (double y = _min.Y(); y <= _max.Y(); y += _resolution)
      for (double z = _min.Z(); z <= _max.Z(); z += _resolution)
        this->queryPoints_.emplace_back(x, y, z);

  ignmsg << "[WorldQuerySystem] Grid: "
         << _min << " -> " << _max
         << " res=" << _resolution << "m"
         << " (" << this->queryPoints_.size() << " points)" << std::endl;
}

// ---------------------------------------------------------------------------
// LoadCSV
// ---------------------------------------------------------------------------

void WorldQuerySystem::LoadCSV(const std::string &_path)
{
  this->queryPoints_.clear();
  std::ifstream f(_path);
  if (!f.is_open())
  {
    ignerr << "[WorldQuerySystem] Cannot open CSV: " << _path << std::endl;
    return;
  }

  std::string line;
  bool headerSkipped = false;
  while (std::getline(f, line))
  {
    if (!headerSkipped && (line.find("x") != std::string::npos ||
                           line.find("X") != std::string::npos))
    {
      headerSkipped = true;
      continue;
    }
    headerSkipped = true;

    std::istringstream ss(line);
    double x, y, z;
    char comma;
    if (ss >> x >> comma >> y >> comma >> z)
      this->queryPoints_.emplace_back(x, y, z);
  }

  ignmsg << "[WorldQuerySystem] Loaded " << this->queryPoints_.size()
         << " points from " << _path << std::endl;
}

// ---------------------------------------------------------------------------
// PostUpdate — extract collision shapes from ECM, test all query points
// ---------------------------------------------------------------------------

void WorldQuerySystem::PostUpdate(
    const gz::sim::UpdateInfo &/*_info*/,
    const gz::sim::EntityComponentManager &_ecm)
{
  if (this->done_)
    return;

  this->currentIteration_++;
  if (this->currentIteration_ < this->waitIterations_)
    return;

  // ---- Step 1: Build semantic label map from ECM ----
  auto labelMap = BuildLabelMap(_ecm);
  ignmsg << "[WorldQuerySystem] Found " << labelMap.size()
         << " entities with semantic labels." << std::endl;

  // ---- Step 2: Extract all collision shapes with their world poses ----
  std::vector<CollisionShape> shapes;

  _ecm.Each<gz::sim::components::Collision,
            gz::sim::components::Geometry,
            gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &collisionEntity,
          const gz::sim::components::Collision *,
          const gz::sim::components::Geometry *geomComp,
          const gz::sim::components::ParentEntity *parentComp) -> bool
      {
        const sdf::Geometry &geom = geomComp->Data();
        auto geomType = geom.Type();

        // Compute world pose: collision pose relative to link,
        // link pose relative to model, etc.
        // We'll compose poses up the entity tree.
        gz::math::Pose3d worldPose;
        gz::sim::Entity e = collisionEntity;
        while (e != gz::sim::kNullEntity)
        {
          auto poseComp = _ecm.Component<gz::sim::components::Pose>(e);
          if (poseComp)
            worldPose = poseComp->Data() * worldPose;
          auto pp = _ecm.Component<gz::sim::components::ParentEntity>(e);
          if (!pp) break;
          e = pp->Data();
        }

        CollisionShape shape;
        shape.type = geomType;
        shape.worldPose = worldPose;
        shape.semanticLabel = LookupLabel(collisionEntity, _ecm, labelMap);

        // Get entity name for debugging
        auto nameComp = _ecm.Component<gz::sim::components::Name>(
            parentComp->Data());
        if (nameComp)
          shape.entityName = nameComp->Data();

        switch (geomType)
        {
          case sdf::GeometryType::BOX:
          {
            auto box = geom.BoxShape();
            if (box)
              shape.size = box->Size() * 0.5;  // half-extents
            shapes.push_back(shape);
            break;
          }
          case sdf::GeometryType::CYLINDER:
          {
            auto cyl = geom.CylinderShape();
            if (cyl)
            {
              shape.radius = cyl->Radius();
              shape.halfLength = cyl->Length() * 0.5;
            }
            shapes.push_back(shape);
            break;
          }
          case sdf::GeometryType::SPHERE:
          {
            auto sph = geom.SphereShape();
            if (sph)
              shape.radius = sph->Radius();
            shapes.push_back(shape);
            break;
          }
          case sdf::GeometryType::PLANE:
          {
            shapes.push_back(shape);
            break;
          }
          case sdf::GeometryType::MESH:
          {
            auto mesh = geom.MeshShape();
            if (mesh && LoadMeshTriangles(mesh, worldPose, shape))
            {
              shape.entityName += " [mesh]";
              shapes.push_back(shape);
            }
            else
            {
              ignwarn << "[WorldQuerySystem] Skipping mesh collision '"
                      << shape.entityName << "' — failed to load."
                      << std::endl;
            }
            break;
          }
          default:
            ignwarn << "[WorldQuerySystem] Unsupported geometry type "
                    << static_cast<int>(geomType) << std::endl;
            break;
        }

        return true;  // continue
      });

  ignmsg << "[WorldQuerySystem] Extracted " << shapes.size()
         << " collision shapes." << std::endl;

  if (shapes.empty())
  {
    ignwarn << "[WorldQuerySystem] No collision shapes found! "
            << "Ensure world has models with <collision> elements."
            << std::endl;
  }

  // ---- Step 3: Test each query point against all shapes ----
  const double margin = this->rayLength_;
  const size_t total = this->queryPoints_.size();
  const size_t logInterval = std::max<size_t>(total / 20, 1);

  std::vector<QueryResult> results;
  results.reserve(total);

  for (size_t i = 0; i < total; ++i)
  {
    const auto &pt = this->queryPoints_[i];
    QueryResult qr;
    qr.x = pt.X();
    qr.y = pt.Y();
    qr.z = pt.Z();
    qr.occupied = false;
    qr.semanticLabel = 0;
    qr.minDist = -1.0;

    double bestDist = std::numeric_limits<double>::max();

    for (const auto &shape : shapes)
    {
      // Transform point to shape-local frame
      gz::math::Vector3d local =
          shape.worldPose.Inverse().CoordPositionAdd(pt);

      bool inside = false;
      double dist = 0.0;

      switch (shape.type)
      {
        case sdf::GeometryType::BOX:
          inside = PointInBox(local, shape.size, margin);
          if (inside)
          {
            // Approximate distance: min distance to any face
            dist = std::min({
                shape.size.X() - std::abs(local.X()),
                shape.size.Y() - std::abs(local.Y()),
                shape.size.Z() - std::abs(local.Z())});
            dist = std::max(0.0, dist);
          }
          break;

        case sdf::GeometryType::CYLINDER:
          inside = PointInCylinder(local, shape.radius,
                                   shape.halfLength, margin);
          if (inside)
          {
            double rDist = shape.radius -
                std::sqrt(local.X()*local.X() + local.Y()*local.Y());
            double zDist = shape.halfLength - std::abs(local.Z());
            dist = std::max(0.0, std::min(rDist, zDist));
          }
          break;

        case sdf::GeometryType::SPHERE:
          inside = PointInSphere(local, shape.radius, margin);
          if (inside)
            dist = std::max(0.0, shape.radius - local.Length());
          break;

        case sdf::GeometryType::PLANE:
          inside = PointNearPlane(local, margin);
          if (inside)
            dist = std::abs(local.Z());
          break;

        case sdf::GeometryType::MESH:
        {
          // Mesh triangles are already in world frame, so use pt directly
          double meshDist = 0.0;
          inside = PointNearMesh(pt, shape, margin, meshDist);
          if (inside)
            dist = meshDist;
          break;
        }

        default:
          break;
      }

      if (inside)
      {
        qr.occupied = true;
        if (dist < bestDist)
        {
          bestDist = dist;
          qr.minDist = dist;
          qr.semanticLabel = shape.semanticLabel;
          qr.entityName = shape.entityName;
        }
      }
    }

    results.push_back(qr);

    if ((i + 1) % logInterval == 0)
    {
      size_t nOcc = 0;
      for (const auto &r : results)
        if (r.occupied) nOcc++;
      ignmsg << "[WorldQuerySystem] Progress: " << (i + 1) << "/" << total
             << " (" << nOcc << " occupied)" << std::endl;
    }
  }

  // ---- Step 4: Save results ----
  this->SaveResults(this->outputPath_, results);

  size_t nOcc = 0;
  for (const auto &r : results)
    if (r.occupied) nOcc++;

  ignmsg << "[WorldQuerySystem] Done. " << results.size() << " points queried, "
         << nOcc << " occupied." << std::endl;

  this->done_ = true;
}

// ---------------------------------------------------------------------------
// SaveResults
// ---------------------------------------------------------------------------

void WorldQuerySystem::SaveResults(
    const std::string &_path,
    const std::vector<QueryResult> &_results)
{
  std::ofstream f(_path);
  if (!f.is_open())
  {
    ignerr << "[WorldQuerySystem] Cannot write to " << _path << std::endl;
    return;
  }

  f << "x,y,z,occupied,semantic_label,distance,entity_name\n";
  f << std::fixed << std::setprecision(4);

  for (const auto &r : _results)
  {
    f << r.x << "," << r.y << "," << r.z << ","
      << (r.occupied ? 1 : 0) << ","
      << r.semanticLabel << ","
      << r.minDist << ","
      << r.entityName << "\n";
  }

  ignmsg << "[WorldQuerySystem] Results saved to " << _path << std::endl;
}

// ---------------------------------------------------------------------------
// Plugin registration
// ---------------------------------------------------------------------------

IGNITION_ADD_PLUGIN(
    world_query_system::WorldQuerySystem,
    gz::sim::System,
    world_query_system::WorldQuerySystem::ISystemConfigure,
    world_query_system::WorldQuerySystem::ISystemPostUpdate)
