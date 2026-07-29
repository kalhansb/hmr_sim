#ifndef WORLD_QUERY_SYSTEM__WORLDQUERYSYSTEM_HH_
#define WORLD_QUERY_SYSTEM__WORLDQUERYSYSTEM_HH_

#include <memory>
#include <string>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/sim/System.hh>

namespace world_query_system
{
  /// \brief Gazebo system plugin that queries occupancy and semantic labels
  /// for a grid of 3D points using multi-directional raycasting.
  ///
  /// Casts 6 rays (±X, ±Y, ±Z) from each query point. A point is "occupied"
  /// if any ray hits within ray_length. Semantic label comes from the nearest
  /// hit visual's Label user data.
  ///
  /// ## SDF Parameters
  ///
  /// Grid mode (generates a regular 3D grid):
  /// - <grid_min> xyz min corner (e.g. "-5 -5 0")
  /// - <grid_max> xyz max corner (e.g. "15 15 3")
  /// - <resolution> voxel size in meters (e.g. "0.1")
  ///
  /// CSV mode (loads external points):
  /// - <csv_file> path to CSV with x,y,z columns
  ///
  /// Common:
  /// - <output_csv_file> output path (default: "world_query_results.csv")
  /// - <ray_length> max hit distance in meters (default: resolution/2 or 0.05)
  /// - <wait_iterations> sim iterations before querying (default: 200)
  class WorldQuerySystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    public: WorldQuerySystem();
    public: ~WorldQuerySystem() override;

    public: void Configure(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr) override;

    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    private: struct QueryResult
    {
      double x, y, z;
      bool occupied;
      int semanticLabel;
      double minDist;
      std::string entityName;
    };

    /// \brief Generate a regular 3D grid of query points.
    private: void GenerateGrid(
                 const gz::math::Vector3d &_min,
                 const gz::math::Vector3d &_max,
                 double _resolution);

    /// \brief Load query points from a CSV file.
    private: void LoadCSV(const std::string &_path);

    /// \brief Save results to CSV.
    private: void SaveResults(
                 const std::string &_path,
                 const std::vector<QueryResult> &_results);

    /// \brief Query points
    private: std::vector<gz::math::Vector3d> queryPoints_;

    /// \brief Output file path
    private: std::string outputPath_{"world_query_results.csv"};

    /// \brief Max ray hit distance to count as occupied
    private: double rayLength_{0.05};

    /// \brief Iterations to wait for rendering to initialise
    private: int waitIterations_{200};

    /// \brief Current iteration counter
    private: int currentIteration_{0};

    /// \brief Whether queries have been completed
    private: bool done_{false};
  };
}

#endif  // WORLD_QUERY_SYSTEM__WORLDQUERYSYSTEM_HH_
