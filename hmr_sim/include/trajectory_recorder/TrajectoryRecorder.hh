#ifndef TRAJECTORY_RECORDER__TRAJECTORYRECORDER_HH_
#define TRAJECTORY_RECORDER__TRAJECTORYRECORDER_HH_

#include <memory>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>

namespace trajectory_recorder
{
  /// \brief Gazebo system plugin that teleports a camera model through a
  /// trajectory and records RGB-D + semantic frames to disk.
  ///
  /// Produces output identical to the Replica pipeline (poses.json, camera.json,
  /// color/*.npy, depth/*.npy, semantic/*.npy) so the existing replay_node +
  /// SCovox pipeline can consume it directly.
  ///
  /// ## SDF Parameters
  /// - <trajectory_file> Path to poses.json (same format as Replica)
  /// - <output_dir> Directory to write frames (created if absent)
  /// - <model_name> Name of the camera model in the world (default: trajectory_camera)
  /// - <settle_steps> Sim steps to wait after teleport for render (default: 10)
  class TrajectoryRecorder :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
  {
    public: TrajectoryRecorder();
    public: ~TrajectoryRecorder() override;

    public: void Configure(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr) override;

    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    private: struct Impl;
    private: std::unique_ptr<Impl> impl_;
  };
}

#endif  // TRAJECTORY_RECORDER__TRAJECTORYRECORDER_HH_
