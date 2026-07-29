#ifndef SKYBOX_SYSTEM__SKYBOX_SYSTEM_HH_
#define SKYBOX_SYSTEM__SKYBOX_SYSTEM_HH_

#include <gz/sim/System.hh>
#include <memory>
#include <string>

namespace skybox_system
{
  class SkyboxSystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    public: SkyboxSystem();
    public: ~SkyboxSystem() override = default;

    public: void Configure(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager) override;

    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    private: bool skyConfigured{false};
    private: double timeOfDay{10.0};
    private: std::string cubemapUri;
  };
}

#endif
