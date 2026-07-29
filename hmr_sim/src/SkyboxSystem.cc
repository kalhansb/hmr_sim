#include "skybox_system/SkyboxSystem.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include <ignition/common/Console.hh>

using namespace skybox_system;

SkyboxSystem::SkyboxSystem() = default;

void SkyboxSystem::Configure(
    const gz::sim::Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_element,
    gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &/*_eventManager*/)
{
  // Read <time> from SDF (hours, 0-24)
  if (_element->HasElement("time"))
  {
    this->timeOfDay = _element->Get<double>("time");
    ignmsg << "[SkyboxSystem] Time of day set to: "
           << this->timeOfDay << " hours" << std::endl;
  }

  // Read optional <cubemap_uri> for custom skybox texture
  if (_element->HasElement("cubemap_uri"))
  {
    this->cubemapUri = _element->Get<std::string>("cubemap_uri");
    ignmsg << "[SkyboxSystem] Cubemap URI: "
           << this->cubemapUri << std::endl;
  }

  ignmsg << "[SkyboxSystem] Plugin configured." << std::endl;
}

void SkyboxSystem::PostUpdate(
    const gz::sim::UpdateInfo &/*_info*/,
    const gz::sim::EntityComponentManager &/*_ecm*/)
{
  if (this->skyConfigured)
    return;

  // Find the ogre2 render engine
  auto engine = gz::rendering::engine("ogre2");
  if (!engine)
  {
    // Engine may not be ready yet on the first few iterations
    return;
  }

  if (engine->SceneCount() == 0)
    return;

  auto scene = engine->SceneByIndex(0);
  if (!scene)
    return;

  // Enable the built-in sky
  scene->SetSkyEnabled(true);

  ignmsg << "[SkyboxSystem] Sky enabled on scene \""
         << scene->Name() << "\"" << std::endl;

  this->skyConfigured = true;
}

IGNITION_ADD_PLUGIN(
    skybox_system::SkyboxSystem,
    gz::sim::System,
    skybox_system::SkyboxSystem::ISystemConfigure,
    skybox_system::SkyboxSystem::ISystemPostUpdate)
