#include <gz/plugin/Register.hh>
#include <gz/sim/gui/GuiSystem.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Util.hh>

namespace gz::sim {

/// \brief GUI system plugin that bridges SensorTopic components to sensor entities
/// in the Gazebo GUI EntityComponentManager (ECM). In Gazebo Harmonic (gz-sim 8),
/// SceneBroadcaster serializes SensorTag, NameTag, ParentEntityTag, and PoseTag,
/// but does not replicate SensorTopic to the GUI ECM. This causes GUI visualization
/// plugins like VisualizeLidar to fail to resolve the sensor entity, causing lidar
/// visual rays to render at world origin (0, 0, 0) instead of tracking the robot.
class LidarTopicBridge : public gz::sim::GuiSystem {
  Q_OBJECT

public:
  LidarTopicBridge() = default;
  ~LidarTopicBridge() override = default;

  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override {
    if (_pluginElem) {
      auto sElem = _pluginElem->FirstChildElement("sensor_name");
      if (sElem && sElem->GetText()) {
        this->sensorName = sElem->GetText();
      }
      auto tElem = _pluginElem->FirstChildElement("topic");
      if (tElem && tElem->GetText()) {
        this->topicName = tElem->GetText();
      }
    }
  }

  void Update(const UpdateInfo &, EntityComponentManager &_ecm) override {
    _ecm.Each<components::Sensor, components::Name>(
      [&](const Entity &_e, const components::Sensor *, const components::Name *_name) -> bool {
        // If SensorTopic is already present on this entity, do nothing
        if (_ecm.Component<components::SensorTopic>(_e)) {
          return true;
        }

        const std::string &name = _name->Data();
        if (name == this->sensorName ||
            name.find("lidar") != std::string::npos ||
            name.find("rplidar") != std::string::npos) {
          _ecm.CreateComponent(_e, components::SensorTopic(this->topicName));
        }
        return true;
      });
  }

private:
  std::string sensorName{"rplidar_a1m8"};
  std::string topicName{"scan"};
};

}  // namespace gz::sim

GZ_ADD_PLUGIN(gz::sim::LidarTopicBridge,
              gz::gui::Plugin,
              gz::sim::GuiSystem)

#include "LidarTopicBridge.moc"
