#pragma once

#include <chrono>
#include <mutex>

#include <gz/common/MouseEvent.hh>
#include <gz/gui/Plugin.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>

class CentrifugeButton : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QString machineStatus READ MachineStatus NOTIFY MachineStatusChanged)

  public: QString MachineStatus() const { return this->statusText; }
  signals: void MachineStatusChanged();

  protected: void LoadConfig(const tinyxml2::XMLElement *element) override;
  protected: bool eventFilter(QObject *object, QEvent *event) override;

  private: void PickButton();
  private: void OnStatus(const gz::msgs::StringMsg &message);
  private: QString statusText = "等待设备状态";
  private: gz::transport::Node transport;
  private: gz::transport::Node::Publisher publisher;
  private: gz::rendering::ScenePtr scene;
  private: gz::rendering::CameraPtr camera;
  private: gz::common::MouseEvent mouse;
  private: std::mutex mutex;
  private: bool pending = false;
  private: std::chrono::steady_clock::time_point lastClick{};
};
