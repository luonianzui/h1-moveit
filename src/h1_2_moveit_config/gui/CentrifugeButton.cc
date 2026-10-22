#include "CentrifugeButton.hh"

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <QJsonDocument>
#include <QJsonObject>

void CentrifugeButton::LoadConfig(const tinyxml2::XMLElement *)
{
  this->publisher = this->transport.Advertise<gz::msgs::StringMsg>(
    "/centrifuge/button/action");
  this->transport.Subscribe("/centrifuge/machine/status", &CentrifugeButton::OnStatus, this);
  auto *window = gz::gui::App()->findChild<gz::gui::MainWindow *>();
  if (!window || !this->publisher)
  {
    gzerr << "Centrifuge button disabled: missing window or transport" << std::endl;
    return;
  }
  window->installEventFilter(this);
  gzmsg << "Centrifuge buttons: blue lid, green start/stop"
        << std::endl;
}

bool CentrifugeButton::eventFilter(QObject *object, QEvent *event)
{
  if (event->type() == gz::gui::events::LeftClickOnScene::kType)
  {
    const auto &click =
      static_cast<gz::gui::events::LeftClickOnScene *>(event)->Mouse();
    std::lock_guard<std::mutex> guard(this->mutex);
    if (click.Button() == gz::common::MouseEvent::LEFT &&
        click.Type() == gz::common::MouseEvent::RELEASE &&
        !click.Dragging() && !click.Control() && !click.Shift() && !click.Alt())
    {
      this->mouse = click;
      this->pending = true;
    }
  }
  else if (event->type() == gz::gui::events::Render::kType)
  {
    this->PickButton();
  }
  return QObject::eventFilter(object, event);
}

void CentrifugeButton::PickButton()
{
  std::lock_guard<std::mutex> guard(this->mutex);
  if (!this->pending)
    return;
  this->pending = false;
  if (!this->scene)
    this->scene = gz::rendering::sceneFromFirstRenderEngine();
  if (!this->scene)
    return;
  if (!this->camera)
  {
    for (unsigned int index = 0; index < this->scene->NodeCount(); ++index)
    {
      auto candidate = std::dynamic_pointer_cast<gz::rendering::Camera>(
        this->scene->NodeByIndex(index));
      if (candidate && candidate->HasUserData("user-camera"))
      {
        auto value = candidate->UserData("user-camera");
        if (const auto *isUser = std::get_if<bool>(&value); isUser && *isUser)
        {
          this->camera = candidate;
          break;
        }
      }
    }
  }
  if (!this->camera)
    return;
  const auto visual = this->scene->VisualAt(this->camera, this->mouse.Pos());
  if (!visual)
    return;
  std::string action;
  if (visual->Name().find("centrifuge::assembly::lid_button_") == 0)
    action = "lid";
  else if (visual->Name().find("centrifuge::assembly::rotor_button_") == 0)
    action = "rotor";
  else
    return;
  const auto now = std::chrono::steady_clock::now();
  if (now - this->lastClick < std::chrono::milliseconds(500))
    return;
  this->lastClick = now;
  gz::msgs::StringMsg message;
  message.set_data(action);
  this->publisher.Publish(message);
  gzmsg << "CENTRIFUGE_BUTTON_CLICK visual=" << visual->Name() << std::endl;
}

void CentrifugeButton::OnStatus(const gz::msgs::StringMsg &message)
{
  const auto payload = QJsonDocument::fromJson(
    QByteArray::fromStdString(message.data())).object();
  const auto speed = payload.value("rotor_rpm").toDouble();
  const auto enabled = payload.value("rotor_enabled").toBool();
  const auto ready = payload.value("ready").toBool();
  const QString state = !ready ? "等待设备" : enabled ? "运行中" :
    (std::abs(speed) > 0.2 ? "减速中，禁止开盖" : "已停止");
  const QString mode = payload.value("open_lid_demo").toBool() ?
    "开盖演示（仅仿真）" : "关盖互锁";
  const QString text = mode + "  |  " + state + QString("  |  %1 rpm").arg(speed, 0, 'f', 1);
  QMetaObject::invokeMethod(this, [this, text]()
  {
    this->statusText = text;
    emit this->MachineStatusChanged();
  }, Qt::QueuedConnection);
}

GZ_ADD_PLUGIN(CentrifugeButton, gz::gui::Plugin)
