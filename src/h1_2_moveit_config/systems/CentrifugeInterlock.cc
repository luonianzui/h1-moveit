#include "CentrifugeSafety.hh"

#include <chrono>
#include <deque>
#include <mutex>
#include <sstream>
#include <variant>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>

class CentrifugeInterlock : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &configuration,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    this->model = entity;
    this->safety.openLidDemo = configuration->Get<bool>("allow_open_lid_demo", false).first;
    this->lidPublisher = this->transport.Advertise<gz::msgs::Double>(
      "/centrifuge/lid/applied_angle");
    this->statusPublisher = this->transport.Advertise<gz::msgs::StringMsg>(
      "/centrifuge/machine/status");
    this->transport.Subscribe("/centrifuge/lid/target_angle",
      &CentrifugeInterlock::OnLid, this);
    this->transport.Subscribe("/centrifuge/rotor/enable",
      &CentrifugeInterlock::OnRotor, this);
    this->transport.Subscribe("/centrifuge/machine/heartbeat",
      &CentrifugeInterlock::OnHeartbeat, this);
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &entities) override
  {
    if (info.paused)
      return;
    if (this->lid == gz::sim::kNullEntity)
    {
      const gz::sim::Model machine(this->model);
      this->lid = machine.JointByName(entities, "lid_hinge_joint");
      this->rotor = machine.JointByName(entities, "rotor_spin_joint");
      if (this->lid == gz::sim::kNullEntity || this->rotor == gz::sim::kNullEntity)
      {
        this->lid = gz::sim::kNullEntity;
        return;
      }
      gz::sim::Joint(this->lid).EnablePositionCheck(entities);
      gz::sim::Joint(this->lid).EnableVelocityCheck(entities);
      gz::sim::Joint(this->rotor).EnableVelocityCheck(entities);
    }
    auto *position = entities.Component<gz::sim::components::JointPosition>(this->lid);
    auto *lidSpeed = entities.Component<gz::sim::components::JointVelocity>(this->lid);
    auto *rotorSpeed = entities.Component<gz::sim::components::JointVelocity>(this->rotor);
    this->safety.ready = position && lidSpeed && rotorSpeed &&
      !position->Data().empty() && !lidSpeed->Data().empty() && !rotorSpeed->Data().empty();
    if (this->safety.ready)
    {
      this->safety.lidPosition = position->Data()[0];
      this->safety.lidVelocity = lidSpeed->Data()[0];
      this->safety.rotorVelocity = rotorSpeed->Data()[0];
      this->safety.ready = std::isfinite(this->safety.lidPosition) &&
        std::isfinite(this->safety.lidVelocity) && std::isfinite(this->safety.rotorVelocity);
    }
    std::deque<std::variant<double, bool>> pending;
    bool leaseFresh;
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      pending.swap(this->requests);
      leaseFresh = std::chrono::steady_clock::now() - this->heartbeat <
        std::chrono::seconds(1);
    }
    for (const auto &request : pending)
    {
      if (const auto *target = std::get_if<double>(&request))
        this->safety.RequestLid(*target);
      else
        this->safety.RequestRotor(std::get<bool>(request), leaseFresh);
    }
    const double duration = std::clamp(
      std::chrono::duration<double>(info.dt).count(), 0.0, 0.1);
    const double previousTarget = this->safety.rotorTarget;
    this->safety.Step(duration, leaseFresh);
    double torque = 0.0;
    if (this->safety.ready && duration > 0.0)
    {
      torque = std::clamp(
        0.04 * (this->safety.rotorTarget - previousTarget) / duration +
        2.0 * (this->safety.rotorTarget - this->safety.rotorVelocity) +
        0.01 * this->safety.rotorVelocity, -2.0, 2.0);
    }
    entities.SetComponentData<gz::sim::components::JointForceCmd>(this->rotor, {torque});
    const double simTime = std::chrono::duration<double>(info.simTime).count();
    if (simTime - this->lastPublish < 0.02)
      return;
    this->lastPublish = simTime;
    gz::msgs::Double lidCommand;
    lidCommand.set_data(this->safety.lidTarget);
    this->lidPublisher.Publish(lidCommand);
    const auto &state = this->safety;
    std::ostringstream payload;
    payload << std::boolalpha
      << "{\"ready\":" << state.ready << ",\"rotor_enabled\":" << state.enabled
      << ",\"open_lid_demo\":" << state.openLidDemo
      << ",\"rotor_rad_s\":" << (state.ready ? state.rotorVelocity : 0.0)
      << ",\"rotor_rpm\":" << (state.ready ? state.rotorVelocity * 30.0 / M_PI : 0.0)
      << ",\"lid_rad\":" << (state.ready ? state.lidPosition : 0.0)
      << ",\"lid_target_rad\":" << state.lidTarget
      << ",\"can_open\":" << state.CanOpen() << ",\"can_start\":" << state.CanStart()
      << ",\"starts\":" << state.starts << ",\"stops\":" << state.stops
      << ",\"blocked_lid\":" << state.blockedLid << ",\"blocked_start\":" << state.blockedStart
      << ",\"rejected\":\"" << state.rejected << "\"}";
    gz::msgs::StringMsg status;
    status.set_data(payload.str());
    this->statusPublisher.Publish(status);
  }

  private: void OnLid(const gz::msgs::Double &message)
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    if (this->requests.size() < 128)
      this->requests.emplace_back(message.data());
  }

  private: void OnRotor(const gz::msgs::Boolean &message)
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    if (this->requests.size() < 128)
      this->requests.emplace_back(message.data());
  }

  private: void OnHeartbeat(const gz::msgs::Boolean &message)
  {
    if (!message.data()) return;
    std::lock_guard<std::mutex> guard(this->mutex);
    this->heartbeat = std::chrono::steady_clock::now();
  }

  private: gz::sim::Entity model = gz::sim::kNullEntity;
  private: gz::sim::Entity lid = gz::sim::kNullEntity;
  private: gz::sim::Entity rotor = gz::sim::kNullEntity;
  private: gz::transport::Node transport;
  private: gz::transport::Node::Publisher lidPublisher;
  private: gz::transport::Node::Publisher statusPublisher;
  private: CentrifugeSafety safety;
  private: std::mutex mutex;
  private: std::deque<std::variant<double, bool>> requests;
  private: std::chrono::steady_clock::time_point heartbeat{};
  private: double lastPublish = -1.0;
};

GZ_ADD_PLUGIN(CentrifugeInterlock, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
