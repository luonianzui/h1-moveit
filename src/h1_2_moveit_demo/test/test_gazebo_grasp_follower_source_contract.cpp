#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <string>

#ifndef H1_GAZEBO_GRASP_FOLLOWER_SOURCE
#error "H1_GAZEBO_GRASP_FOLLOWER_SOURCE must name the plugin source file"
#endif

namespace
{

std::string plugin_source()
{
  std::ifstream input(H1_GAZEBO_GRASP_FOLLOWER_SOURCE);
  EXPECT_TRUE(input.is_open()) << H1_GAZEBO_GRASP_FOLLOWER_SOURCE;
  return std::string(
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>());
}

void expect_contains(const std::string & source, const std::string & token)
{
  EXPECT_NE(source.find(token), std::string::npos) << token;
}

TEST(GazeboGraspFollowerSourceContract, ChildScopeSelectsOneHostModelLink)
{
  const std::string source = plugin_source();
  expect_contains(source, "HasElement(\"child_scope\")");
  expect_contains(source, "link_candidate_count != 1U");
  expect_contains(source, "IsDescendantOf(child_link_entity_, child_model_entity_, ecm)");
  expect_contains(source, "child_pose_entity_ = child_link_entity_");
  expect_contains(source, "worldPose(child_pose_entity_, ecm)");
}

TEST(GazeboGraspFollowerSourceContract, WrenchConfigurationIsFiniteAndOptional)
{
  const std::string source = plugin_source();
  expect_contains(source, "Get<bool>(\"gravity_compensation\", true)");
  expect_contains(source, "\"max_force\", child_mass_ * kMaximumLinearAcceleration");
  expect_contains(source, "\"max_torque\", kDefaultMaximumTorque");
  expect_contains(source, "IsPositiveFinite(maximum_force_)");
  expect_contains(source, "LimitMagnitude(force, maximum_force_)");
  expect_contains(source, "LimitMagnitude(torque, maximum_torque_)");
  expect_contains(source, "child_link.AddWorldWrench(ecm, force, torque)");
}

TEST(GazeboGraspFollowerSourceContract, DetachedPreloadRecapturesPoseBeforeAttachAck)
{
  const std::string source = plugin_source();
  expect_contains(source, "Get<bool>(\"initially_attached\", true)");
  expect_contains(source, "attached_.store(initially_attached)");
  expect_contains(
    source, "initially_attached ? kPublishAttached : kPublishDetached");
  expect_contains(source, "if (initially_attached) {");
  expect_contains(source, "CaptureCurrentRelativePose(ecm);");

  const std::size_t pre_update = source.find("void PreUpdate(");
  const std::size_t attach_request = source.find("if (attach_requested_)", pre_update);
  const std::size_t recapture =
    source.find("CaptureCurrentRelativePose(ecm);", attach_request);
  const std::size_t enable = source.find("attached_.store(true);", recapture);
  const std::size_t attach_ack =
    source.find("requested_state_.store(kPublishAttached);", enable);
  const std::size_t wrench_gate = source.find("if (attached_.load())", pre_update);
  ASSERT_NE(pre_update, std::string::npos);
  ASSERT_NE(attach_request, std::string::npos);
  ASSERT_NE(recapture, std::string::npos);
  ASSERT_NE(enable, std::string::npos);
  ASSERT_NE(attach_ack, std::string::npos);
  ASSERT_NE(wrench_gate, std::string::npos);
  EXPECT_LT(attach_request, recapture);
  EXPECT_LT(recapture, enable);
  EXPECT_LT(enable, attach_ack);
  EXPECT_LT(attach_ack, wrench_gate);

  const std::size_t on_attach = source.find("void OnAttach(");
  const std::size_t capture_function =
    source.find("void CaptureCurrentRelativePose(", on_attach);
  ASSERT_NE(on_attach, std::string::npos);
  ASSERT_NE(capture_function, std::string::npos);
  const std::string on_attach_body =
    source.substr(on_attach, capture_function - on_attach);
  expect_contains(on_attach_body, "attached_.store(false)");
  expect_contains(on_attach_body, "attach_requested_ = true");
  EXPECT_EQ(on_attach_body.find("attached_.store(true)"), std::string::npos);
  EXPECT_EQ(on_attach_body.find("kPublishAttached"), std::string::npos);

  const std::size_t on_detach = source.find("void OnDetach(");
  ASSERT_NE(on_detach, std::string::npos);
  const std::string on_detach_body = source.substr(on_detach, on_attach - on_detach);
  expect_contains(on_detach_body, "attach_requested_ = false");
  expect_contains(on_detach_body, "attached_.store(false)");
  expect_contains(on_detach_body, "requested_state_.store(kPublishDetached)");
}

TEST(GazeboGraspFollowerSourceContract, StateHeartbeatPersistsAndFreshAcksTakePriority)
{
  const std::string source = plugin_source();
  expect_contains(
    source, "static constexpr std::uint32_t kStateHeartbeatPeriodSteps = 250U;");
  expect_contains(source, "std::uint32_t state_heartbeat_steps_{0U};");
  EXPECT_EQ(source.find("initial_announce_steps_"), std::string::npos);

  const std::size_t pre_update = source.find("void PreUpdate(");
  const std::size_t state_exchange =
    source.find("requested_state_.exchange(kPublishNone)", pre_update);
  const std::size_t ack_branch =
    source.find("if (requested != kPublishNone)", state_exchange);
  const std::size_t ack_reset =
    source.find("state_heartbeat_steps_ = 0U;", ack_branch);
  const std::size_t ack_publish = source.find("PublishState(", ack_reset);
  const std::size_t heartbeat_branch = source.find("} else {", ack_publish);
  const std::size_t heartbeat_increment =
    source.find("++state_heartbeat_steps_;", heartbeat_branch);
  const std::size_t heartbeat_threshold =
    source.find(
      "state_heartbeat_steps_ >= kStateHeartbeatPeriodSteps", heartbeat_increment);
  const std::size_t heartbeat_publish =
    source.find("PublishState(attached_.load() ? \"attached\" : \"detached\")",
      heartbeat_threshold);
  const std::size_t private_section = source.find("\nprivate:", pre_update);
  ASSERT_NE(pre_update, std::string::npos);
  ASSERT_NE(state_exchange, std::string::npos);
  ASSERT_NE(ack_branch, std::string::npos);
  ASSERT_NE(ack_reset, std::string::npos);
  ASSERT_NE(ack_publish, std::string::npos);
  ASSERT_NE(heartbeat_branch, std::string::npos);
  ASSERT_NE(heartbeat_increment, std::string::npos);
  ASSERT_NE(heartbeat_threshold, std::string::npos);
  ASSERT_NE(heartbeat_publish, std::string::npos);
  ASSERT_NE(private_section, std::string::npos);
  EXPECT_LT(state_exchange, ack_branch);
  EXPECT_LT(ack_branch, ack_reset);
  EXPECT_LT(ack_reset, ack_publish);
  EXPECT_LT(ack_publish, heartbeat_branch);
  EXPECT_LT(heartbeat_branch, heartbeat_increment);
  EXPECT_LT(heartbeat_increment, heartbeat_threshold);
  EXPECT_LT(heartbeat_threshold, heartbeat_publish);
  EXPECT_LT(heartbeat_publish, private_section);

  const std::size_t configure_request = source.find(
    "initially_attached ? kPublishAttached : kPublishDetached");
  const std::size_t on_detach = source.find("void OnDetach(");
  const std::size_t detach_request =
    source.find("requested_state_.store(kPublishDetached)", on_detach);
  const std::size_t attach_capture =
    source.find("CaptureCurrentRelativePose(ecm);", pre_update);
  const std::size_t attach_request =
    source.find("requested_state_.store(kPublishAttached);", attach_capture);
  ASSERT_NE(configure_request, std::string::npos);
  ASSERT_NE(detach_request, std::string::npos);
  ASSERT_NE(attach_request, std::string::npos);
  EXPECT_LT(configure_request, pre_update);
  EXPECT_LT(attach_request, state_exchange);
}

TEST(GazeboGraspFollowerSourceContract, DoesNotCommandPoseJointsOrCollisionState)
{
  const std::string source = plugin_source();
  for (const char * forbidden : {
      "PoseCmd", "JointPositionReset", "JointVelocityCmd", "CollisionEnabled"})
  {
    EXPECT_EQ(source.find(forbidden), std::string::npos) << forbidden;
  }
}

}  // namespace
