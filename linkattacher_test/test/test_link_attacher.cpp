// Copyright 2026 IFRA-Cranfield
// Licensed under the Apache License, Version 2.0.

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdf/Root.hh>
#include <sdf/World.hh>

using namespace std::chrono_literals;
namespace sim = gz::sim;
namespace components = gz::sim::components;
using Attach = linkattacher_msgs::srv::AttachLink;
using Detach = linkattacher_msgs::srv::DetachLink;

class LinkAttacherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context);
    node = std::make_shared<rclcpp::Node>("link_attacher_test", options);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor->add_node(node);
    fixture = std::make_unique<sim::TestFixture>(TEST_WORLD_PATH);
    fixture->OnConfigure([this](const auto & entity, const auto &, auto &, auto & events) {
      world = entity;
      event_manager = &events;
    });
    fixture->OnPreUpdate([this](const auto &, auto & ecm) {
      if (before_step) {
        auto callback = std::move(before_step);
        callback(ecm);
      }
      if (each_step) {
        each_step(ecm);
      }
    });
    fixture->OnPostUpdate([this](const auto &, const auto & ecm) {
      auto robot = ecm.EntityByComponents(components::Model(), components::Name("robot"));
      auto object = ecm.EntityByComponents(components::Model(), components::Name("object"));
      auto free_box = ecm.EntityByComponents(components::Model(), components::Name("box_free"));
      tool_pose = sim::worldPose(sim::Model(robot).LinkByName(ecm, "tool"), ecm);
      if (object != sim::kNullEntity) {
        object_pose = sim::worldPose(object, ecm);
      }
      free_pose = sim::worldPose(free_box, ecm);
      joints = 0;
      ecm.template Each<components::DetachableJoint>(
        [this](const auto &, const auto *) {++joints; return true;});
    }).Finalize();
    ASSERT_TRUE(fixture->Server()->Run(true, 1, false));
    attach = node->create_client<Attach>("/ATTACHLINK");
    detach = node->create_client<Detach>("/DETACHLINK");
    ASSERT_TRUE(attach->wait_for_service(10s));
    ASSERT_TRUE(detach->wait_for_service(10s));
  }

  void TearDown() override
  {
    fixture.reset();  // Unload the real plugin, including its ROS context.
    EXPECT_TRUE(context->is_valid());
    executor.reset();
    node.reset();
    context->shutdown("Test finished");
  }

  template<typename Service>
  typename Service::Response::SharedPtr Call(
    const typename rclcpp::Client<Service>::SharedPtr & client,
    const std::string & model1 = "robot", const std::string & link1 = "tool",
    const std::string & model2 = "object", const std::string & link2 = "link")
  {
    auto req = std::make_shared<typename Service::Request>();
    req->model1_name = model1;
    req->link1_name = link1;
    req->model2_name = model2;
    req->link2_name = link2;
    auto future = client->async_send_request(req);
    auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      // Paused updates must still process requests, without advancing physics.
      fixture->Server()->RunOnce(true);
      executor->spin_some();
      if (future.wait_for(0s) == std::future_status::ready) {
        return future.get();
      }
      std::this_thread::sleep_for(1ms);
    }
    throw std::runtime_error("Timed out waiting for LinkAttacher service response");
  }

  void Step(uint64_t iterations)
  {
    ASSERT_TRUE(fixture->Server()->Run(true, iterations, false));
  }

  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Client<Attach>::SharedPtr attach;
  rclcpp::Client<Detach>::SharedPtr detach;
  std::unique_ptr<sim::TestFixture> fixture;
  std::function<void(sim::EntityComponentManager &)> before_step;
  std::function<void(sim::EntityComponentManager &)> each_step;
  sim::Entity world{sim::kNullEntity};
  sim::EventManager * event_manager{nullptr};
  gz::math::Pose3d tool_pose, object_pose, free_pose;
  size_t joints{0};
};

TEST_F(LinkAttacherTest, FollowsMovingToolAndFallsAfterDetach)
{
  auto response = Call<Attach>(attach);
  ASSERT_TRUE(response->success) << response->message;
  Step(2);
  const auto relative = tool_pose.Inverse() * object_pose;
  const auto start_x = tool_pose.Pos().X();
  const auto free_z = free_pose.Pos().Z();
  each_step = [](auto & ecm) {
    auto robot = ecm.EntityByComponents(components::Model(), components::Name("robot"));
    auto joint = sim::Model(robot).JointByName(ecm, "slide");
    auto command = ecm.template Component<components::JointVelocityCmd>(joint);
    if (command) {
      command->Data() = {0.5};
    } else {
      ecm.CreateComponent(joint, components::JointVelocityCmd({0.5}));
    }
  };
  Step(200);
  EXPECT_EQ(joints, 1u);
  EXPECT_GT(tool_pose.Pos().X(), start_x + 0.05);
  EXPECT_NEAR((tool_pose.Inverse() * object_pose).Pos().Distance(relative.Pos()), 0.0, 0.002);
  EXPECT_TRUE((tool_pose.Inverse() * object_pose).Rot().Equal(relative.Rot(), 0.002));
  EXPECT_LT(free_pose.Pos().Z(), free_z - 0.1);

  response.reset();
  auto detached = Call<Detach>(detach);
  ASSERT_TRUE(detached->success) << detached->message;
  const auto attached_z = object_pose.Pos().Z();
  Step(200);
  EXPECT_EQ(joints, 0u);
  EXPECT_LT(object_pose.Pos().Z(), attached_z - 0.1);
  EXPECT_TRUE(Call<Attach>(attach)->success);
  Step(5);
  EXPECT_EQ(joints, 1u);
  EXPECT_TRUE(Call<Detach>(detach)->success);
  Step(5);
  EXPECT_EQ(joints, 0u);
}

TEST_F(LinkAttacherTest, RejectsInvalidRequestsAndKeepsOtherAttachments)
{
  EXPECT_FALSE(Call<Attach>(attach, "missing")->success);
  EXPECT_FALSE(Call<Attach>(attach, "robot", "missing")->success);
  EXPECT_FALSE(Call<Attach>(attach, "robot", "tool", "missing")->success);
  EXPECT_FALSE(Call<Attach>(attach, "robot", "tool", "object", "missing")->success);
  EXPECT_FALSE(Call<Attach>(attach, "robot", "tool", "robot", "base")->success);
  EXPECT_FALSE(Call<Attach>(attach, "robot", "tool", "ground_plane", "ground")->success);
  EXPECT_FALSE(Call<Detach>(detach)->success);
  ASSERT_TRUE(Call<Attach>(attach)->success);
  EXPECT_FALSE(Call<Attach>(attach)->success);
  EXPECT_FALSE(Call<Attach>(attach, "box_free", "link")->success);
  EXPECT_FALSE(Call<Attach>(attach, "object", "link", "robot", "tool")->success);
  ASSERT_TRUE(Call<Attach>(attach, "robot", "tool", "box_free", "link")->success);
  Step(5);
  EXPECT_EQ(joints, 2u);
  EXPECT_FALSE(Call<Detach>(detach, "robot", "base")->success);
  EXPECT_TRUE(Call<Detach>(detach)->success);
  EXPECT_FALSE(Call<Detach>(detach)->success);
  Step(5);
  EXPECT_EQ(joints, 1u);
  EXPECT_TRUE(Call<Detach>(detach, "robot", "tool", "box_free", "link")->success);
  Step(5);
  EXPECT_EQ(joints, 0u);
}

TEST_F(LinkAttacherTest, ReattachesAfterDetachDeleteAndRespawn)
{
  ASSERT_TRUE(Call<Attach>(attach)->success);
  Step(5);
  ASSERT_EQ(joints, 1u);
  ASSERT_TRUE(Call<Detach>(detach)->success);
  Step(5);
  before_step = [](auto & ecm) {
    const auto object = ecm.EntityByComponents(components::Model(), components::Name("object"));
    ecm.RequestRemoveEntity(object);
  };
  Step(5);
  EXPECT_EQ(joints, 0u);
  EXPECT_FALSE(Call<Detach>(detach)->success);
  before_step = [this](auto & ecm) {
    sdf::Root root;
    ASSERT_TRUE(root.Load(TEST_WORLD_PATH).empty());
    const auto * model = root.WorldByIndex(0)->ModelByName("object");
    ASSERT_NE(model, nullptr);
    sim::SdfEntityCreator creator(ecm, *event_manager);
    creator.SetParent(creator.CreateEntities(model), world);
  };
  Step(5);
  ASSERT_TRUE(Call<Attach>(attach)->success);
  Step(5);
  EXPECT_EQ(joints, 1u);
  EXPECT_TRUE(Call<Detach>(detach)->success);
  Step(5);
  EXPECT_EQ(joints, 0u);
}

TEST_F(LinkAttacherTest, ClearsAttachmentsOnWorldReset)
{
  ASSERT_TRUE(Call<Attach>(attach)->success);
  Step(5);
  ASSERT_EQ(joints, 1u);
  ASSERT_TRUE(fixture->Server()->Reset(0));
  Step(5);
  EXPECT_EQ(joints, 0u);
  EXPECT_FALSE(Call<Detach>(detach)->success);
  EXPECT_TRUE(Call<Attach>(attach)->success);
  Step(5);
  EXPECT_EQ(joints, 1u);
  EXPECT_TRUE(Call<Detach>(detach)->success);
  Step(5);
}
