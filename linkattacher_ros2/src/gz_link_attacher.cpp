/*
# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: May, 2023.                                                                     #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) IFRA Gazebo-ROS 2 Link Attacher. URL: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.
*/


#include "linkattacher_ros2/gz_link_attacher.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gz::sim::systems
{
class GzLinkAttacherPrivate
{
public:
  struct Attachment
  {
    Entity model1;
    Entity link1;
    Entity model2;
    Entity link2;
    Entity joint;
  };

  ~GzLinkAttacherPrivate()
  {
    if (context) {
      context->shutdown("LinkAttacher unloaded");
    }
  }

  template<typename Request, typename Response>
  void Process(const Request & req, Response & res, bool attach)
  {
    // Only called by spin_some() in PreUpdate, on Gazebo's simulation thread.
    auto & manager = *ecm;
    const auto model1 = manager.EntityByComponents(
      components::Model(), components::Name(req.model1_name),
      components::ParentEntity(world));
    const auto model2 = manager.EntityByComponents(
      components::Model(), components::Name(req.model2_name),
      components::ParentEntity(world));
    if (model1 == kNullEntity || model2 == kNullEntity) {
      res.message = "Model not found: " +
        (model1 == kNullEntity ? req.model1_name : req.model2_name);
      return;
    }
    const auto link1 = Model(model1).LinkByName(manager, req.link1_name);
    const auto link2 = Model(model2).LinkByName(manager, req.link2_name);
    if (link1 == kNullEntity || link2 == kNullEntity) {
      res.message = "Link not found: " +
        (link1 == kNullEntity ? req.link1_name : req.link2_name);
      return;
    }

    auto existing = std::find_if(attachments.begin(), attachments.end(),
      [&](const Attachment & a) {return a.link1 == link1 && a.link2 == link2;});
    if (!attach) {
      if (existing == attachments.end()) {
        res.message = "No attachment exists for the requested link pair.";
        return;
      }
      manager.RequestRemoveEntity(existing->joint);
      attachments.erase(existing);
      res.success = true;
      res.message = "DETACHED: " + req.model1_name + "::" + req.link1_name +
        " -- " + req.model2_name + "::" + req.link2_name;
      return;
    }

    if (model1 == model2) {
      res.message = "Attachment requires two different models.";
      return;
    }
    if (existing != attachments.end()) {
      res.message = "The requested links are already attached.";
      return;
    }
    if (Model(model2).Static(manager)) {
      res.message = "The child model must be dynamic (<static>false</static>).";
      return;
    }
    // DART detachable joints require a tree of models. A child can have one
    // parent, and attaching an ancestor under its descendant would form a loop.
    for (const auto & a : attachments) {
      if (a.model2 == model2) {
        res.message = "The child model already has an attachment; detach it first.";
        return;
      }
    }
    auto ancestor = model1;
    while (ancestor != kNullEntity) {
      if (ancestor == model2) {
        res.message = "Attachment would create a kinematic loop.";
        return;
      }
      auto parent = std::find_if(attachments.begin(), attachments.end(),
        [&](const Attachment & a) {return a.model2 == ancestor;});
      ancestor = parent == attachments.end() ? kNullEntity : parent->model1;
    }

    const auto joint = manager.CreateEntity();
    manager.CreateComponent(joint, components::DetachableJoint({link1, link2, "fixed"}));
    attachments.push_back({model1, link1, model2, link2, joint});
    res.success = true;
    res.message = "ATTACHED: " + req.model1_name + "::" + req.link1_name +
      " -- " + req.model2_name + "::" + req.link2_name;
  }

  Entity world{kNullEntity};
  EntityComponentManager * ecm{nullptr};
  std::vector<Attachment> attachments;
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  rclcpp::Service<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_service;
  rclcpp::Service<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_service;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
};

GzLinkAttacher::GzLinkAttacher()
: impl_(std::make_unique<GzLinkAttacherPrivate>())
{
}

GzLinkAttacher::~GzLinkAttacher() = default;

void GzLinkAttacher::Configure(
  const Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
  EntityComponentManager & ecm, EventManager &)
{
  if (!World(entity).Valid(ecm)) {
    throw std::runtime_error("GzLinkAttacher must be loaded as a world plugin.");
  }
  impl_->world = entity;
  // Own the ROS context so unloading this plugin never shuts down other ROS
  // plugins in the Gazebo process, and no global signal handler is installed.
  impl_->context = std::make_shared<rclcpp::Context>();
  impl_->context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(impl_->context);
  std::string ros_namespace;
  if (sdf && sdf->HasElement("ros")) {
    auto ros = sdf->Clone()->GetElement("ros");
    ros_namespace = ros->Get<std::string>("namespace", "").first;
  }
  impl_->node = std::make_shared<rclcpp::Node>("gz_link_attacher", ros_namespace, options);
  impl_->attach_service = impl_->node->create_service<linkattacher_msgs::srv::AttachLink>(
    "ATTACHLINK", [this](
      const linkattacher_msgs::srv::AttachLink::Request::SharedPtr req,
      linkattacher_msgs::srv::AttachLink::Response::SharedPtr res)
    {impl_->Process(*req, *res, true);});
  impl_->detach_service = impl_->node->create_service<linkattacher_msgs::srv::DetachLink>(
    "DETACHLINK", [this](
      const linkattacher_msgs::srv::DetachLink::Request::SharedPtr req,
      linkattacher_msgs::srv::DetachLink::Response::SharedPtr res)
    {impl_->Process(*req, *res, false);});
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = impl_->context;
  impl_->executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
  impl_->executor->add_node(impl_->node);
  RCLCPP_INFO(impl_->node->get_logger(), "LinkAttacher ready: %s and %s",
    impl_->attach_service->get_service_name(), impl_->detach_service->get_service_name());
}

void GzLinkAttacher::PreUpdate(const UpdateInfo &, EntityComponentManager & ecm)
{
  // Remove stale records when a model/link has been deleted. Newly spawned
  // models with the same name have new entity IDs and can be attached safely.
  auto & attachments = impl_->attachments;
  attachments.erase(std::remove_if(attachments.begin(), attachments.end(),
    [&](const GzLinkAttacherPrivate::Attachment & a) {
      if (!ecm.HasEntity(a.link1) || !ecm.HasEntity(a.link2) || !ecm.HasEntity(a.joint)) {
        if (ecm.HasEntity(a.joint)) {
          ecm.RequestRemoveEntity(a.joint);
        }
        return true;
      }
      return false;
    }), attachments.end());

  // Execute callbacks here, even while paused. All ECM access stays on this
  // thread; service callbacks never wait on a future or a background thread.
  impl_->ecm = &ecm;
  impl_->executor->spin_some();
  impl_->ecm = nullptr;
}

void GzLinkAttacher::Reset(const UpdateInfo &, EntityComponentManager & ecm)
{
  for (const auto & a : impl_->attachments) {
    if (ecm.HasEntity(a.joint)) {
      ecm.RequestRemoveEntity(a.joint);
    }
  }
  impl_->attachments.clear();
}
}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::GzLinkAttacher, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate, gz::sim::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::GzLinkAttacher, "gz::sim::systems::GzLinkAttacher")
