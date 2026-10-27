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


#ifndef LINKATTACHER_ROS2__GZ_LINK_ATTACHER_HPP_
#define LINKATTACHER_ROS2__GZ_LINK_ATTACHER_HPP_

#include <memory>
#include <gz/sim/System.hh>

namespace gz::sim::systems
{
class GzLinkAttacherPrivate;

class GzLinkAttacher : public System,
  public ISystemConfigure,
  public ISystemPreUpdate,
  public ISystemReset
{
public:
  GzLinkAttacher();
  ~GzLinkAttacher() override;

  void Configure(
    const Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    EntityComponentManager & ecm, EventManager & event_manager) override;
  void PreUpdate(const UpdateInfo & info, EntityComponentManager & ecm) override;
  void Reset(const UpdateInfo & info, EntityComponentManager & ecm) override;

private:
  std::unique_ptr<GzLinkAttacherPrivate> impl_;
};
}  // namespace gz::sim::systems

#endif  // LINKATTACHER_ROS2__GZ_LINK_ATTACHER_HPP_
