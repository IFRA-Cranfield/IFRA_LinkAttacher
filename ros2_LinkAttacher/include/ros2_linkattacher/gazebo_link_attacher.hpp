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
# IFRA-Cranfield (2023) Gazebo-ROS2 Link Attacher. URL: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.
*/

#ifndef GAZEBO_LINK_ATTACHER_HPP_
#define GAZEBO_LINK_ATTACHER_HPP_

#include <gazebo/common/Plugin.hh>
#include <memory>

struct JointSTRUCT
{
  std::string model1;
  gazebo::physics::ModelPtr m1;
  std::string link1;
  gazebo::physics::LinkPtr l1;
  std::string model2;
  gazebo::physics::ModelPtr m2;
  std::string link2;
  gazebo::physics::LinkPtr l2;
  gazebo::physics::JointPtr joint;
};

namespace gazebo_ros
{

class GazeboLinkAttacherPrivate;

class GazeboLinkAttacher : public gazebo::WorldPlugin
{
public:
  
  // Constructor:
  GazeboLinkAttacher();

  // Destructor:
  virtual ~GazeboLinkAttacher();

  // LOAD plugin:
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

private:

  std::unique_ptr<GazeboLinkAttacherPrivate> impl_;

};

}  // namespace gazebo_ros

#endif  // GAZEBO_LINK_ATTACHER_HPP_