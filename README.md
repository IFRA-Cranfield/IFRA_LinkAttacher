<!--
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
# IFRA-Cranfield (2023) Gazebo-ROS 2 Link Attacher. URL: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.
-->

<div id="top"></div>



<br />
<div align="center">

  <h2 align="center">IFRA_LinkAttacher - Gazebo-ROS 2 Plugin</h2>

  <p align="center">
    IFRA (Intelligent Flexible Robotics and Assembly) Group
    <br />
    Centre for Robotics and Assembly
    <br />
    Cranfield University
  </p>
</div>

<br />

## ABOUT

### Intelligent Flexible Robotics and Assembly Group

The IFRA (Intelligent Flexible Robotics and Assembly) Group is part of the Centre for Robotics and Assembly at Cranfield University.

IFRA Group pushes technical boundaries. At IFRA we provide high tech automation & assembly solutions, and we support smart manufacturing with Smart Industry technologies and solutions. Flexible Manufacturing Systems (FMS) are a clear example. They can improve overall operations and throughput quality by adapting to real-time changes and situations, supporting and pushing the transition towards flexible, intelligent and responsive automation, which we continuously seek and support.

The IFRA Group undertakes innovative research to design, create and improve Intelligent, Responsive and Flexible automation & assembly solutions, and this series of GitHub repositories provide background information and resources of how these developments are supported.

__SOCIAL MEDIA__:

IFRA-Cranfield:
- YouTube: https://www.youtube.com/@IFRACranfield
- LinkedIn: https://www.linkedin.com/in/ifra-cranfield/

Centre for Robotics and Assembly:
- Instagram: https://www.instagram.com/cranfieldrobotics/
- Facebook: https://www.facebook.com/cranfieldunirobotics/
- YouTube: https://www.youtube.com/@CranfieldRobotics
- LinkedIn: https://www.linkedin.com/company/cranfieldrobotics/
- Website: https://www.cranfield.ac.uk/centres/centre-for-robotics-and-assembly 


### IFRA_LinkAttacher Repository

The IFRA_LinkAttacher repository contains three ROS 2 packages:

- `linkattacher_msgs`: the AttachLink and DetachLink service definitions.
- `linkattacher_ros2`: the Gazebo Harmonic world plugin and its environment hook.
- `linkattacher_test`: UR3/cube test assets.

The plugin has been designed with the main purpose of simulating object pick and place tasks using Robot Manipulators in Gazebo. The link attachment/detachment feature is available through a ROS 2 Service Server, which creates a temporary fixed joint between two links for the "ATTACH" request, and removes the previously generated joint for the "DETACH" service. Thanks to this functionality, objects can be easily attached to any end-effector and pick/place tasks can be simulated in Gazebo-ROS 2.

This version targets **Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic (gz-sim 8)**. It is a Gazebo Sim world system plugin using `components::DetachableJoint` to create a real fixed joint. ROS service callbacks run on the simulation thread during `PreUpdate`, including paused updates.

The migration was informed by [TrebezIA's Jazzy port](https://github.com/TrebezIA/IFRA_LinkAttacher/tree/jazzy), reviewed at commit `a68cbb9c9714955c970dc0fe4ca80289a0d1a3d5`. That fork uses pose commands and locks `box_*` models for its cargo application. This implementation keeps physical attachment and detachment: objects retain their current relative pose when attached, and gravity and collisions apply after detaching. Multiple objects may be attached simultaneously; unrelated models are never frozen.

The video below shows the original Gazebo Classic implementation.

__VIDEO: Pick and Place Task - ROS 2 Gazebo Simulation (UR3 + Robotiq 2f-85)__

[![Alt text](https://img.youtube.com/vi/t4L4VAfgqZw/0.jpg)](https://www.youtube.com/watch?v=t4L4VAfgqZw)

<br />

## INSTALLATION

Install dependencies and build from a terminal sourced for Jazzy:

```bash
cd ~/dev_ws/src
git clone -b jazzy https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
cd ~/dev_ws
colcon build
```

The build uses the ROS `gz_sim_vendor` and `gz_plugin_vendor` packages, with no hardcoded Gazebo include paths. Sourcing the workspace adds the installed plugin library directory to `GZ_SIM_SYSTEM_PLUGIN_PATH`. For the plugin alone, build `--packages-up-to linkattacher_ros2`. The plugin package contains no test targets or test assets; all testing belongs to `linkattacher_test`. The UR3 test uses the existing `ros2_SimRealRobotControl` workspace packages.

<br />

## USAGE

The plugin provides two ROS 2 services for attaching and detaching links:

```bash
# Attach an object to the robot's end effector.
ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink "{model1_name: 'model1', link1_name: 'link1', model2_name: 'model2', link2_name: 'link2'}"

# Detach the object.
ros2 service call /DETACHLINK linkattacher_msgs/srv/DetachLink "{model1_name: 'model1', link1_name: 'link1', model2_name: 'model2', link2_name: 'link2'}"
```

| Request field | Description |
| --- | --- |
| `model1_name` | Name of the robot model in Gazebo. |
| `link1_name` | Name of the robot's end-effector link. |
| `model2_name` | Name of the spawned object model in Gazebo. |
| `link2_name` | Name of the object's link. |

### Load the plugin in a Gazebo Harmonic world

Place this inside the `<world>` element of your SDF world file:

```xml
<plugin filename="gz_link_attacher_plugin" name="gz::sim::systems::GzLinkAttacher"/>
```

This replaces the Classic `libgazebo_link_attacher.so` / `gazebo_link_attacher` entry. The world must also load Harmonic's physics system, as in the included example. The ROS services are provided directly by the plugin; no `ros_gz_bridge` is needed for them.

An optional ROS namespace can be set per world (use a distinct namespace when running multiple worlds):

```xml
<plugin filename="gz_link_attacher_plugin" name="gz::sim::systems::GzLinkAttacher">
  <ros>
    <namespace>/cell1</namespace>
  </ros>
</plugin>
```

This exposes `/cell1/ATTACHLINK` and `/cell1/DETACHLINK`.

### UR3 and cube test

Use two terminals: Terminal 1 runs the simulation and MoveIt, and Terminal 2 runs the test commands. Add the world plugin entry shown above before launching. Choose one test option per fresh simulation to avoid spawning duplicate cubes.

#### Terminal 1 — Launch the UR3

```bash
ros2 launch ros2srrc_launch moveit2.launch.py package:=linkattacher_test config:=LATest_1
```

#### Terminal 2 — Run a test

##### Option 1 — Show attachment with a visible offset

This test keeps the cube away from the end effector so the attachment is easy
to see. Contact is not required: the plugin preserves the relative position
and orientation of the two links when attaching.

**1. Spawn the white cube.**

```bash
ros2 run ros2srrc_execution SpawnObject.py --package linkattacher_test --sdf WhiteCube.sdf --name WhiteCube --x 0.257 --y 0.363 --z 0.92
```

**2. Move the robot to the starting configuration.**

```bash
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 90.00, joint2: -90.00, joint3: 90.00, joint4: -90.00, joint5: -90.00, joint6: 0.00}, speed: 1.0}"
```

**3. Lower the end effector by 0.20 m.**

```bash
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.00, y: 0.00, z: -0.2}, speed: 1.0}"
```

**4. Attach the cube to `EE_LinkAttacher`.**

```bash
ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink "{model1_name: 'ur3', link1_name: 'EE_LinkAttacher', model2_name: 'WhiteCube', link2_name: 'WhiteCube'}"
```

**5. Raise the end effector by 0.20 m.** The cube should move up with the robot, maintaining its offset from the end effector.

```bash
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.00, y: 0.00, z: 0.2}, speed: 1.0}"
```

**6. Detach the cube.** It should fall under gravity.

```bash
ros2 service call /DETACHLINK linkattacher_msgs/srv/DetachLink "{model1_name: 'ur3', link1_name: 'EE_LinkAttacher', model2_name: 'WhiteCube', link2_name: 'WhiteCube'}"
```

##### Option 2 — Simulate a pick and release

This test places the cube on the table and moves the end effector to the cube before attaching it.

**1. Spawn the white cube.**

```bash
ros2 run ros2srrc_execution SpawnObject.py --package linkattacher_test --sdf WhiteCube.sdf --name WhiteCube --x 0.257 --y 0.363 --z 0.92
```

**2. Move the robot to the starting configuration.**

```bash
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 90.00, joint2: -90.00, joint3: 90.00, joint4: -90.00, joint5: -90.00, joint6: 0.00}, speed: 1.0}"
```

**3. Move the end effector above the cube.**

```bash
ros2 action send_goal -f /Robmove ros2srrc_data/action/Robmove "{type: 'LIN', speed: 1.0, x: 0.257, y: 0.363, z: 0.90, qx: 1.0, qy: 0.0, qz: 0.0, qw: 0.0}"
```

**4. Lower the end effector by 0.02 m.**

```bash
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.00, y: 0.00, z: -0.02}, speed: 0.1}"
```

**5. Attach the cube to `EE_LinkAttacher`.**

```bash
ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink "{model1_name: 'ur3', link1_name: 'EE_LinkAttacher', model2_name: 'WhiteCube', link2_name: 'WhiteCube'}"
```

**6. Raise the end effector by 0.20 m.** The cube should lift with the robot.

```bash
ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.00, y: 0.00, z: 0.2}, speed: 1.0}"
```

**7. Detach the cube.** It should fall back onto the table.

```bash
ros2 service call /DETACHLINK linkattacher_msgs/srv/DetachLink "{model1_name: 'ur3', link1_name: 'EE_LinkAttacher', model2_name: 'WhiteCube', link2_name: 'WhiteCube'}"
```

<br />

## License

<p>
  Intelligent Flexible Robotics and Assembly Group
  <br />
  Created on behalf of the IFRA Group at Cranfield University, United Kingdom
  <br />
  E-mail: IFRA@cranfield.ac.uk 
  <br />
  <br />
  Licensed under the Apache-2.0 License.
  <br />
  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
  <br />
  <br />
  <a href="https://www.cranfield.ac.uk/">Cranfield University</a>
  <br />
  School of Aerospace, Transport and Manufacturing (SATM)
  <br />
    <a href="https://www.cranfield.ac.uk/centres/centre-for-robotics-and-assembly">Centre for Robotics and Assembly</a>
  <br />
  College Road, Cranfield
  <br />
  MK43 0AL, Bedfordshire, UK
  <br />
</p>

<br />

## Cite our work

<p>
  You can cite our work with the following statement:
  <br />
  IFRA-Cranfield (2023) Gazebo-ROS 2 Link Attacher. URL: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.
</p>

<br />

## Contact

<p>
  Mikel Bueno Viso - Research Assistant in Intelligent Automation at Cranfield University
  <br />
  E-mail: Mikel.Bueno-Viso@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/mikel-bueno-viso/
  <br />
  Profile: https://www.cranfield.ac.uk/people/mikel-bueno-viso-32884399
  <br />
  <br />
  Dr. Seemal Asif - Lecturer in Artificial Intelligence and Robotics at Cranfield University
  <br />
  E-mail: s.asif@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/dr-seemal-asif-ceng-fhea-miet-9370515a/
  <br />
  Profile: https://www.cranfield.ac.uk/people/dr-seemal-asif-695915
  <br />
  <br />
  Professor Phil Webb - Professor of Aero-Structure Design and Assembly at Cranfield University
  <br />
  E-mail: p.f.webb@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/phil-webb-64283223/
  <br />
  Profile: https://www.cranfield.ac.uk/people/professor-phil-webb-746415 
  <br />
</p>