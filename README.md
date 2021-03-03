# Large Step-Ups Planner

This repo contains the code used to generate the trajectories for large step-ups. It consists of a library which exploits [``CasADi``](https://web.casadi.org/) to solve an optimal control problem, plus a [``ROS2``](https://index.ros.org/doc/ros2/) node to communicate with the robot.

## Citing
This code is related to the paper "Non-Linear Trajectory Optimization for Large Step-Ups: Application to the Humanoid Robot Atlas".
Paper: https://ieeexplore.ieee.org/document/9341587
Arxiv: https://arxiv.org/abs/2004.12083

To cite this work, please add the following to your publication
```
S. Dafarra, S. Bertrand, R. J. Griffin, G. Metta, D. Pucci and J. Pratt, "Non-Linear Trajectory Optimization for Large Step-Ups: Application to the Humanoid Robot Atlas," 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Las Vegas, NV, USA, 2020, pp. 3884-3891, doi: 10.1109/IROS45743.2020.9341587.
```
Bibtex:
```
@INPROCEEDINGS{9341587,  author={S. {Dafarra} and S. {Bertrand} and R. J. {Griffin} and G. {Metta} and D. {Pucci} and J. {Pratt}},  booktitle={2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},   title={Non-Linear Trajectory Optimization for Large Step-Ups: Application to the Humanoid Robot Atlas},   year={2020},  volume={},  number={},  pages={3884-3891},  doi={10.1109/IROS45743.2020.9341587}}
```

## Dependencies
- [``CasADi``](https://web.casadi.org/)
- [``Ipopt``](https://github.com/coin-or/Ipopt)
- [``cmake``](https://cmake.org/)
- A C++ compiler

If the CMake option ``BUILD_INTERFACE`` is ON (it is ON by default), you also need
- [``ihmc-open-robotics-software``](https://bitbucket.ihmc.us/projects/LIBS/repos/ihmc-open-robotics-software/browse)
- [``ROS2``](https://index.ros.org/doc/ros2/) (Tested with Crystal Clemmys)

## Linux Installation Instructions

The following instructions have been tested on Ubuntu 16.04. It is also assumed that no other ``ROS2`` installation is already present on the system.

### Install system dependencies
Run the following command to install ``gcc``, ``cmake`` and ``Ipopt`` from ``apt``.
```
sudo apt install gcc g++ gfortran git cmake liblapack-dev pkg-config coinor-libipopt-dev --install-recommends
```

### Install CasADi
(These instructions have been adapted from https://github.com/casadi/casadi/wiki/InstallationLinux#building-casadi-from-sources)

- Move to the directory where you want to download ``CasADi``, e.g. ``~/dev/large_step_ups``.
```
cd ~/dev/large_step_ups
```

- Clone the repository
```
git clone https://github.com/casadi/casadi.git
```

- Install ``CasADi``.
In order to avoid polluting system directories, it is suggested to specify an installation directory different from ``/usr/local``. In this case we create an ``install`` folder inside the ``build`` folder.
```
mkdir build
cd build
mkdir install
export NEW_INSTALL_DIR=$(pwd)/install
cmake -DWITH_IPOPT=ON -DCMAKE_INSTALL_PREFIX=$NEW_INSTALL_DIR ..
make install -j4
```

- Modify the ``.bashrc`` exporting the ``casadi_DIR`` variable. In this way ``CasADi`` can be easily found by ``cmake`` projects. Add the following line.
```
casadi_DIR=/path/to/casadi/install
```
In our case
```
casadi_DIR=~/dev/large_step_ups/casadi/build/install
```
It is then necessary to source the modified ``.bashrc`` in order to apply the changes.


### Install ``ROS2``
(These instructions have been adapted from https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Development-Setup/)

- Install ``ROS2`` system dependencies
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-lark-parser \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget
```
```
python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools
```
```
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
```

- Create a ``ROS2`` workspace and retrieve the code.
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
wget https://raw.githubusercontent.com/ros2/ros2/crystal/ros2.repos
vcs import src < ros2.repos
```

- Install ``ROS2`` dependencies via ``rosdep``
```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro crystal -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 python3-lark-parser rti-connext-dds-5.3.1 urdfdom_headers"
python3 -m pip install -U lark-parser
```

- Compile ``ROS2``
```
cd ~/ros2_ws
colcon build --symlink-install --packages-ignore qt_gui_cpp rqt_gui_cpp
```

### Compile the controller messages C++ interface
Here we assume that the following messages are available in the ``ihmc-open-robotics-software/ihmc-interfaces/src/main/messages/ihmc_interfaces/controller_msgs/msg`` folder:
- ``StepUpPlannerControlElement.msg``
- ``StepUpPlannerCostWeights.msg``
- ``StepUpPlannerErrorMessage.msg``
- ``StepUpPlannerParametersMessage.msg``
- ``StepUpPlannerPhase.msg``
- ``StepUpPlannerPhaseParameters.msg``
- ``StepUpPlannerPhaseResult.msg``
- ``StepUpPlannerRequestMessage.msg``
- ``StepUpPlannerRespondMessage.msg``
- ``StepUpPlannerStepParameters.msg``
- ``StepUpPlannerVector2.msg``

If not, you may checkout the branch ``feature/largeStepUps`` of ``ihmc-open-robotics-software``. Now we assume the ``ihmc-open-robotics-software`` to be in the path ``~/dev/atlas/ihmc-open-robotics-software/``.

- Create a ``controller_msgs`` workspace
```
mkdir -p ~/dev/large_step_ups/controller_msgs_ws/src
```

- Create a symbolic link to the ``controller_msgs`` folder
```
ln -s ~/dev/atlas/ihmc-open-robotics-software/ihmc-interfaces/src/main/messages/ihmc_interfaces/controller_msgs/ ~/dev/large_step_ups/controller_msgs_ws/src/controller_msgs
```
Alternatively, it is possible to exploit the [``ihmc_interfaces``](https://bitbucket.ihmc.us/projects/ROS/repos/ihmc_interfaces/browse) repo. In this case it would be enough to clone the repo in the ``~/dev/large_step_ups/controller_msgs_ws/src`` folder (make sure that the messages listed above are available).

- Compile the message workspace
```
cd ~/dev/large_step_ups/controller_msgs_ws
source ~/ros2_ws/install/local_setup.sh
colcon build --symlink-install
```

### Clone and compile this repo
- Clone this repo
```
cd ~/dev/large_step_ups
git clone https://bitbucket.ihmc.us/scm/icsl/large-step-ups-planner.git
```

- Source the ROS setup files to make sure that all the enviromental variables are set.
```
source ~/ros2_ws/install/local_setup.sh
source ~/dev/large_step_ups/controller_msgs_ws/install/local_setup.sh
```

- Compile the code
```
cd large-step-ups-planner/
mkdir build
cmake ../
make
```

## Run the responder node.
This node waits for a ``StepUpPlannerParametersMessage`` and a  ``StepUpPlannerRequestMessage``. The first is used to setup the planner. It returns a ``StepUpPlannerErrorMessage`` as an acknowledgement. Depending on the error code, the parameters may be have been successfully set or refused. Some info are provided in the message about what is wrong.
After the parameters are set, it is possible to send requests. The node will respond with a ``StepUpPlannerRespondMessage`` containing the solution.

In order to run, the node needs all the ROS2 variables set up. To this end, it is suggested to add the following alias to the ``.bashrc``
```
alias LARGE_STEP_UPS_SETUP='export ROS_DOMAIN_ID=8 && source ~/ros2_ws/install/local_setup.sh && source ~/dev/large_step_ups/controller_msgs_ws/install/local_setup.sh && export PATH=$PATH:~/dev/large_step_ups/large-step-ups-planner/build/bin'
```

In this way, it is possible to run the node with the following commands:
```
LARGE_STEP_UPS_SETUP
step_up_planner_responder
```
The output should look like
```
[INFO] [StepUpPlannerResponder]: Running...

```
### ``StepUpPlannerParametersMessage``
It is necessary to send a ``StepUpPlannerParametersMessage`` the first time a trajectory has to be computed or in case the following values need to change:
- number of phases
- phase settings
    - phase type
    - foot vertexes
    - foot scale
    - center offset
- number of instants per phase
- solver verbosity
- max leg length parameter
- min leg length parameter
- ``Ipopt`` linear solver
- portion of the final phase used to weight the final error
- static friction coefficient
- torsional friction coefficient
- cost weights
- whether to include or not the `CenterOfMassTrajectoryMessage`, the `PelvisHeightTrajectoryMessage` and/or the `FootstepDataListMessage` into the solution message.
- whether or not to send directly to the walking controller he `CenterOfMassTrajectoryMessage`, the `PelvisHeightTrajectoryMessage` and/or the `FootstepDataListMessage`, and their respective topic name.
- delta to be used when creating a `PelvisHeightTrajectoryMessage` from the CoM height profile.
- number of data points per message.

In particular, when sending or including the `CenterOfMassTrajectoryMessage` and the `PelvisHeightTrajectoryMessage` a different message per phase is created. Multiple messages per phase are created if the number of data points per message is lower than the number of instants per phase.

If a  ``StepUpPlannerParametersMessage`` has already been sent and successfully set, and none of the above parameters need to change, then it is not necessary to send such message again.

### ``StepUpPlannerRequestMessage``
Is the message to be sent every time a new trajectory has to be computed.

# Tests on the robot.
Tests on the robot have been performed by launching the node as specified above and by launching the ``AtlasStepUpPlannerDemo`` defined in ``ihmc-open-robotics-software``.
