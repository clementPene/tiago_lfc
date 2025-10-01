
## Installation

This repository is aimed to be used with Tiago Harmonic workspace. You can find main repository [here](https://github.com/Tiago-Harmonic/tiago_harmonic). 

Clone the repo inside your tiago workspace and build it with colcon

```bash
# colcon build --symlink-install --packages-select tiago_lfc
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_DISABLE_FIND_PACKAGE_Doxygen=ON
```

## Usage

### Start Tiago, simulated in Gazebo Harmonic
```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=house # or empty
```
### Switch controllers
```bash 
ros2 launch tiago_lfc launch_controllers.launch.py 
```
It will deactivate current arm_controller and activate torque control with a linear feedbakc controller. 
> [!note]
> 3 controllers are packaged together and are in chained mode.
> hardware -> Joint-state-estimator -> Linear-feedback-controller -> Passthrough-controller -> hardware


## Tiago Configuration

I modified tiago robot conf : tiago_description/config/tiago_configuration.yaml
and launch_pal/robot_arguments/common.py

I also modified default settings : 

## Authors
* Clément Pène - pene.clement@gmail.com