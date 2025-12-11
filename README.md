
## Installation

This repository is aimed to be used with Tiago Harmonic workspace.

Don't clone the repo ! instead, import it from main repository found [here](https://github.com/clementPene/tiago_harmonic).

```bash
cd src 
vcs import . < tiago_harmonic/dependencies/lfc_dependencies.repos
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_DISABLE_FIND_PACKAGE_Doxygen=ON \
    --packages-select tiago_lfc \
                      linear_feedback_controller_msgs \
                      linear_feedback_controller
colcon build --symlink-install
```

## Usage

### Start Tiago, simulated in Gazebo Harmonic and add lfc controllers

You can use simple launch file to set up environment. 
Useful launch files can be found under directory tiago_lfc/launch

```bash
# launch tiago in gazebo
ros2 launch tiago_lfc tiago_gazebo.launch.py world_name:=house # or empty
# Load lfc controllers
ros2 launch tiago_lfc switch_to_lfc_controllers.launch.py
```

> [!note]
> 3 controllers are packaged together and are in chained mode.
> hardware -> Joint-state-estimator -> Linear-feedback-controller -> Passthrough-controller -> hardware

### Use high level launch file

Concreate application can be found in launch folder at the root of the package.

|**Launch file**                  | **Description**                                                                                     |
|---------------------------------|---------------------------------------------------------------------------------------------------|
|`pd_plus_controller.launch.py`   |Use pd plus exemple found in lfc package. **TODO :** find a beter way to wait for controllers spawn before launching pd_controller node



## Authors
* Clément Pène - pene.clement@gmail.com
* based on [AGIMUS](https://github.com/agimus-project) 