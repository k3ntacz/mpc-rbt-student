# MPC-RBT Student

<div align="center">

[![Tests](https://github.com/Robotics-BUT/mpc-rbt-student/actions/workflows/test.yml/badge.svg?branch=main)](https://github.com/Robotics-BUT/mpc-rbt-student/actions/workflows/test.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

</div>

This repository is a template for all student solutions for lab tasks in the MPC-RBT course. The `main` branch implements a ROS 2 package `mpc_rbt_solution` that will serve as the main project for the duration of the MPC-RBT course. Other branches will contain templates for other introductory labs.

## Installation

Follow installation steps in the [MPC-RBT Simulator](https://github.com/Robotics-BUT/mpc-rbt-simulator) repository to install the required versions of Ubuntu, ROS, and Webots, and to prepare your workspace.

Create a fork of this repository, navigate to your created workspace directory (e.g., `mpc_rbt_ws`) and clone your fork into its `src` subdirectory.

Install any additional missing dependencies using the following command:

```
rosdep install --from-paths src -y -r --ignore-src --rosdistro humble
```

## Package Structure

>TODO

## Usage

Navigate to the workspace directory (`mpc_rbt_ws`) and build it:

```
colcon build
```

Set up the environment for the workspace:

```
source install/setup.bash
```

Launch all your solutions using:

```
ros2 launch mpc_rbt_solution solution.launch.py
```

## Testing

Navigate to the workspace directory (`mpc_rbt_ws`) and build it:

```
colcon build
```

Run tests using:

```
colcon test --ctest-args tests --packages-select mpc_rbt_solution
```

View the results using:
```
colcon test-result --verbose --all
```
