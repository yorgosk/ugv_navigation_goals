# ugv_navigation_goals

## About

This is a ROS package comprising of the rulah_navigation_goals ROS node software.
It was developed at the Spring of 2018 by **Georgios Kamaras** as a complementary
software to his undergraduate thesis in **Embodied Artificial Intelligence**. More
specifically, ugv_navigation_goals consists of an _offline path planner_ for incline
terrain navigation. This planner takes as input facts about a specific incline
terrain region, like degree of slope, the robot's start position and its goal
position and lethal obstacles that exist in this terrain region and outputs a vector
of waypoints that have been deduced as the _recommended_ for the robot to traverse
the incline terrain region with safety. This software package serves as a
_proof of concept_ for its authors proposal regarding the utilization of Bezier curves
to construct a smooth curved path from the robot's start position, on the start of
the slope, to its goal position, significantly higher in the slope.

## Usage

### Prerequisites

* To use this software to its full extend, your robot should run on the ROS middleware.
* Prior to launching the rulah_navigation_goals node you should have already launched
your robot's specific packages and especially its **move_base** node.

### Launch

Type:
> roslaunch rulah_navigation_goals rulah_navigation_goals.launch

### Experimental Validation

For the experimental validation of our work we used the [Clearpath Husky](https://www.clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/)
UGV in simulation and we created three simulation environments, which can be found
on the address <https://github.com/yorgosk/husky_simulator>.

For the simulations, the [Husky Gazebo](https://github.com/yorgosk/husky_simulator)
ROS package was used, with the husky\_roboskel\_playground\_world.launch
launch file. The robot was placed in the start position manually, using the
teleop\_twist\_keyboard ROS package, and its move\_base was initialized using the
[Husky Navigation](https://github.com/husky/husky/tree/kinetic-devel/husky_navigation)
ROS package, with the launch file move\_base\_mapless\_demo.launch. We used the
present ugv_navigation_goals package by calling the _Rulah Navigation Goals_
ROS node with the rulah\_navigation\_goals.launch launch file.

## Contact & Feedback

* Georgios Kamaras
