# Robot systems laboratory homework
This is the repo for our custom node made for our laboratory homework.

The homework consist of three main parts:
- the project builds upon a robotnik rb1 base robot which we extended with an upper structure and UH sensors according to a real robot being developed at ELTE's ethology department
- we made a custom map for the project based on the test environment the robot is used in
- we made a custom node that is capable of extending the robot's move_base with basic waypoint navigation

## Dependencies (to run the simulation)
### Our stuff
- rb1_base_common (our fork) [link](https://github.com/PhylMacao/rb1_base_common)
- rb1_base_sim (our fork) [link](https://github.com/PhylMacao/rb1_base_sim)

### Original robotnik packages
- robot_localization_utils [link](https://github.com/RobotnikAutomation/robot_localization_utils)
- robotnik_base_hw_sim [link](https://github.com/RobotnikAutomation/robotnik_base_hw_sim)
- robotnik_sensors [link](https://github.com/RobotnikAutomation/robotnik_sensors)
- robotnik_msgs [link](https://github.com/RobotnikAutomation/robotnik_msgs)

In the workspace install the packages dependencies:
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```

## How to run project
Launch the modified rb-1 base simulation.

```bash
roslaunch rb1_base_sim_bringup rb1_base_complete.launch
```

Run the path_publisher node manually:
```bash
rosrun robotlab_hw path_publisher.py
```