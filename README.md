
## Programming a Real Self-Driving Car Project

This project aim is to create software to controls real self-driving car. It is based on architecture which is provided by Udacity as a base for final [task](https://github.com/udacity/CarND-Capstone).


### Team members

| Team member      | e-mail                   |
| -------------    | -------------            |
| Andrey Zabegaev  | andronics@mail.ru        |
| Pavel Silin      | silinp@yandex.ru         |
| Jens Kutschera   | jenskutschera@gmail.com  |
| Assem Sabbah     | assem.sabbah@gmail.com   |


### Instalation and run instructions

The best and stable way to run this project would be to use [docker](https://www.docker.com/). Docker installation instruction can be found [here](https://docs.docker.com/engine/installation/).

Clone this [project repository](https://github.com/Spinch/CarND-Capstone)
```bash
git clone https://github.com/Spinch/CarND-Capstone.git
```

Got to project directory
```bash
cd ./CarND-Capstone
```

Build the docker container
```bash
docker build . -t capstone
```

Run the docker container
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it --name roskinetic capstone
```

If you would like to create another connection to this container you can use command:
```bash
docker exec -i -t roskinetic bash
```

In container make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

Run the [simulator](https://github.com/udacity/CarND-Capstone/releases) on host machine.


### System architecture

System components communication i based on [ROS](http://www.ros.org/) communication ideology. We have three main subsystems: perception, planing and control. Here is the system architecture diagram:

![System architecture][ros_architecture]

### Perception subsystem

This part will be written later.


### Planning subsystem

Planning subsystem has two nodes:

1. Waypoint Loader - loads waypoints from the file for all track.
1. Waypoint Updater - receives all waypoints and current car position and generate list with fixed number of next waypoints.

Also, waypoint updater takes into account if there are red or yellow traffic light ahead of car. It calculates distance to traffic light stop line and linear decrease speed from fixed distance to this line. As the result cars smoothly stops right before stop line if necessary.

We have reduced number of final waypoints from 200 to 50 as it makes CPU load less and free resources for other tasks.

### Control subsystem

The control subsystem has two nodes: `Waypoint Follower` and `DBW Node`. First node takes desired path waypoints as input and produce desired linear and angular velocities as input for the second node.

DBW Node takes desired velocities as input and outputs three control commands: throttle, brake and steering. It consists of 4 parts:

1. DBW_node.py - ROS wrapper for subscribing, publishing and getting parameters
1. twist_controller.py - main control logic
1. yaw_controller.py - steering controller, converts target angular and linear velocities to steering command
1. pid.py - PID controller, used for throttle/brake command

twist_controller uses pid controller for throttle/brake control and twist_controller for steering control. Next brake value converts from `m/s^2` to `N*m` by multiplication of vehicle mass and wheel radius. Also, if desired speed is less that 0.1 m/s than brake value is set to minimum of 700 N*m to hold car on place.

[ros_architecture]: ./imgs/ros-graph.png "System architecture"
