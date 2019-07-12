# iiwa_examples
This repo contains applications developed for a [KUKA LBR iiwa 14 R820](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/lbr-iiwa).
## Developing environment
**[Ubuntu16.04](http://releases.ubuntu.com/16.04/)**
**[ROS-Kinetic](http://wiki.ros.org/kinetic)**
**[Catkin Command Line Tools](https://catkin-tools.readthedocs.io/en/latest/)**

# Installation
## Setup [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack)
Please refer to [this wiki](https://github.com/IFL-CAMP/iiwa_stack/wiki#setup-guide) for a more detailed setup guide.

## Setup [this repo](https://github.com/linZHank/iiwa_examples.git)
```console
cd ~/ros_ws/src
git clone https://github.com/linZHank/iiwa_examples.git
cd ..
catkin build
source ~/ros_ws/devel/setup.bash
```

# Quick Start
> Make sure the scripts are executable
`chmod +x ~/ros_ws/src/iiwa_examples/scripts/*.py`
`rosrun iiwa_examples joint_control_test`

Hey Look! Your iiwa is waving!
