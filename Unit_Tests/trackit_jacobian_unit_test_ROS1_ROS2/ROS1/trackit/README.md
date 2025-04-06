# TrackIt!

## Overview
The TrackIt! package is a ROS (Robot Operating System) package designed to provide real-time control capabilities for robotics applications. It offers a flexible framework for implementing and executing real-time control algorithms, enabling precise and responsive control of robotic systems and enabling research on intelligent robotic systems.

This package is intended for use in scenarios where real-time, reactive control is needed, such as human-robot interaction or control in dynamic environments. 

## Features
- Real-time control: The package is designed to achieve real-time, reactive control of robotic systems.
- ROS integration: It seamlessly integrates with other ROS nodes, allowing easy communication and coordination with the broader ROS network.
- Flexible architecture: The package provides a modular and extensible architecture that supports the implementation of various control algorithms and strategies.
- Hardware abstraction: It offers a convenient interface to interact with hardware components, such as sensors and actuators, providing a unified approach to control.


## Packages
The TrackIt! package is a meta package containing several subpackages.

### trackit_core
The `trackit_core` package contains the node responsible for inverse differential kinematics. This node calculates the joint velocity necessary to move the robot in response to a twist topic that the node subscribes to.

### trackit_jacobian
The  'trackit_jacobian' package contain nodes responsible for calculating the jacobian and the inverse.

### trackit_msgs
The `trackit_msgs` package contains custom ros messages for the TrackIt! framework.

### trackit_nullspace
The `trackit_nullspace` package contains nodes responsible for calculating the nullspace projection as well as additional tasks to be for the robot arm to consider.

### trackit_robot_state
The `trackit_robot_state` package contains nodes for obtaining joint limits of the robot to be provided to the trackit_core nodes.

### trackit_utils
The `trackit_utils` package contains convenient nodes such as nodes for filtering and transforming messages.

### trackit_velocity_commands
The `trackit_velocity_commands` package contains nodes that deal with creating twist commands for the robot end effector to follow as well as a sample node to send a joint velocity msg to a robot.

## Nodes

### trackit_core
- trackit_idk: 
  - Node responsible for inverse differential kinematics
  
- deprecated nodes: (older nodes which was converted into trackit_idk)
  - trackit_velocity control
  - trackit_velocity_control_limit
  - trackit_velocity_control_limit_mask
  
### trackit_jacobian
- trackit_jacobian:
  - Node responsible for calculating jacobian of the robot as well as the inverse. Has options for damping the inverse
- trackit_jacobian_edls:
  - Similar to trackit_jacobian with an added option for applying edls to the inverse
  
### trackit_nullspace
- nullspace_projection:
  - Calculates the nullspace projection based on the jacobian. Calculates the projected joint velocity to send to trackit_idk.
- task_joint_middle:
  - A sample task for nullspace projection with a goal of moving the joints of the robot towards the center of the joint space.
  
### trackit_robot_state
- joint_limit_aggregator:
  - A node to combine the various joint limit messages received to pass onto trackit_idk node. Selects the most restrictive limit based on a given time limit.
- joint_velocity_limit_publisher:
  - Publishes joint velocity limit based on a given time to reach the joint position limit.
- static_joint_limit_publisher:
  - Publishes joint position, velocity and effort limit based on values from the URDF.

### trackit_utils
- path_marker:
  - Publishes a visualization_msgs marker based on the end effector motion.
- tf_tracking_error:
  - Calculates the error between two transforms and calculates the twist necessary to move one transform to the other.
- tf_velocity:
  - Tracks a given tf frame and calculates the twist performed by the frame.
- twist_iir_lpf:
  - Applies a low pass filter for twist msgs.
- twist_transform:
  - Transforms a twist msg from one frame to another.
- wrench_iir_lpf:
  - Applies a low pass filter for wrench msgs.
- wrench_transform:
  - Transforms a wrench msg from one frame to another.
  
### trackit_velocity_commands
- admittance_twist_basic:
  - A basic node to calculate the admittance twist based on a wrench topic.
- admittance_twist_spring_mass_damper:
  - A node to calculate the admittance twist based on a spring mass damper model of the end effector.
- joy_to_twist:
  - A node to convert joy node output into a twist msg.
- pid_ff:
  - A PID node to track a tf frame with the end effector. Feed forward velocity can be included to improve tracking
- robot_command:
  - A sample node for adapting the output of trackit joint velocity msg to a generic float65 multiarray msg used by some robots. A deadman topic is included such that the joint velocities are only sent to the robot if the value is true. If false, zeroes are sent instead.

## Installation

### Prerequisites
- ROS (Robot Operating System): The package is built on ROS1 and requires a compatible ROS installation. Visit [ROS Wiki](http://wiki.ros.org/ROS/Installation) for installation instructions. This package has been tested with ROS Noetic

### Package Installation
1. Create a catkin workspace (if you don't have one):
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

2. Clone the Realtime Control Package into the catkin workspace:
```
cd ~/catkin_ws/src
git clone <package-repository-url>
```
3. Install the dependencies
```
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y
```
4. Build the package:
```
cd ~/catkin_ws
catkin_make
```
5. If necessary source the workspace:
```
source ~/catkin_ws/devel/setup.bash
```
Optionally, add the workspace to the bashrc:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


## Usage

### Launching the Package
To launch the Realtime Control Package, use the following command:
```
roslaunch realtime_control_package <launch_file.launch>
```

Replace `<launch_file.launch>` with the appropriate launch file provided by the package.

### Configuring the Control Algorithm
The Realtime Control Package offers configuration parameters to adjust the behavior of the control algorithm. These parameters can be modified in the launch file. Consult the package documentation or launch files for details on available parameters and their effects.

### Interacting with the Package
The package provides ROS topics and services for interaction. Refer to the package documentation or source code for a detailed description of the available interfaces and their usage.

## Documentation
For detailed information on the package, including API references, tutorials, and examples, please refer to the [Realtime Control Package Documentation](https://your-package-documentation-url).

## License
This package is licensed under the [MIT License](LICENSE).

## Acknowledgments
- Mention any acknowledgments or credits for libraries, frameworks, or other resources used in the package.

## Contributing
Contributions to the Realtime Control Package are welcome. Please follow the guidelines in [CONTRIBUTING.md](CONTRIBUTING.md) for details on how to contribute to the project.

## Contact
For questions, issues, or suggestions regarding the package, please contact:

- Your Name
- Your Email

You can also open an issue on the package's repository: [Link to Repository Issues](https://your-package-repository-url/issues)



## Getting started

To make it easy for you to get started with GitLab, here's a list of recommended next steps.

Already a pro? Just edit this README.md and make it your own. Want to make it easy? [Use the template at the bottom](#editing-this-readme)!

## Add your files

- [ ] [Create](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#create-a-file) or [upload](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#upload-a-file) files
- [ ] [Add files using the command line](https://docs.gitlab.com/ee/gitlab-basics/add-file.html#add-a-file-using-the-command-line) or push an existing Git repository with the following command:

```
cd existing_repo
git remote add origin https://code.research.uts.edu.au/102842/trackit.git
git branch -M main
git push -uf origin main
```

## Integrate with your tools

- [ ] [Set up project integrations](https://code.research.uts.edu.au/102842/trackit/-/settings/integrations)

## Collaborate with your team

- [ ] [Invite team members and collaborators](https://docs.gitlab.com/ee/user/project/members/)
- [ ] [Create a new merge request](https://docs.gitlab.com/ee/user/project/merge_requests/creating_merge_requests.html)
- [ ] [Automatically close issues from merge requests](https://docs.gitlab.com/ee/user/project/issues/managing_issues.html#closing-issues-automatically)
- [ ] [Enable merge request approvals](https://docs.gitlab.com/ee/user/project/merge_requests/approvals/)
- [ ] [Automatically merge when pipeline succeeds](https://docs.gitlab.com/ee/user/project/merge_requests/merge_when_pipeline_succeeds.html)

## Test and Deploy

Use the built-in continuous integration in GitLab.

- [ ] [Get started with GitLab CI/CD](https://docs.gitlab.com/ee/ci/quick_start/index.html)
- [ ] [Analyze your code for known vulnerabilities with Static Application Security Testing(SAST)](https://docs.gitlab.com/ee/user/application_security/sast/)
- [ ] [Deploy to Kubernetes, Amazon EC2, or Amazon ECS using Auto Deploy](https://docs.gitlab.com/ee/topics/autodevops/requirements.html)
- [ ] [Use pull-based deployments for improved Kubernetes management](https://docs.gitlab.com/ee/user/clusters/agent/)
- [ ] [Set up protected environments](https://docs.gitlab.com/ee/ci/environments/protected_environments.html)

***

# Editing this README

When you're ready to make this README your own, just edit this file and use the handy template below (or feel free to structure it however you want - this is just a starting point!). Thank you to [makeareadme.com](https://www.makeareadme.com/) for this template.

## Suggestions for a good README
Every project is different, so consider which of these sections apply to yours. The sections used in the template are suggestions for most open source projects. Also keep in mind that while a README can be too long and detailed, too long is better than too short. If you think your README is too long, consider utilizing another form of documentation rather than cutting out information.

## Name
Choose a self-explaining name for your project.

## Description
Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors.

## Badges
On some READMEs, you may see small images that convey metadata, such as whether or not all the tests are passing for the project. You can use Shields to add some to your README. Many services also have instructions for adding a badge.

## Visuals
Depending on what you are making, it can be a good idea to include screenshots or even a video (you'll frequently see GIFs rather than actual videos). Tools like ttygif can help, but check out Asciinema for a more sophisticated method.

## Installation
Within a particular ecosystem, there may be a common way of installing things, such as using Yarn, NuGet, or Homebrew. However, consider the possibility that whoever is reading your README is a novice and would like more guidance. Listing specific steps helps remove ambiguity and gets people to using your project as quickly as possible. If it only runs in a specific context like a particular programming language version or operating system or has dependencies that have to be installed manually, also add a Requirements subsection.

## Usage
Use examples liberally, and show the expected output if you can. It's helpful to have inline the smallest example of usage that you can demonstrate, while providing links to more sophisticated examples if they are too long to reasonably include in the README.

## Support
Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc.

## Roadmap
If you have ideas for releases in the future, it is a good idea to list them in the README.

## Contributing
State if you are open to contributions and what your requirements are for accepting them.

For people who want to make changes to your project, it's helpful to have some documentation on how to get started. Perhaps there is a script that they should run or some environment variables that they need to set. Make these steps explicit. These instructions could also be useful to your future self.

You can also document commands to lint the code or run tests. These steps help to ensure high code quality and reduce the likelihood that the changes inadvertently break something. Having instructions for running tests is especially helpful if it requires external setup, such as starting a Selenium server for testing in a browser.

## Authors and acknowledgment
Show your appreciation to those who have contributed to the project.

## License
For open source projects, say how it is licensed.

## Project status
If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers.
