### Dockerfile for ROS Noetic with catkin workspace

# Enable X11 Access (On Localhost):
    #xhost +local:docker

# To Build (On Localhost):
    # docker build -t ros_noetic .

# To Run (On Localhost):
    # docker run -it --rm \
    # --env DISPLAY=$DISPLAY \
    # --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    # ros_noetic bash

# To Run (On Localhost):
    # docker exec -it ros_noetic bash    

# Source ROS (In Container):
    # source /opt/ros/noetic/setup.bash
    
# Test GUI (In Container):
    # xeyes
    #rosrun rqt_gui rqt_gui

# Build Workspace (In Container):
    # cd /root/catkin_ws
    # catkin_make
    # source devel/setup.bash
    
# Install Dependencies (In Container):
    # rosdep install --from-paths src --ignore-src -r -y
    
# To Run trackit_core Unit Tests (In Container):
    # rosrun trackit_core trackit_idk
    # rosrun trackit_core test_trackit_idk

# To Run trackit_jacobian Unit Tests (In Container):
    # roslaunch trackit_jacobian test_trackit_jacobian.launch

# Use the osrf/ros:noetic-desktop image
FROM osrf/ros:noetic-desktop

# Install dependencies
RUN apt update && apt install -y \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    git \
    gtest \
    ros-noetic-rostest \
    x11-apps \
    libgl1-mesa-glx \
    libx11-dev

# Initialize rosdep
RUN rosdep init && rosdep update

# Create catkin workspace
RUN mkdir -p /root/catkin_ws/src

# Set up the working directory
WORKDIR /root/catkin_ws

# Clone your repository
RUN cd /root/catkin_ws/src && \
    git clone https://github.com/Ryan-Stapleton/Docker.git

# Install dependencies using rosdep
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

# Set up the environment
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set default command to bash
CMD ["bash"]