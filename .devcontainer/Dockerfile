ARG VARIANT=ubuntu-22.04
FROM mcr.microsoft.com/vscode/devcontainers/base:0-${VARIANT}

# --- Configure environment ---

# Configure DEBIAN_FRONTEND to avoid warnings
ENV DEBIAN_FRONTEND=noninteractive

# set up work directory
WORKDIR /home/vscode/workspace

# --- Install ROS2 ---

# install locales for UTF-8 support
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# set locale environment variables
ENV LANG=en_US.UTF-8

# add ROS2 repository
RUN apt-get update && \
    apt-get install -y software-properties-common curl git && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install ROS2 dependencies
RUN apt-get update && apt-get install -y \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-vcstool \
    python3-rosdep \
    ros-dev-tools

# install ROS2
RUN mkdir -p /home/vscode/ros2_humble/src && \
    cd /home/vscode/ros2_humble && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src && \
    sudo rosdep init && \
    rosdep update && \
    apt-get update && \
    apt-get -y upgrade && \
    cd /home/vscode/ros2_humble && \
    rosdep install --rosdistro humble --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" && \
    cd /home/vscode/ros2_humble && \
    colcon build --symlink-install

# --- Install ROS2 tracing ---

# install ROS2 tracing
RUN apt-get install -y lttng-tools liblttng-ust-dev && \
    apt-get install -y python3-babeltrace python3-lttng && \
    apt-get install -y lttng-modules-dkms

# set up user for tracing
RUN groupadd -f -r tracing && \
    usermod -a -G tracing vscode

RUN cd /home/vscode/ros2_humble && \
    colcon build --packages-up-to tracetools --cmake-force-configure

# download ROS2 trace analysis
RUN mkdir -p /home/vscode/ros2_tracing_ws/src/tracetools_analysis && \
    git clone -b humble https://github.com/ros-tracing/tracetools_analysis.git /home/vscode/ros2_tracing_ws/src/tracetools_analysis

# build ROS2 tracing
RUN cd /home/vscode/ros2_tracing_ws &&\
    . /home/vscode/ros2_humble/install/setup.sh &&\
    colcon build --symlink-install --allow-overriding tracetools
    

RUN pip install bokeh jupyter psrecord

# --- Set up user ---

# set user
USER vscode

# source ROS2 setup file
RUN echo ". /home/vscode/ros2_humble/install/setup.sh" >> ~/.bashrc

# source ROS2 tracing setup file
RUN echo '. /home/vscode/ros2_tracing_ws/install/setup.sh' >> /home/vscode/.bashrc

# configure environment variables
ENV DISPLAY=unix:1
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_DOMAIN_ID=0

# --- Set up entrypoint ---

SHELL ["/bin/bash", "-c"]