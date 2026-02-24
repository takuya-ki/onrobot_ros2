FROM ubuntu:22.04

# setting environmental variables
ENV DEBIAN_FRONTEND noninteractive

# Timezone and Launguage
RUN apt update -y && apt install -y --no-install-recommends \
    software-properties-common \        
    locales \
    tzdata \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe

ENV LANG en_US.UTF-8
ENV TZ=Asia/Tokyo

RUN apt update -y && apt install -y --no-install-recommends \
    build-essential \
    libgl1-mesa-glx \
    ca-certificates \
    libglx-mesa0 \
    libxrender1 \
    lsb-release \
    python3-pip \
    python3-tk \
    mesa-utils \
    x11-utils \
    libxext6 \
    libsm6 \
    gnupg2 \
    byobu \
    curl \
    wget \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt upgrade -y && apt update -y && apt install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export XDG_RUNTIME_DIR=/tmp/runtime-futa" >> ~/.bashrc

# Generate bash commands
WORKDIR /ros2_ws/
RUN echo "colcon build --symlink-install --executor sequential" >> ~/.bash_history
RUN echo "ros2 launch onrobot_sd_description display.launch.py" >> ~/.bash_history
RUN echo "ros2 launch onrobot_2fg7_description display.launch.py" >> ~/.bash_history
RUN echo "ros2 launch onrobot_rg2_description display.launch.py" >> ~/.bash_history
RUN echo "ros2 launch onrobot_rg6_description display.launch.py" >> ~/.bash_history
RUN echo "ros2 launch onrobot_vg10_description display.launch.py" >> ~/.bash_history

RUN echo "ros2 launch onrobot_sd_tutorials server.launch.py" >> ~/.bash_history
RUN echo "ros2 service call /onrobot_sd/move_shank onrobot_interfaces/srv/SetCommand \"{command: '0.2'}\"" >> ~/.bash_history
RUN echo "ros2 service call /onrobot_sd/move_shank onrobot_interfaces/srv/SetCommand \"{command: '0.5'}\"" >> ~/.bash_history
RUN echo "byobu" >> ~/.bash_history