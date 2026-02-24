# onrobot_ros2

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](https://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![repo size](https://img.shields.io/github/repo-size/takuya-ki/onrobot_ros2)

OnRobot ROS2 packages

- ROS Humble Hawksbill node examples for OnRobot screw driver.

## Dependency (tested as a host machine)

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=22.04+LTS)
  - Docker 26.1.1
  - Docker Compose 2.27.0

## Installation

```bash
git clone git@github.com:takuya-ki/onrobot_ros2.git --recursive --depth 1 && cd onrobot_ros2 && COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel 
```

## Usage

1. Connect the cable between Compute Box and Tool Changer
2. Connect an ethernet cable between Compute Box and your computer
3. Build and run the docker environment
   - Create and start docker containers in the initially opened terminal
        ```bash
        docker compose up
        ```
   - Execute the container in another terminal
        ```bash
        xhost + && docker exec -it onrobot_humble_container bash
        ```
4. Build program files with the revised yaml
    ```bash
    cd /ros2_ws && colcon build --symlink-install --parallel-workers 1 && source install/setup.bash
    ```
5. Run a planning process in the container
   - Use byobu to easily command several commands  
        ```bash
        byobu
        ```
        - First command & F2 to create a new window & Second command ...
        - Ctrl + F6 to close the selected window

##### Display the robot's (visual and collision) models  
```bash
ros2 launch onrobot_sd_description display.launch.py
```
```bash
ros2 launch onrobot_2fg7_description display.launch.py
```
```bash
ros2 launch onrobot_rg2_description display.launch.py
```
```bash
ros2 launch onrobot_rg6_description display.launch.py
```

##### Run the server receiving motion commands in the real-world
```bash
ros2 launch onrobot_sd_tutorials server.launch.py
ros2 service call /onrobot_sd/move_shank onrobot_interfaces/srv/SetCommand "{command: '0.2'}"
ros2 service call /onrobot_sd/move_shank onrobot_interfaces/srv/SetCommand "{command: '0.5'}"
```
<img src="images/service.gif" height="200">  

## Contributors

We always welcome collaborators!

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)
