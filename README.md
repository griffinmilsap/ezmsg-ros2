# ezmsg-ros2

Griffin Milsap / Johns Hopkins University Applied Physics Lab 2024

Devcontainer setup inspired by [Allison Thackston](https://www.allisonthackston.com/articles/vscode-docker-ros2.html)

## EZMSG ❤️ ROS: A match made in heaven
Manifesto goes here.

## Devcontainer and Examples
ROS is a bit difficult to get started with.  This repository comes with a VSCode `devcontainer` that should hopefully make the process a bit less painful.  This devcontainer runs in Docker, so you'll need an installation of Docker available, and the Docker extension installed in VSCode.

Simply [open this folder in a container](https://code.visualstudio.com/docs/devcontainers/containers#_quick-start-open-an-existing-folder-in-a-container) to get started with a ROS2 environment set up and ready to go.  From there, you can build and run the ROS `ezmsg_ros2_examples` package by running the following commands from the root of this repository:

``` bash
colcon build
source install/setup.bash
```

The included examples in the `ezmsg_ros2_examples` package include
* `sub`: a simple subscriber that subscribes to the `/chatter` topic and publishes received messages into an ezmsg graph (simply logging them for now.)  
  * To generate activity on the `/chatter` topic for this subscriber to consume, run the following in a separate terminal: `ros run demo_nodes_cpp talker`

* [more to come]



