# ROS2 port of franka_ros for Franka Emika Panda (FER) robots

This project is a supposed to be a way of controlling legacy Franka Emike Panda robots using ros2.

This repository includes a custom version of libfranka that is supposed to be the latest version that works with the franka panda arm version 3.0.3. 

It has a few changes made to it to compile on Ubuntu 24.04 LTS and work with ROS2 Jazzy.

## Dependencies

Install Dependencies:

```bash
sudo apt-get install -y build-essential cmake git libpoco-dev  libfmt-dev
```
