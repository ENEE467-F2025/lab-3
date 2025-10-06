# ENEE 467 Fall 2025: Robotics Project Laboratory
## Lab 3: Introduction to Programming with ROS 2

This repository contains a Docker container for the third lab as well as the necessary code templates for completing the exercises.

## Overview

![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-orange)

To avoid software conflicts and increase portability, all lab software will be packaged as a Docker container. Follow the instructions below to get started.

## Building the Container

First check to see if the image is prebuilt on the lab computer by running the following command
```
docker image ls
```
If you see the image named `lab-3-image` in the list then you can skip the build process.

To build the Docker container, ensure that you have [Docker](https://www.docker.com/get-started/) installed and the Docker daemon running.
* Clone this repository and navigate to the `docker` folder
    ```
    cd ~/Labs
    git clone https://github.com/ENEE467-F2025/lab-3.git
    cd lab-3/docker
    ```
* Build the image with Docker compose
    ```
    userid=$(id -u) groupid=$(id -g) docker compose -f lab-3-compose.yml build
    ```

## Starting the Container

The lab computers contain a prebuild image so you will not have to build the image.
* Clone this repo to get the lab-3 code if you haven't done so already
    ```
    cd ~/Labs
    git clone https://github.com/ENEE467-F2025/lab-3.git
    cd lab-3/docker
    ```
* Enable X11 forwarding
    ```
    xhost +local:root
    ```
* Run the Docker container
    ```
    docker compose -f lab-3-compose.yml run --rm lab-3-docker
    ```
* Once inside the container, you should be greeted with the following prompt indicating that the container is running
    ```
    robot@docker-desktop:~$
    ```

## Lab Instructions

Please follow the [lab manual](Lab_3_Intro_to_ROS.pdf) closely. All instructions are contained inside the lab manual.

## MacOS Instructions

For information on running the container on MacOS with X11 forwarding, see [MacOS Instruction](macos.md).