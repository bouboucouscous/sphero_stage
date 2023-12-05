# Controlling a Swarm of Robots

<div align="center">
<img width=1000px src="resources/figures/reception.jpeg" alt="explode"></a>
</div>

<h3 align="center"> Controlling a Swarm of Robots </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-green" alt="explode"></a>
<img width=90px src="https://img.shields.io/badge/team-MRS-yellow" alt="explode"></a>
</div>

## Table of Contents

* [Table of Contents](#table-of-contents)
* [Requirements](#requirements)
* [Project Goal](#project-goal)
* [Launcher](#launcher)
* [Explanation of Files and Directories](#explanation-of-files-and-directories)
* [Team](#team)

## Requirements

The basic requirements to do this project are:

- ROS
- Stage simulator

## Project Goal

The aim of this project is to create a ROS application in order to control a swarm of robots in a Simulator Stage using the Reynolds Rules.

The instructions are the following:

Implement basic Reynolds rules:
   1. separation
   2. alignment
   3. cohesion

Test the implementation in the Stage simulator in the map with a frame.

2. Implement additional Reynolds rules for:
   1. navigation to a given point
   2. obstacle avoidance

Test the implementation in the Stage simulator in the following maps:
   1. a map with a frame (useful to keep boids inside certain space)
   2. a map with different types of obstacles (useful to test obstacle avoidance rules)
   3. a simple maze map (useful to test the combination of obstacle avoidance and navigation rules)
   4. a hard maze map (useful to test the combination of obstacle avoidance and navigation rules)

## Launcher

-----------------------------------------------------------------------
name.launch:
``` 

```
-----------------------------------------------------------------------

## Explanation of Files and Directories

**`/sphero_stage/example.yml`:**
- Implementation for display in a multiple terminal environment (tmux).

**`/sphero_stage/package.xml`:**
- Project information, including project name, description, creator, maintainer, license, dependencies, and export details.

**`/sphero_stage/CMakeLists.txt`:**
- Package management and configuration.

**`/sphero_stage/launch/launch_params.yaml`:**
- Configuration options for launching:
- Number of robots: `num_of_robots`
- Map selection: `map_name`
- Robot formation: `distribution`

**`/sphero_stage/launch/start.py/`:**
- Create world
- Launch stage simulator

**`/sphero_stage/resources/`:**
- Do not modify unless adding new maps.
- Contains various maps used in the simulation.

**`/sphero_stage/src/sphero_stage/`:**
- Place Python files developed for the execution of movements and/or behaviors here.

## Team

- [Sofía Perales](https://github.com/sofiaprlsd)
