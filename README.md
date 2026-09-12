# swri_console

![screenshot](doc/images/screenshot.png)

swri_console is an alternative to rqt_console for viewing the ROS console output.  swri_console was written to scale well to large systems with heavy console output.  It stays responsive under fire and allows users to quickly find information to track down problems.

## Features

- High performance; swri_console handles receiving thousands of logs per second and storing millions in memory while staying responsive
- Ctrl or shift-click to quickly select which nodes you want to monitor
- Hide or show log messages based on substring matches, or, if you need more power, regular expressions
- Hide, show, and colorize log messages based on severity
- Save and load log messages to text files
- Save and load log messages directly from the `/rosout` topic in a bag file
- Right-click on nodes to dynamically set their logger levels
   - *Requires ROS 2 Iron or newer*

## Supported ROS Distributions
The package currently supports all ROS 2 distributions up to `rolling`. Branches marked `*-eol` indicate a distribution that is at end of life status and is no longer updated.

Build Status
------------

### Branches

&nbsp; | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
Branch | [`humble`](https://github.com/swri-robotics/swri_console/tree/kilted-devel) | [`jazzy`](https://github.com/swri-robotics/swri_console/tree/kilted-devel) | [`kilted`](https://github.com/swri-robotics/swri_console/tree/kilted-devel) | [`lyrical`](https://github.com/swri-robotics/swri_console/tree/ros2-devel) | [`rolling`](https://github.com/swri-robotics/swri_console/tree/ros2-devel)

### Released Versions

&nbsp; | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
Version | [![ROS2 Humble](https://img.shields.io/ros/v/humble/swri_console.svg?style=flat-square)](https://index.ros.org/r/swri_console/#humble) | [![ROS2 Jazzy](https://img.shields.io/ros/v/jazzy/swri_console.svg?style=flat-square)](https://index.ros.org/r/swri_console/#jazzy) | [![ROS2 Kilted](https://img.shields.io/ros/v/kilted/swri_console.svg?style=flat-square)](https://index.ros.org/r/swri_console/#kilted) | [![ROS2 Lyrical](https://img.shields.io/ros/v/lyrical/swri_console.svg?style=flat-square)](https://index.ros.org/r/swri_console/#lyrical) | [![ROS2 Rolling](https://img.shields.io/ros/v/rolling/swri_console.svg?style=flat-square)](https://index.ros.org/r/swri_console/#rolling)

### CI

&nbsp; | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
GitHub Actions | [![CI](https://github.com/swri-robotics/swri_console/actions/workflows/humble.yml/badge.svg?branch=kilted-devel)](https://github.com/swri-robotics/swri_console/blob/kilted-devel/.github/workflows/humble.yml) | [![CI](https://github.com/swri-robotics/swri_console/actions/workflows/jazzy.yml/badge.svg?branch=kilted-devel)](https://github.com/swri-robotics/swri_console/blob/kilted-devel/.github/workflows/jazzy.yml) | [![CI](https://github.com/swri-robotics/swri_console/actions/workflows/kilted.yml/badge.svg?branch=kilted-devel)](https://github.com/swri-robotics/swri_console/blob/kilted-devel/.github/workflows/kilted.yml) | [![CI](https://github.com/swri-robotics/swri_console/actions/workflows/lyrical.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/swri_console/blob/ros2-devel/.github/workflows/lyrical.yml) | [![CI](https://github.com/swri-robotics/swri_console/actions/workflows/rolling.yml/badge.svg?branch=ros2-devel)](https://github.com/swri-robotics/swri_console/blob/ros2-devel/.github/workflows/rolling.yml)

### amd64 dev

&nbsp; | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
[`swri_console`](https://github.com/swri-robotics/swri_console) | [![dev](https://build.ros2.org/buildStatus/icon?job=Hdev__swri_console__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__swri_console__ubuntu_jammy_amd64/) | [![dev](https://build.ros2.org/buildStatus/icon?job=Jdev__swri_console__ubuntu_noble_amd64)](https://build.ros2.org/job/Jdev__swri_console__ubuntu_noble_amd64/) | [![dev](https://build.ros2.org/buildStatus/icon?job=Kdev__swri_console__ubuntu_noble_amd64)](https://build.ros2.org/job/Kdev__swri_console__ubuntu_noble_amd64/) | [![dev](https://build.ros2.org/buildStatus/icon?job=Ldev__swri_console__ubuntu_resolute_amd64)](https://build.ros2.org/job/Ldev__swri_console__ubuntu_resolute_amd64/) | [![dev](https://build.ros2.org/buildStatus/icon?job=Rdev__swri_console__ubuntu_resolute_amd64)](https://build.ros2.org/job/Rdev__swri_console__ubuntu_resolute_amd64/)

### amd64 bin

Package | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
[`swri_console`](https://index.ros.org/p/swri_console/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__swri_console__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__swri_console__ubuntu_jammy_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__swri_console__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__swri_console__ubuntu_noble_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__swri_console__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__swri_console__ubuntu_noble_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Lbin_uR64__swri_console__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Lbin_uR64__swri_console__ubuntu_resolute_amd64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Rbin_uR64__swri_console__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Rbin_uR64__swri_console__ubuntu_resolute_amd64__binary/)

### arm64 bin

Package | **Humble** | **Jazzy** | **Kilted** | **Lyrical** | **Rolling**
:--- | :---: | :---: | :---: | :---: | :---:
[`swri_console`](https://index.ros.org/p/swri_console/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__swri_console__ubuntu_jammy_arm64__binary)](https://build.ros2.org/job/Hbin_ujv8_uJv8__swri_console__ubuntu_jammy_arm64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__swri_console__ubuntu_noble_arm64__binary)](https://build.ros2.org/job/Jbin_unv8_uNv8__swri_console__ubuntu_noble_arm64__binary/) | [![bin](https://build.ros2.org/buildStatus/icon?job=Kbin_unv8_uNv8__swri_console__ubuntu_noble_arm64__binary)](https://build.ros2.org/job/Kbin_unv8_uNv8__swri_console__ubuntu_noble_arm64__binary/) | not built | not built

