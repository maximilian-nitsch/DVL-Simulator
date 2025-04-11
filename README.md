# C++/ROS 2 Doppler-Velocity Log Simulator
![Build](https://github.com/maximilian-nitsch/DVL-Simulator/actions/workflows/ci.yaml/badge.svg)<!-- -->
[![License](https://img.shields.io/badge/license-BSD--3-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)<!-- -->
[![Last Commit](https://img.shields.io/github/last-commit/maximilian-nitsch/DVL-Simulator)](https://github.com/maximilian-nitsch/DVL-Simulator/commits/main)<!-- -->
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://index.ros.org/doc/ros2/Installation/Humble/)<!-- -->
[![Release](https://img.shields.io/github/v/release/maximilian-nitsch/DVL-Simulator)](https://github.com/maximilian-nitsch/DVL-Simulator/releases)<!-- -->
[![Open Issues](https://img.shields.io/github/issues/maximilian-nitsch/DVL-Simulator)](https://github.com/maximilian-nitsch/DVL-Simulator/issues)<!-- -->
[![Contributors](https://img.shields.io/github/contributors/maximilian-nitsch/DVL-Simulator)](https://github.com/maximilian-nitsch/DVL-Simulator/graphs/contributors)

<img src="./data/icon.svg" alt="Icon" width="20%">

<!--- protected region package header begins -->
**Author:**
- Maximilian Nitsch <m.nitsch@irt.rwth-aachen.de>

**Affiliation:** Institute of Automatic Control - RWTH Aachen University

**Maintainer:**
  - Maximilian Nitsch <m.nitsch@irt.rwth-aachen.de>
<!--- protected region package header ends -->

## Description
This project provides a simple DVL simulator written in C++.

The simulator implements the following features:
- DVL measurement simulation (sensor frame)
- DVL beam measurement simulation (Janus array)
- Ideal measurement model
- Beam velocity white noise model
- Scale factor model
- Gyroscope noise as additional error between IMU and DVL due to lever arm
- Quantization errors
- Altitude limit (min./max. altitude above ground/under ice)
- Velocity limit (max. allowed 2-norm of velocity vector / = speed)
- All parameters for the DVL can be configured in a YAML file
- All models and effects can be enabled/disabled separately

The acoustic propagation is currently not simulated. 
Instead, the sensor frame velocity is transformed to beam velocities.
The beam velocites are then corrupted with errors and transformed back (least square solution) to DVL velocity.

An example config file from real data of a WaterLinked DVL A50 is provided.

## Table of Contents

- [Dependencies](#dependencies)
- [ROS 2 Node](#ros-2-node)
  - [Publishers](#publishers)
  - [Subscribers](#subscribers)
- [Installation](#installation)
- [Usage](#usage)
- [Coding Guidelines](#coding-guidelines)
- [References](#references)
- [Reports](#reports)
- [Contributing](#contributing)
- [License](#license)

# Dependencies

This project depends on the following literature and libraries:

- **Eigen3**: Eigen is a C++ template library for linear algebra: [Eigen website](https://eigen.tuxfamily.org/).
- **ROS 2 Humble**: ROS 2 is a set of software libraries and tools for building robot applications: [ROS 2 Installation page](https://docs.ros.org/en/humble/Installation.html).
- **Navigation-Interfaces**: Custom ROS 2 interfaces repository: [Navigation-Interfaces](https://github.com/maximilian-nitsch/Navigation-Interfaces).

## ROS 2 Node Description

The DVL simulator node implements eleven publishers and subscribes to one topic.
Some publishers provide placeholder values to replicate DVL driver data or hold values for future extensions.
ROS 2 services or actions are not provided.

### Publishers

This node publishes the following topics:

| Topic Name       | Message Type        | Description                        | Link     |
|------------------|---------------------|------------------------------------|----------|
| `velocity_and_transducer`   | `nanoauv_sensor_driver_interfaces/DvlVelocity.msg`   | Custom DVL velocity and beam data.| [DvlVelocity.msg](https://github.com/maximilian-nitsch/Navigation-Interfaces/blob/main/nanoauv_sensor_driver_interfaces/msg/DvlVelocity.msg) |

### Subscribers

This node subscribes to the following topics:

| Topic Name        | Message Type        | Description                        | Link     |
|-------------------|---------------------|------------------------------------|----------|
| `odometry`| `nav_msgs/Odometry.msg`| Estimate of a position and velocity in free space.| [Odometry.msg](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) |



# Installation

To install the `dvl_simulator_package`, you need to follow these steps:

1. **Install Eigen3**: Eigen3 is a dependency for your package. You can install it using your package manager. For example, on Ubuntu, you can install it using the following command:

    ```bash
    sudo apt-get install libeigen3-dev
    ```

2. **Install ROS 2 Humble**: Ensure you have ROS 2 (Humble) installed. You can follow the official installation instructions provided by ROS 2. Visit [ROS 2 Humble Installation page](https://docs.ros.org/en/humble/Installation.html) for detailed installation instructions tailored to your platform.

3. **Clone the Package**: Clone the package repository to your ROS 2 workspace. If you don't have a ROS 2 workspace yet, you can create one using the following commands:

    ```bash
    mkdir -p /path/to/ros2_workspace/src
    cd /path/to/ros2_workspace/src
    ```

    Now, clone the package repository:

    ```bash
    git clone <repository_url>
    ```

    Replace `<repository_url>` with the URL of your package repository.

4. **Build the Package**: Once the package is cloned, you must build it using colcon, the default build system for ROS 2. Navigate to your ROS 2 workspace and run the following command:

    ```bash
    cd /path/to/ros2_workspace
    colcon build
    ```

    This command will build all the packages in your workspace, including the newly added package.

5. **Source the Workspace**: After building the package, you need to source your ROS 2 workspace to make the package available in your ROS 2 environment. Run the following command:

    ```bash
    source /path/to/ros2_workspace/install/setup.bash
    ```

    Replace `/path/to/ros2_workspace` with the actual path to your ROS 2 workspace.

That's it! Your `dvl_simulator_package` should now be installed along with its dependencies and ready to use in your ROS 2 environment.

## Usage

1. **Configure your YAML file** for your DVL or use the default file.

2. **Start the DVL simulator** with the launch file:
    ```bash
    ros2 launch dvl_simulator_package dvl_simulator.launch.py
    ```
  The DVL simulator prints your settings and now waits for a ground truth odometry message.

3. **Provide an odometry publisher** from you vehicle simulation.

4. **Check ROS 2 topics** The DVL values should now be published.


**Important Usage Information**:
- The odometry message must be published with at least the DVL data rate/sample time.
- The message `/velocity_and_transducer/diagnostic_array` will show `WARN` if the odometry rate is lower.
- If no odometry message is published, the message `/velocity_and_transducer/diagnostic_array` will show `STALE`.
- If everything is correct `/velocity_and_transducer/diagnostic_array` will show `OK`. 

## Coding Guidelines

This project follows these coding guidelines:
- https://google.github.io/styleguide/cppguide.html
- http://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html 

## References

The DVL simulator implementation follows the datasheets and publications:
- Water Linked AS. (2022). *The World’s Smallest DVL - DVL A50* (WL-21035-3–DVL-A50-Datasheet–v2). Water Linked AS, June 21, 2022.
- Nortek Group. (2023). *Technical Specification DVL1000 - 300 m*. Nortek Group.
- Rowe Technologies, Inc. (2023). *SeaPILOT DVL/OEM Specifications*. Rowe Technologies, Inc.
- Teledyne RD Instruments. (2020). *Teledyne Marine Product Brochure Navigation Wayfinder DVL*.
- Whitcomb, L.; Yoerger, D.; Singh, H. (1999). Advances in Doppler-based Navigation of Underwater Robotic Vehicles. In: *Proceedings 1999 IEEE International Conference on Robotics and Automation (Cat. No.99CH36288C)*, Vol. 1, May 1999, pp. 399–406. doi: [10.1109/ROBOT.1999.770011](https://doi.org/10.1109/ROBOT.1999.770011).
- Randeni, S.; Schneider, T.; Bhatt, E. C.; Víquez, O. A.; Schmidt, H. (2023). A High-Resolution AUV Navigation Framework with Integrated Communication and Tracking for under-Ice Deployments. In: *Journal of Field Robotics*, Vol. 40, no. 2, pp. 346–367. doi: [10.1002/rob.22133](https://doi.org/10.1002/rob.22133).
- Gordon, R. L. (1996). *Acoustic Doppler Current Profiler Principles of Operation: A Practical Primer* (P/N 951-6069-00). Teledyne RD Instruments, Inc, Jan. 8, 1996, p. 57.
- Oertel, D. (2018). *Deep-Sea Model-Aided Navigation Accuracy for Autonomous Underwater Vehicles Using Online Calibrated Dynamic Models*. PhD thesis, Karlsruhe: Karlsruher Institut für Technologie (KIT).
- Tal, A.; Klein, I.; Katz, R. (2017). Inertial Navigation System/Doppler Velocity Log (INS/DVL) Fusion with Partial DVL Measurements. In: *Sensors*, Vol. 17, no. 2, 2 Feb. 2017, p. 415. doi: [10.3390/s17020415](https://doi.org/10.3390/s17020415).
- Wang, Y.; Chirikjian, G. (2006). Error Propagation on the Euclidean Group with Applications to Manipulator Kinematics. In: *IEEE Transactions on Robotics*, Vol. 22, no. 4, Aug. 2006, pp. 591–602. doi: [10.1109/TRO.2006.878978](https://doi.org/10.1109/TRO.2006.878978).
- Hegrenæs, Ø.; Ramstad, A.; Pedersen, T.; Velasco, D. (2016). Validation of a New Generation DVL for Underwater Vehicle Navigation. In: *2016 IEEE/OES Autonomous Underwater Vehicles (AUV)*, Nov. 2016, pp. 342–348. doi: [10.1109/AUV.2016.7778694](https://doi.org/10.1109/AUV.2016.7778694).
- Hegrenaes, O.; Berglund, E. (2009). Doppler Water-Track Aided Inertial Navigation for Autonomous Underwater Vehicle. In: *OCEANS 2009-EUROPE*, May 2009, pp. 1–10. doi: [10.1109/OCEANSE.2009.5278307](https://doi.org/10.1109/OCEANSE.2009.5278307).
- Taudien, J.; Bilén, S. (2016). Quantifying Error Sources That Affect Long-Term Accuracy of Doppler Velocity Logs. Sept. 19, 2016. doi: [10.1109/OCEANS.2016.7761084](https://doi.org/10.1109/OCEANS.2016.7761084).
- Taudien, J. Y.; Bilén, S. G. (2018). Quantifying Long-Term Accuracy of Sonar Doppler Velocity Logs. In: *IEEE Journal of Oceanic Engineering*, Vol. 43, no. 3, July 2018, pp. 764–776. doi: [10.1109/JOE.2017.2735558](https://doi.org/10.1109/JOE.2017.2735558).
- Meachum, B.; Hui, N. (2021). Verifying Performance, Integration of DVL for Small ROVs, AUVs. In: *SeaTechnology*, Feb. 2021. Ed. by Compass Publications Inc.

## Contributing

If you want to contribute to the project, see the [CONTRIBUTING](CONTRIBUTING) file for details.

## License

This project is licensed under the BSD-3-Clause License. See the [LICENSE](LICENSE) file for details.

