# Quadcopter Drone Swarm with PX4 Autopilot, RPI 3 Mesh Network, and ROS Python Integration

This repository contains the code for an autonomous drone swarm project aimed at providing counter-UAS airspace services by creating a network of custom-built quadcopter drones capable of flying in unison, communicating with each other, and avoiding collisions. The project utilizes the PX4 autopilot system, communicates with a Raspberry Pi running ROS (Robot Operating System) through Python, and leverages the OLSR (Optimized Link State Routing) protocol to create a mesh network between multiple systems.

## Project Overview

The primary objective of this project is to develop a robust and autonomous sUAS (small unmanned aerial system) capable of mitigating threats posed by other unauthorized drones entering the airspace. The drone swarm consists of multiple quadcopter drones that communicate with each other and work collaboratively to ensure safe and synchronized flight operations. The key features and components of this project are as follows:

1. **Quadcopter Drones**: The project involves the construction and deployment of custom-built quadcopter drones. These drones are equipped with the necessary hardware, including flight controllers, motors, propellers, and sensors, to ensure stable flight and reliable data collection.

2. **PX4 Autopilot System**: The drones are powered by the PX4 autopilot system, a flexible and open-source flight control software. PX4 provides the necessary algorithms and flight control logic to stabilize the drones in various flight modes and execute autonomous missions.

3. **Raspberry Pi and ROS**: Each drone is connected to a Raspberry Pi, a small single-board computer, which serves as the onboard companion computer. The Raspberry Pi runs ROS, a flexible framework for writing robot software, enabling seamless communication between the drones and facilitating high-level control and coordination.

4. **Mesh Network using OLSR**: To establish a communication network between the drones, the project utilizes the OLSR protocol. OLSR allows the formation of a self-configuring mesh network, where each drone acts as a node. This mesh network enables real-time data exchange, such as sharing telemetry, position information, and mission updates.

5. **Collision Avoidance**: The project incorporates custom collision avoidance code to ensure safe flight operations within the drone swarm. By leveraging sensor data, including GPS, lidar, and cameras, the drones can detect potential collisions, make intelligent decisions, and adjust their flight paths to avoid any conflicts.
