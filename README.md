
<img align="left" width="180" height="180" src="https://github.com/AlbertoZandara/Saturn-system/blob/main/Miscellaneus/Saturn13.png" alt="Resume application project app icon">

# Saturn system

###### The Saturn system is a technological solution designed for agricultural parameters monitoring. The system is conceived to be integrated in a smart agriculture system in order to achieve a reliable and simple crops data collection. It consists in a low power sensor network based on the 802.15.4 communication protocol. The system is composed by two types of device: the Dominus and the Agricola.

### Overview

In this reposutory is contained all the software necessary to configure a working systems. The hardware design is still under development, the working prototype of the network has been implemented only with breadboards and so only the Agricola schematic is actually available. 

![Schematic](https://github.com/AlbertoZandara/Saturn-system/blob/main/Schematics/SCHEMATIC.png)

### C++ program

In the C++ program is contained the low-level software designed to empower the low-power sensors. The code contains: a sensing pipline with all the required function to collect data from the environment, some power-save features and a little communication protocol.

### Node-RED program

The Node-RED flow is conceived to run on the system gateway that forwards sensor data to the internet. The flow contains: network management and security features, a simple communication protocol and the actual UI available for the user.

![UI](https://github.com/AlbertoZandara/Saturn-system/blob/main/Miscellaneus/Example.png)
