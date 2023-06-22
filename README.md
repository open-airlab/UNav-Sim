# 🌊🤖 Introducing UNav-Sim: The Ultimate Underwater Robotics Simulator! 🚀

![UNavSim_logo](UNavSim_logo.png)


Are you fascinated by underwater robotics and want to explore the depths of the ocean from the comfort of your computer? Look no further! UNav-Sim is here to take you on an exciting underwater adventure.

**UNav-Sim** is an open-source cutting-edge underwater robotics simulator tool that leverages the power of **Unreal Engine 5 (UE5)** and [AirSim](https://github.com/microsoft/AirSim) to provide highly detailed rendering and simulation capabilities. Whether you're a researcher, developer, or robotics enthusiast, UNav-Sim offers a platform for you to dive deep into the world of underwater robotics.

## Key Features:
- **High-fidelity rendering:** Highest fidelty rendering engine using **Unreal Engine 5 (UE5)**.
- **Underwater physics and robot models:** Underwater physics and robot models are implemented.
- **ROS1 and ROS2 support:** Implement your control and navigation algorithms as ROS packages **ROS (Robot Operating System)**

Discover a wide range of possibilities with UNav-Sim, from exploring underwater terrains to designing and testing autonomous underwater vehicles (AUVs). Whether you're a student, researcher, or hobbyist, this powerful tool will accelerate your learning and experimentation process.

Join the underwater robotics revolution and unleash your creativity with UNav-Sim! 🌊🤖

To get started, visit our [GitHub repository]([https://github.com/your-username/UNav-Sim](https://github.com/open-airlab/UNav-Sim)) and dive into the world of underwater robotics.

\#UnderwaterRobotics #Simulation #UNavSim #UE5 #ROS #AutonomousUnderwaterVehicles #Robotics


# Installation
UNav-Sim relies in Unreal Engine 5 for generating realistic renderings. First, install UE5 and then proceed to install UNav-Sim

## UE5
- Make sure you are [registered with Epic Games](https://docs.unrealengine.com/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html). This is required to get source code access for Unreal Engine.

- Clone Unreal 5.1 in your favorite folder and build it (this may take a while!)
```bash
# go to the folder where you clone GitHub projects
git clone -b 5.1 git@github.com:EpicGames/UnrealEngine.git
cd UnrealEngine
./Setup.sh
./GenerateProjectFiles.sh
make
```

## UNav-Sim
Clone this repo in your favourite folder:
```bash
git clone git@github.com:open-airlab/UNav-Sim.git
cd UNav-Sim
```
We provide you scripts to automatically setup and build UNav-Sim:
```bash
./setup.sh
./build.sh
# use ./build.sh --debug to build in debug mode
```
**Troubleshooting:** If you're installing UNav-Sim from Ubuntu 22, then instead of `vulkan-utils` you will need to install `vulkan-tools`:
```bash
sudo apt update
sudo apt install vulkan-tools
```
## Setting up the UE5 environment
```bash
cd ~/UNav-Sim/Unreal/Environments/Blocks
right-click on Blocks.uproject, select Unreal Engine Generate Project Files
```


## Running the simulation
```bash
- open the project using your IDE 
- Run and debug using Launch BlocksEditor(DebugGame)
```


## **⭐ Please star this repository if you find it useful! ⭐**

Hey there! If you find this repository helpful, we kindly ask you to consider giving it a star. It's a great way for me to gauge its value and it also motivates me to continue maintaining and improving it. Your support is greatly appreciated!

Thank you for using this project! 🌟
