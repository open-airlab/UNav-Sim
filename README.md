# 🌊🤖 Introducing UNav-Sim: The Ultimate Underwater Robotics Simulator! 🚀

![UNavSim_logo](UNavSim_logo.png)



**UNav-Sim** is an open-source cutting-edge underwater robotics simulator tool that leverages the power of **Unreal Engine 5 (UE5)** and [AirSim](https://github.com/microsoft/AirSim) to provide highly detailed rendering and simulation capabilities. Whether you're a researcher, developer, or robotics enthusiast, UNav-Sim offers a platform for you to dive deep into the world of underwater robotics.

## Key Features:
- **Open-source** :heavy_check_mark:
- **High-fidelity rendering using Unreal Engine 5** :heavy_check_mark:
- **Underwater physics and robot models** :heavy_check_mark:
- **Linux and Windows compatible** :heavy_check_mark:
- **ROS1 and ROS2 support** :heavy_check_mark:


 ## Example environments: [MIMIR](https://github.com/remaro-network/MIMIR-UW) 

|    |   |
:-------------------------:|:-------------------------:
![](https://github.com/remaro-network/MIMIR-UW/blob/main/media/MIMIR-oceanfloor.gif?raw=true)  |  ![](https://github.com/remaro-network/MIMIR-UW/blob/main/media/MIMIR-seafloor.gif?raw=true) |  
![](https://github.com/remaro-network/MIMIR-UW/blob/main/media/MIMIR-seaflooralgae.gif?raw=true)  |  ![](https://github.com/remaro-network/MIMIR-UW/blob/main/media/MIMIR-sandpipe.gif?raw=true) |  

# Index 
 [1. Installation](https://github.com/open-airlab/UNav-Sim#1-installation)
 
 [2. Running the simulation](https://github.com/open-airlab/UNav-Sim#2-running-the-simulation)
 
 [3. UNav-Sim ros navigation stack](https://github.com/open-airlab/UNav-Sim#3-unav-sim-ros-navigation-stack)

 [4. UNav-Sim documentation](https://github.com/open-airlab/UNav-Sim#4-unav-sim-documentation)

# 1. Installation Instructions (for Ubuntu OS)
UNav-Sim relies in Unreal Engine 5 for generating realistic renderings. First, install UE5 and then proceed to install UNav-Sim

## 1.1. UE5
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

## 1.2. UNav-Sim
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
## 1.3. Setting up the UE5 environment
```bash
cd ~/UNav-Sim/Unreal/Environments/Blocks
right-click on Blocks.uproject, select Unreal Engine Generate Project Files
```


# 2. Running the simulation
- Add the settings.json file to C:\Users\[username]\Documents\AirSim

```bash
- open the project using your IDE 
- Run and debug using Launch BlocksEditor(DebugGame)
```


# 3. UNav-Sim ros navigation stack 
The UNav-Sim ROS navigation stack, as detailed in the paper, encompasses various components essential for underwater vehicle navigation. It integrates a Nonlinear Model Predictive Controller (NMPC) tailored for the BlueROV2 Heavy model and a pipe-following path planner. Additionally, the stack incorporates several Visual Simultaneous Localization and Mapping (VSLAM) algorithms.

## Components:

1. **NMPC for BlueROV2 Heavy Model**  
   Located in the ROS 1 repository, this component provides a Nonlinear Model Predictive Controller (NMPC) specifically designed for the BlueROV2 Heavy model. The controller facilitates precise and dynamic control of the underwater vehicle.  
   Repository Path: `\UNav-Sim\ros\src\UNav-Sim_NMPC`

2. **Pipe Following Path Planner**  
   The stack includes a path planner neural network trained for pipe-following task, essential for various underwater inspection and maintenance missions. 

3. **Visual Simultaneous Localization and Mapping (VSLAM) Algorithms**  
   The UNav-Sim ROS navigation stack incorporates several Visual Simultaneous Localization and Mapping (VSLAM) algorithms. These algorithms enable the vehicle to localize itself in its environment while simultaneously mapping the surroundings using visual sensor data.  
   Repository Path for ROS 2: `\UNav-Sim\ros2\src\robot_visual_localization`



# 4. UNav-Sim documentation
[Link to paper](https://ieeexplore.ieee.org/document/10406819) 

## Citation
If you use this framework in your work, please cite the following paper:[Link to paper](https://ieeexplore.ieee.org/document/10406819) 

```bash
@inproceedings{amer2023unav,
  title={UNav-Sim: A Visually Realistic Underwater Robotics Simulator and Synthetic Data-generation Framework},
  author={Amer, Abdelhakim and {\'A}lvarez-Tu{\~n}{\'o}n, Olaya and U{\u{g}}urlu, Halil {\.I}brahim and Sejersen, Jonas Le Fevre and Brodskiy, Yury and Kayacan, Erdal},
  booktitle={2023 21st International Conference on Advanced Robotics (ICAR)},
  pages={570--576},
  year={2023},
  organization={IEEE}
}
```

```bash
@inproceedings{alvarez2023mimir,
  title={Mimir-uw: A multipurpose synthetic dataset for underwater navigation and inspection},
  author={{\'A}lvarez-Tu{\~n}{\'o}n, Olaya and Kanner, Hemanth and Marnet, Luiza Ribeiro and Pham, Huy Xuan and le Fevre Sejersen, Jonas and Brodskiy, Yury and Kayacan, Erdal},
  booktitle={2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={6141--6148},
  year={2023},
  organization={IEEE}
}
```


### **Questions and inquiries**
For  questions and inquiries please create a GitHub issue and we will try to respond as soon a possible!





### **⭐ Please star this repository if you find it useful! ⭐**
Thank you for using this project! 🌟

[![Star History Chart](https://api.star-history.com/svg?repos=open-airlab/UNav-Sim&type=Date)](https://star-history.com/#open-airlab/UNav-Sim&Date)





