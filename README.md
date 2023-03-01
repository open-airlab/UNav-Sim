# Welcome to UNav-Sim!

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
