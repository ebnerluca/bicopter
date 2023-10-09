# BiCopter
In this project a custom bicopter platform is designed based on a generic RaspberryPi setup.

The aim of the project is mainly fun, but also to have a low cost research platform to test various vision based methods such as [UWDepth](https://github.com/ebnerluca/uw_depth).

<img src="https://github.com/ebnerluca/bicopter/assets/48278846/9bfedd10-16d8-43e5-8040-08637a36a194" width="40%">


## Modules

- `bicopter_sim`: Metapackage for building and launching the simulation
- `bicopter_pybullet_controller`: Lowlevel controller based on pybullet simulator.
- `bicopter_angle_controller`: Angle controller for bicopter based on linearization of bicopter dynamics around equilibrium.
- `bicopter_model`: Model and property files (.urdf, .stl, .yaml).
- `bicopter_analytics`: This package can be run on the laptop and should show insights on the operatio of the bicopter. (e.g. visualization of the IMU readings)
- others are deprecated and soon to be removed.

## Install
### Raspi Hardware Setup
On an install of Ubuntu 20.04 on a Raspberry Pi 4 device, do as follows:
```
# install gpiozero and pigpio python packages
sudo apt install python3-gpiozero python3-pigpio

# install pigpio daemon
sudo apt install pigpio  # or from source as simply described here: https://abyz.me.uk/rpi/pigpio/download.html

# launch pigpio daemon
sudo pigpio  # once per session, you should create a systemctl service to automate that
```

### Laptop Setup
```
# install pybullet simulator
pip3 install pybullet

# install tf_transformations
pip3 install transforms3d
sudo apt install ros-foxy-tf-transformations
```




