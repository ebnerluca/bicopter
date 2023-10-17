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
On an install of Ubuntu 22.04 on a Raspberry Pi 4 device, install ROS2 Humble with the official instructions. Additionally do as follows:
```
# install gpiozero and pigpio python packages
sudo apt install python3-gpiozero python3-pigpio

# install pigpio daemon
sudo apt install pigpio  # or from source as simply described here: https://abyz.me.uk/rpi/pigpio/download.html

# launch pigpio daemon
sudo pigpio  # once per session, you should create a systemctl service to automate that
```

Since pgpiod.service does not exist if pigpiod is installed from source, add this custom service to `/lib/systemd/system/pigpiod.service`:
```
[Unit]
Description=Daemon required to control GPIO pins via pigpio
[Service]
ExecStart=/usr/local/bin/pigpiod
ExecStop=/bin/systemctl kill -s SIGKILL pigpiod
Type=forking
[Install]
WantedBy=multi-user.target
```
and enable the service with `sudo systemctl enable pigpiod. Reboot and check if the service is working: 
```
sudo systemctl status pigpiod  # check if service is running
pigs t  # check daemon id
```

### Laptop Setup
On your machine with Ubuntu 22.04, setup a virtual environment (dont forget access to system site pkgs!) and install a few dependencies:
```
# venv
python3 -m venv --system-site-packages venv
source venv/bin/activate

# install pybullet simulator
pip3 install pybullet

# install tf_transformations
pip3 install transforms3d
sudo apt install ros-humble-tf-transformations

# install
colcon build --packages-up-to bicopter_sim
source install/setup.bash
```

### Run sim
```
# steer with joystick
ros2 launch bicopter_sim run_bicopter_sim.launch.py
```




