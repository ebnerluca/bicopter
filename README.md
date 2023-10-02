# BiCopter
In this project a custom bicopter platform is designed based on a generic RaspberryPi setup.

The aim of the project is mainly fun, but also to have a low cost research platform to test various vision based methods such as [UWDepth](https://github.com/ebnerluca/uw_depth).

<img src="https://github.com/ebnerluca/bicopter/assets/48278846/9bfedd10-16d8-43e5-8040-08637a36a194" width="40%">


## Modules

- `bicopter_sim`: Metapackage for building and launching the simulation
- `bicopter_pybullet_controller`: Lowlevel controller based on pybullet simulator.
- `bicopter_angle_controller`: Angle controller for bicopter based on linearization of bicopter dynamics around equilibrium.
- `bicopter_model`: Model and property files (.urdf, .stl, .yaml).
- others are deprecated and soon to be removed.




