my_cpp_py_pkg (turn into HIRO-HIL-SERL-Utils)
================================
This repository contains a set of libraries and utils for the HIRO HIL-SERL implementation that you can find at this link: [hil-serl-hiro](https://github.com/claudio-dg/hil-serl-hiro/tree/hiro_simulation).
The main goal of these libraries is that of permitting a proper data exchange between Gym's environment and its robot counterpart, both for simulation case (i.e. with Mujoco) and both for the real UR case.
Moreover, here you can find the scripts to control the robot by using an XBOX joypad


Table of contents
----------------------

* [Repository structure](#repository-structure)
* [Joypad Commands](#joypad-commands)
<!-- * [Limitations and Possible Improvements](#limitations-and-possible-improvements) -->


## Repository structure
The main part of this repository is contained in the ```scripts``` folder.  
The following table briefly introduces the content of each folder.


 **Table for code structure**

| Code Directory | Description |
| --- | --- |
| [config.state_bridge_params.yaml](https://github.com/claudio-dg/my_cpp_py_pkg/blob/master/config/state_bridge_params.yaml) | Parameter file to select what data to "bridge". (currently for Simulation only) |
| [launch.**state_bridge.launch.py**](https://github.com/claudio-dg/my_cpp_py_pkg/blob/master/launch/state_bridge.launch.py) | Launch File to start "bridging" information between ```Mujoco``` and ```Gym``` |
| [msg](https://github.com/claudio-dg/my_cpp_py_pkg/tree/master/msg) | Contains custom ROS messages to deal with [simulation](https://github.com/claudio-dg/my_cpp_py_pkg/blob/master/msg/SimulationState.msg) and [real](https://github.com/claudio-dg/my_cpp_py_pkg/blob/master/msg/RealState.msg) robot states |
| [scripts](https://github.com/claudio-dg/my_cpp_py_pkg/tree/master/scripts) | Contains the key parts of the repo explained here below |
| [scripts.**ForceBridgeNode.py**](https://github.com/claudio-dg/my_cpp_py_pkg/blob/master/scripts/ForceBridgeNode.py) | Script to bridge the simulation force from ```Mujoco``` to ```Gym``` |
| [scripts.**StateBridgeNode.py**](https://github.com/claudio-dg/my_cpp_py_pkg/blob/master/scripts/StateBridgeNode.py)|  Script to bridge data between ```Mujoco``` and ```Gym's``` environments|
| [scripts.**RealStateBridgeNode.py**](https://github.com/claudio-dg/my_cpp_py_pkg/blob/master/scripts/RealStateBridgeNode.py) | Script to bridge data between the real robot and ```Gym's``` environment |
| [scripts.**UR_joystick_move.py**](https://github.com/claudio-dg/my_cpp_py_pkg/blob/master/scripts/UR_joystick_move.py) | Code to collect input from joystick and send them to the Bridge or directly to the Robot |


## Joypad Commands
The following pictures show how to use the XBOX pad to move and/or rotate the UR end effector. In the scripts **(metterli in una sottocartella e linkarla, ma conviene farlo da qua o da vecow e ushare?)** you can adjust the following parameters to adjust the control to your need: increase the scale value to have faster movements, or decrease it to have slower and more accurate control of the robot
```bash
    CONTROLLER_CONFIGS = {
        ControllerType.XBOX: ControllerConfig(
            # XBOX controller joystick values have 16 bit resolution [0, 65535]
            resolution={
                'ABS_X': 2**16,
                'ABS_Y': 2**16,
                'ABS_RX': 2**16,
                'ABS_RY': 2**16,
                'ABS_Z': 2**8,
                'ABS_RZ': 2**8,
                'ABS_HAT0X': 1.0,
            },
            scale={
                'ABS_X': 0.1*screw_case_scale_xyz, 
                'ABS_Y': 0.1*screw_case_scale_xyz, 
                'ABS_RX': 0.3, ## ignored
                'ABS_RY': 0.3, ## ignored
                

                'ABS_Z': 0.015*5*screw_case_scale_Z_RZ, 
                'ABS_RZ': 0.015*5*screw_case_scale_Z_RZ,
                'ABS_HAT0X': 0.3,
            }
        ),
    }
```

UR_joystick_move.py: only translational movement
====================================================================
<p align="center">
  <img src="https://github.com/claudio-dg/hil-serl-hiro/blob/hiro_simulation/docs/images/joystick.png" width="550">
</p>

UR_joystick_wRotation_move.py: include end effector rotational movement
====================================================================
<p align="center">
  <img src="https://github.com/claudio-dg/hil-serl-hiro/blob/hiro_simulation/docs/images/joystick_2.png" width="550">
</p>
