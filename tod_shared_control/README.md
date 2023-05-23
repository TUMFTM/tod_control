# tod_shared_control
This package provides nodes related to two shared control approaches for obstacle avoidance. These are Shared Velocity Control (SVC) and Shared Steering and Velocity Control (SSVC). The main capability is the correction of the operator's control commands in the presence of risk of hitting an obstacle. This package is meant to be run from the [container repository](https://github.com/TUMFTM/teleoperated_driving). The simulations described below are functional with `vehicleID=tum-q7` and `mode=onlySim`. 

* [Dependencies](#dependencies)
* [Documentation](#documentation)
* [Simulative Validation](#simulative-validation)
  + [SVC with Simulated Operator](#svc-with-simulated-operator)
  + [SSVC with Simulated Operator](#ssvc-with-simulated-operator)
  + [Operator in the Loop](#operator-in-the-loop)
* [Demonstrations of Operator in the Experiments](#demonstrations-of-operator-in-the-experiments)
* [Publication](#publication)


## Dependencies
  * ROS Packages: see `package.xml`
  * Third Party: 
    * [acados](https://github.com/acados/acados) with [blasfeo](https://github.com/giaf/blasfeo) and [hpipm](https://github.com/giaf/hpipm)
      ```
      cd
      git clone https://github.com/acados/acados.git
      cd acados
      git checkout d65fb0ee114b2192aeb8b85788c84581540066bc 
      git submodule update --recursive --init
      mkdir -p build
      cd build
      sudo cmake ..
      sudo make install -j32
      sudo bash -c "echo /home/$USER/acados/lib>> /etc/ld.so.conf.d/x86_64-linux-gnu.conf"
      sudo ldconfig
      cd ../external
      wget -q -nc --show-progress https://github.com/casadi/casadi/releases/download/3.4.0/casadi-linux-matlabR2014b-v3.4.0.tar.gz
      mkdir -p casadi-matlab
      tar -xf casadi-linux-matlabR2014b-v3.4.0.tar.gz -C casadi-matlab
      ```


## Documentation
There are multiple nodes in this package. These are categorized as follows. 

The SVC approach is contained and documented in `src/svc`. 

The SSVC approach based on Model Predictive Control (MPC) is contained and documented in `src/mpc`. 

There are two path tracking controller nodes through which the control commands from an operator can be simulated (see below under Simulative Validation for the launch settings.) The first controller, based on Feedback Linearization (FBL), is contained and documented in `src/fbl`. The second controller, based on MPC, is contained and documented in `src/ptc`.

To transmit relevant data from the vehicle to the operator side and vice-versa, there are two network nodes. These are contained and documented in `src/network`. 

Also, for the simulative validation, there are two helper nodes for publishing a dummy object list and a dummy path for the path tracking controllers to follow. These are contained and documented under `src/dummy`.


## Simulative Validation
With this state of the package, several simulations can be run in order to validate the shared control approaches. As described above, it is recommended to launch the complete software stack (with `both.launch`) from the container repository with the `vehicleID=tum-q7` and the `mode=onlySim`. After the launch, make sure to hit connect, select shared control and start the teleoperation in the manager GUI. 

There are also the options to visualize and monitor shared control approaches in `rqt_multiplot` and `rviz` by setting the flags in `launch/tod_shared_control.launch` accordingly.

### SVC with Simulated Operator
A video, showcasing a validation of SVC with a simulated operator (based on FBL) is available on [YouTube](https://youtu.be/yFzSiwtUtq4).
This run can be resimulated by using the following launch settings in `launch/tod_shared_control.launch`.
  * `mode=svc`
  * `shadow_mode=fbl`
  * `useDummyObjects=true`
  * `useShadowAsOperatorSim=true` 


### SSVC with Simulated Operator
A video, showcasing a validation of SSVC with a simulated operator (based on FBL) is available on [YouTube](https://youtu.be/vz8slCFW140).
This run can be resimulated by using the following launch settings in `launch/tod_shared_control.launch`.
  * `mode=mpc`
  * `shadow_mode=fbl`
  * `useDummyObjects=true`
  * `useShadowAsOperatorSim=true`

### Operator in the Loop
To manually control the vehicle in above scenario for a shared control mode of choice, use the following launch settings in `launch/tod_shared_control.launch`.
  * `mode=svc` or `mode=mpc`
  * `shadow_mode=none`
  * `useDummyObjects=true`
  * `useShadowAsOperatorSim=false`

For minimal requirements, select the `virtual` input device in the `tod_manager` GUI. 

Instead of the objects positioned alternatingly left and right, the vehicle can also be operated in a random obstacle field as well. For this, see the documentation of the dummy object list publisher node under `src/dummy`. 


## Demonstrations of Operator in the Experiments
Videos, showcasing experimental validations of the shared control approaches on a 1:10-scale vehicle testbed also available on YouTube for
  * [SVC](https://youtu.be/yXRfVOLSFuM), 
  * [SSVC](https://youtu.be/aiOSsajcSfM) 
  * and (for completeness) [Direct Control](https://youtu.be/oY-a-6BItjg). 


## Publication

The paper, describing the Steering Action-aware Adaptive Cruise Control (SVC) in detail, is available on [IEEE](https://ieeexplore.ieee.org/document/9945081). Therein, a comparison with the Model Predictive Control-based (MPC / SSVC) approach is also described.  

```
@INPROCEEDINGS{9945081,
  author={Schimpe, Andreas and Majstorovic, Domagoj and Diermeyer, Frank},
  booktitle={2022 IEEE International Conference on Systems, Man, and Cybernetics (SMC)}, 
  title={Steering Action-aware Adaptive Cruise Control for Teleoperated Driving}, 
  year={2022},
  pages={988-993},
  doi={10.1109/SMC53654.2022.9945081}}
```
