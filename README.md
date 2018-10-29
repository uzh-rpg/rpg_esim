# ESIM: an Open Event Camera Simulator

This is the code for the 2018 CoRL paper **ESIM: an Open Event Camera Simulator** by [Henri Rebecq](http://henri.rebecq.fr), Daniel Gehrig and [Davide Scaramuzza](http://rpg.ifi.uzh.ch/people_scaramuzza.html):
```bibtex
@Article{Rebecq18corl,
  author        = {Henri Rebecq and Daniel Gehrig and Davide Scaramuzza},
  title         = {{ESIM}: an Open Event Camera Simulator},
  journal       = {Conf. on Robotics Learning (CoRL)},
  year          = 2018,
  month         = oct
}
```
You can find a pdf of the paper [here](http://rpg.ifi.uzh.ch/docs/CORL18_Rebecq.pdf). If you use any of this code, please cite this publication.

## Features

- Accurate event simulation, guaranteed by the tight integration between the rendering engine and the event simulator
- IMU simulation
- Support for multi-camera systems
- Ground truth camera poses, IMU biases, angular/linear velocities, depth maps, and optic flow maps
- Support for camera distortion
- Different C+/C- contrast thresholds
- Basic noise simulation for event cameras (based on additive Gaussian noise on the contrast threshold)
- Motion blur simulation
- Publish to ROS and/or save data to rosbag

## Install

Installation instructions can be found in [our wiki](https://github.com/uzh-rpg/rpg_event_camera_simulator/wiki/Installation).

## Run

Specific instructions to run the simulator depending on the chosen rendering engine can be found in [our wiki](https://github.com/uzh-rpg/rpg_event_camera_simulator/wiki).

## Acknowledgements

We thank Raffael Theiler and Dario Brescianini for their contributions to ESIM.
This research was supported by by Swiss National Center of Competence Research Robotics (NCCR), Qualcomm (through the Qualcomm Innovation Fellowship Award 2018), the SNSF-ERC Starting Grant and DARPA FLA.
