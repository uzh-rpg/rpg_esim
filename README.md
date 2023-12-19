# Foreward

The work herein is primarily that of Rebecq et al, in the Acknowlegements section below. This fork simply impelements a dockerized version such that ESIM can be used on platforms without ROS Melodic. 

## Suggested Installation

After cloning this repository, simply build the dockerfile as follows.

```bash
# where $(pwd) is /the/path/to/rpg_esim
podman build -t esim:melodic -f melodic.Dockerfile
```

Alternatively, build with docker (untested)

```bash
podman build -t esim:melodic -f melodic.Dockerfile
```

## Run

Then, simply run the container, source the setup file, and launch the renderer. Please refer to specific instructions to run a given simulator in [the original rpg_esim wiki](https://github.com/uzh-rpg/rpg_esim/wiki).

```bash
# On the host
# Running as root via rootless podman mitigates security risks that come with root access to xhost.
# Use with caution if you are not running rootless podman!
podman run --rm -ti -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro esim
```

```bash
# In the container
. /home/user/sim_ws/devel/setup.bash
roscd esim_ros
# Based on the suggested test renderer
roslaunch esim_ros esim.launch config:=cfg/example.conf
```

An alternative, 3D, rendering option is below.

### OpenGL Renderer

```bash
roscd esim_ros
roslaunch esim_ros esim.launch config:=cfg/opengl.conf
```

## TODO
- [x] Add a graphical run script for visualization compatibility as discussed in numerous forum posts like [this one](https://unix.stackexchange.com/questions/330366/how-can-i-run-a-graphical-application-in-a-container-under-wayland)

## Acknowledgements

Thank you to the [Robotics and Perception Group](https://rpg.ifi.uzh.ch/) for all of their hard work and open source implementations. 
