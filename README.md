yarp-omega3
======================

![CI badge](https://github.com/robotology-playground/yarp-omega3/workflows/C++%20CI%20Workflow/badge.svg)

Simple YARP-enabled server for the [Force Dimension Omega.3](https://www.forcedimension.com/products/omega) device.

Supports Linux only at the moment.

### Dependencies

- [libpthread](https://www.gnu.org/software/hurd/libpthread.html)
- [libusb](https://libusb.info/)
- [YCM](https://github.com/robotology/ycm)

Notes:
- the SDK of the robot is automatically downloaded during the build process
- `libpthread` and `libusb`, both required by the robot SDK, can be easily installed, e.g. in Ubuntu use
   ```
   apt install libusb-1.0-0-dev libpthread-stubs0-dev
   ```

### How to build

```
git clone https://github.com/robotology-playground/yarp-omega3
cd yarp-omega3
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<where_to_install> ../
make install
```

In order to use the device in user space, you need to import the provided [udev rule](config/99-omega3-libusb.rules), e.g. in `/etc/udev/rules.d`.

### How to use

For the first use it might be neccesary to reboot system to connect to `yarp-omega3-server` when connected to a usb hub.

Run `yarpserver --write`, `yarp-omega3-server` and send RPC commands to `/yarp-omega3-server/rpc:i`.

Available commands are
- help
- stop (completely disengange robot control)
- quit (close the module)
- set_position x y z (send a static position)
- track_position x y z (send a position, to be used in _streaming_ mode)
- set_force f_x f_y f_z (send a force reference)

The module switches from position to force control depending on the input from the user.

The state of the robot is available in forms of `yarp::sig::Vector`s sent over the ports
- `/yarp-omega3-server/position:i` (3 Cartesian coordinates of the robot)
- `/yarp-omega3-server/force:i` (3 coordinates of the force at the end effector)

[Sample modules](src/samples/python) written in Python are available.

### Maintainers

This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/xenvre.png" width="40">](https://github.com/xenvre) | [@xenvre](https://github.com/xenvre) |
