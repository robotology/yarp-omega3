yarp-omega3
======================

![CI badge](https://github.com/robotology-playground/yarp-omega3/workflows/C++%20CI%20Workflow/badge.svg)

Simple YARP-enabled server for the [Force Dimension Omega.3](https://www.forcedimension.com/products/omega) device.

Supports Linux only at the moment.

### Dependencies

- [libpthread](https://www.gnu.org/software/hurd/libpthread.html)
- [libusb](https://libusb.info/)
- [YCM](https://github.com/robotology/ycm)

#### Notes:
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

Run `yarp-omega3-server` and send RPC commands to `/yarp-omega3-server/rpc:i`.

> please run the `yarpserver` if you did not already before running the `yarp-omega3-server`

Available commands are
- `help`
- `stop` (completely disengange robot control)
- `quit` (close the module)
- `set_position(x, y, z)` (send a static position)
- `set_position_move_param(amax, vmax, jerk)` (limit movement parameters)
- `track_position(x, y, z)` (send a position, to be used in _streaming_ mode)
- `set_position_track_param(amax, vmax, jerk)` (limit movement parameters)
- `set_force(f_x, f_y, f_z)` (send a force reference)

The module switches from position to force control depending on the input from the user.

The state of the robot is available in forms of `yarp::sig::Vector`s sent over the port
- `/yarp-omega3-server/robot_state:o` (3 position, 3 velocity, 3 force)

[Sample modules](src/samples/python) written in Python are available.

#### Note:
When connected to a usb hub it might be neccesary to reboot system to connect to `yarp-omega3-server` for the first use.

### Maintainers

This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/xenvre.png" width="40">](https://github.com/xenvre) | [@xenvre](https://github.com/xenvre) |
