yarp-omega3
======================

Simple YARP-enabled server for the [Force Dimension Omega.3](https://www.forcedimension.com/products/omega) device.

Supports Linux only at the moment.

### How to build

```
git clone https://github.com/robotology-playground/yarp-omega3
cd yarp-omega3
mkdir build
cd build
cmake . -DCMAKE_PREFIX_PATH=<where_to_install>
make install
```

In order to use the device in user space, you need to import the provided [udev rule](config/99-omega3-libusb.rules), e.g. in `/etc/udev/rules.d`.

### How to use

Run `yarp-omega3-server` and send RPC commands to `/yarp-omega-3-server/rpc:i`.

Available commands are 
- help
- stop (completely disengange robot control)
- quit (close the module)
- set_position <x> <y> <z> (send a static position)
- track_position <x> <y> <z> (send a position, to be used in _streaming_ mode)
- set_force <f_x> <f_y> <f_z> (send a force reference)
  
The module switches from position to force control depending on the input from the user.

[Sample modules](src/samples/python) written in Python are available.
