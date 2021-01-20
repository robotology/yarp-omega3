yarp-omega3
======================

Simple YARP-enabled server for Force Dimension Omega.3 device.

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

Run `yarp-omega3-server` and send RPC commands to `/yarp-omega-3-server/rpc:i`
