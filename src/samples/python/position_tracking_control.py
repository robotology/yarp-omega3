import math
import time
import yarp


def send_position_reference(rpc, x, y, z):

    cmd = yarp.Bottle()
    reply = yarp.Bottle()

    cmd.addString('track_position')
    cmd.addFloat64(x)
    cmd.addFloat64(y)
    cmd.addFloat64(z)


    rpc.write(cmd, reply)


def send_stop(rpc):

    cmd = yarp.Bottle()
    reply = yarp.Bottle()

    cmd.addString('stop')

    rpc.write(cmd, reply)


def main():

    yarp.Network.init()

    rpc_client = yarp.RpcClient()
    rpc_client.open('/test-omega/rpc:o')
    yarp.NetworkBase.connect( '/test-omega/rpc:o', '/yarp-omega3-server/rpc:i')

    # Test a simple circular motion

    r = 0.04
    f = 0.5
    dt = 0.01
    t = 0.0

    try:
        while True:

            send_position_reference\
            (
                rpc_client,
                r * math.sin(2 * math.pi * f * t),
                r * math.cos(2 * math.pi * f * t),
                0.0
            )

            t += dt

            time.sleep(dt)

    except KeyboardInterrupt:
        send_stop(rpc_client)
        rpc_client.close()

if __name__ == '__main__':
    main()
