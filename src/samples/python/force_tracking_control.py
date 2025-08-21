import math
import time
import yarp


def send_force_reference(rpc, x, y, z):

    cmd = yarp.Bottle()
    reply = yarp.Bottle()

    cmd.addString('set_force')
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

    print('##################################################################')
    print('# Warning: the robot will try to exert a force along the y axis. #')
    print('# Please hold the end-effector firmly before continuing.         #')
    print('# Press any key to continue.                                     #')
    print('# Press Ctrl-C at any time to stop robot.                        #')
    print('##################################################################')
    input()

    # Test a simple sinusoidal reference

    a = 3.0
    f = 0.5
    dt = 0.01
    t = 0.0

    try:
        while True:

            send_force_reference(rpc_client, 0.0, a * math.sin(2 * math.pi * f * t), 0.0)

            t += dt

            time.sleep(dt)

    except KeyboardInterrupt:
        send_stop(rpc_client)
        rpc_client.close()

if __name__ == '__main__':
    main()
