import time
import yarp


def send_position_reference(rpc, x, y, z):

    cmd = yarp.Bottle()
    reply = yarp.Bottle()

    cmd.addString('set_position')
    cmd.addFloat32(x)
    cmd.addFloat32(y)
    cmd.addFloat32(z)

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

    # Test four poses
    try:
        send_position_reference(rpc_client, 0.04, 0.04, 0.0)
        time.sleep(2.0)

        send_position_reference(rpc_client, 0.04, -0.04, 0.0)
        time.sleep(2.0)

        send_position_reference(rpc_client, 0.0, -0.04, 0.0)
        time.sleep(2.0)

        send_position_reference(rpc_client, 0.0, 0.04, 0.0)
        time.sleep(2.0)

        send_position_reference(rpc_client, 0.0, 0.0, 0.0)
        time.sleep(2.0)

        send_stop(rpc_client)
        time.sleep(2.0)

    except KeyboardInterrupt:
        send_stop(rpc_client)
        rpc_client.close()

if __name__ == '__main__':
    main()
