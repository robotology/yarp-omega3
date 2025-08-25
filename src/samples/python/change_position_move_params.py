'''
This example shows how to read and change the robot motion parameters. Retrieving tracking parameters works similarly.
'''
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

def get_position_move_parameters(rpc):
    '''amax, vmax, jerk'''
    
    cmd = yarp.Bottle()
    reply = yarp.Bottle()

    cmd.addString('get_pos_move_parameters')

    rpc.write(cmd, reply)
    param_str = reply.get(0).asString()
    
    return tuple(map(float, param_str.split()))


def set_position_move_parameters(rpc, amax, vmax, jerk):
    '''amax, vmax, jerk'''

    cmd = yarp.Bottle()
    reply = yarp.Bottle()

    cmd.addString('set_pos_move_parameters')
    cmd.addFloat32(amax)
    cmd.addFloat32(vmax)
    cmd.addFloat32(jerk)

    rpc.write(cmd, reply)
    print(f"Reply: {reply.toString()}")


def send_stop(rpc):

    cmd = yarp.Bottle()
    reply = yarp.Bottle()

    cmd.addString('stop')

    print("Sending stop command.")
    rpc.write(cmd, reply)
    print(f"Reply: {reply.toString()}")


def main():

    yarp.Network.init()

    rpc_client = yarp.RpcClient()
    rpc_client.open('/test-omega/rpc:o')
    connected = yarp.NetworkBase.connect('/test-omega/rpc:o', '/yarp-omega3-server/rpc:i')
    print(f"RPC connection status: {connected}")

    # Test four with different movement parameters
    try:
        print("Getting default position move parameters...")
        print(get_position_move_parameters(rpc_client))
        send_position_reference(rpc_client, 0.0, 0.09, 0.0)
        time.sleep(3.0)


        print("Setting position move parameters to: 0.1, 1.0, 1.0")
        print(set_position_move_parameters(rpc_client, 0.1, 1.0, 1.0))
        print("Getting position move parameters...")
        print(get_position_move_parameters(rpc_client))
        send_position_reference(rpc_client, 0.0, -0.09, 0.0)
        time.sleep(4.0)

        print("Setting position move parameters to: 2.0, 1.0, 1.0")
        print(set_position_move_parameters(rpc_client, 2.0, 1.0, 1.0))
        print("Getting position move parameters...")
        print(get_position_move_parameters(rpc_client))
        send_position_reference(rpc_client, 0.0, 0.09, 0.0)
        time.sleep(2.0)

        print("Setting position move parameters to: 10.0, 0.1, 1.0")
        print(set_position_move_parameters(rpc_client, 10.0, 0.1, 1.0))
        print("Getting position move parameters...")
        print(get_position_move_parameters(rpc_client))
        send_position_reference(rpc_client, 0.0, -0.09, 0.0)
        time.sleep(5.0)

        print("Setting position move parameters to: 1.0, 1.0, 1.0")
        print(set_position_move_parameters(rpc_client, 1.0, 1.0, 1.0))
        print("Getting position move parameters...")
        print(get_position_move_parameters(rpc_client))
        send_position_reference(rpc_client, 0.0, 0.0, 0.0)
        time.sleep(2.0)

        send_stop(rpc_client)
        time.sleep(2.0)

    except KeyboardInterrupt:
        send_stop(rpc_client)
        rpc_client.close()

if __name__ == '__main__':
    main()
