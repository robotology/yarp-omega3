import time
import yarp

def send_position_reference(rpc, x, y, z):

    cmd = yarp.Bottle()
    reply = yarp.Bottle()

    cmd.addString('set_position')
    cmd.addDouble(x)
    cmd.addDouble(y)
    cmd.addDouble(z)

    rpc.write(cmd, reply)

def send_force_reference(rpc, fx, fy, fz):

    cmd = yarp.Bottle()
    reply = yarp.Bottle()

    cmd.addString('set_force')
    cmd.addDouble(fx)
    cmd.addDouble(fy)
    cmd.addDouble(fz)

    rpc.write(cmd, reply)

def send_stop(rpc):

    cmd = yarp.Bottle()
    reply = yarp.Bottle()

    cmd.addString('stop')

    rpc.write(cmd, reply)


# tol_position = 0.01
# tol_force = 0.01
# max_force = 5
# step_size_position = 0.01
# step_size_force = 0.1
# counter = 0
# f_z_init = 0.01  # initial force
# time_to_press = 5  # sec?
# counter = 0


def main():

    # move that config
    yarp.Network.init()

    # setting up connection for sending commands
    rpc_client = yarp.RpcClient()
    rpc_client.open('/server-omega/rpc:o')
    yarp.NetworkBase.connect('/server-omega/rpc:o', '/yarp-omega3-server/rpc:i', "fast_tcp")
    if not yarp.NetworkBase.connect('/server-omega/rpc:o', '/yarp-omega3-server/rpc:i', "fast_tcp"):
        print("Failed to connect to server")

    # setting up connection for recieving position stream
    position_stream = yarp.BufferedPortVector() # BufferedPortProperty instead of port?
    position_stream.open("/position_input:o")
    yarp.Network.connect("/yarp-omega3-server/position:o", "/position_input:o", "fast_tcp")
    if not yarp.Network.connect("/yarp-omega3-server/position:o", "/position_input:o", "fast_tcp"):
        print("Failed to connect to yarp port for position input")

    # setting up connection for recieving force stream
    force_stream = yarp.BufferedPortVector()
    force_stream.open("/force_input:o")
    yarp.NetworkBase.connect("/yarp-omega3-server/force:o", "/force_input:o", "fast_tcp")
    if not yarp.NetworkBase.connect("/yarp-omega3-server/force:o", "/force_input:o", "fast_tcp"):
        print("Failed to connect to yarp port for force input")

    # update loop
    try:
        send_stop(rpc_client) # let the robot be moved freely

        print("**************************************")
        print("*Move the robot to start position    *")
        print("*and press ENTER to start experiment.*") 
        print("*The procedure can be interrupted at *")
        print("*anytime by pressing CTRL + C.       *")
        print("**************************************")
        
        # case one: init position
        #if (input()=='s'):
        # move omega by hand to initial to position and store coordinates when button pressed
        # wait for keybord input to save ref positions
        # input()

        # read in from yarp port
        positions = position_stream.read()
        print(positions[0], positions[1], positions[2])
        forces = force_stream.read()
        print(forces[0], forces[1], forces[2])



    except (KeyboardInterrupt):
        send_stop(rpc_client)
        rpc_client.close()


if __name__ == '__main__':
    main()
