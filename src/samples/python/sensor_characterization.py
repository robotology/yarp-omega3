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

    yarp.Network.init()

    # connect to port for sending commands
    rpc_client_commands = yarp.RpcClient()
    rpc_client_commands.open('/commands-omega/rpc:o')
    yarp.NetworkBase.connect('/commands-omega/rpc:o', '/yarp-omega3-server/rpc:i', "fast_tcp")

    # connect to port for recieving position
    rpc_client_position = yarp.RpcClient()
    rpc_client_position.open("/position_input:o")
    yarp.NetworkBase.connect("/yarp-omega3-position/rpc:o", "/position_input:o", "fast_tcp")

    # connect to port for recieving force
    rpc_client_position = yarp.RpcClient()
    rpc_client_position.open("/position_input:o")
    yarp.NetworkBase.connect("/yarp-omega3-force/rpc:o", "/position_input:o", "fast_tcp")
    
    try:
        send_stop(rpc_client_commands)
        print("**********************************")
        print("*Move the robot to start position*")
        print("*and press ENTER to confirm      *" ) 
        print("**********************************")
        
        # case one: init position
        #if (input()=='s'):
        # move omega by hand to initial to position and store coordinates when button pressed
        # wait for keybord input to save ref positions
        input()
        print("ENTER pressed!")
        # read in from yarp port
        print("got the position")

        # x_ref = get_position_reference(rpc_client_commands, x, y, z)[2]
        # y_ref = get_position_reference(rpc_client_commands, x, y, z)[3]
        # z_ref = get_position_reference(rpc_client_commands, x, y, z)[4]

        # # move up until no contact + tol. -> safe coordinates (x_ini, y_ini, z_ini)
        # while (get_force_reference(rpc_client_commands, f_x, f_y, f_z)[4] > 0):
        #     send_position_reference(
        #         rpc_client_commands, x_ref, y_ref, z_ref+counter/1000)
        # # increase distnace by tol to be sure no contact is made even after hysteresis
        # # and store values
        # z_ref = z_ref+tol+counter/1000
        # send_position_reference(rpc_client_commands, x_ref, y_ref, z_ref)

        # # case two: apply forces
        # #if (input()=='e'):
        # # loop over different forces applied to fingertip by omega
        # for force in range(f_z_init, max_force, step_size):
        #     # move slowly down to make contact and go on moving until goal force reached
        #     send_position_reference(
        #         rpc_client_commands, x_ref, y_ref, z_ref-step_size_position)
        #     get_force_reference(rpc_client_commands, f_x, f_y, f_z)
        #     counter = counter+1
        #     # when goal force reached hold for certain time
        #     if (get_force_reference(rpc_client_commands, f_x, f_y, f_z)[4] >= force):
        #         # read time here: time_pre_start
        #         start = timeit.default_timer()
        #         while(timeit.default_timer() <= start + time_to_press):
        #             # read time here again
        #             send_position_reference(
        #                 rpc_client_commands, x_ref, y_ref, z_ref-step_size_position)
        #     # keep force const. for n time
        #     # dump data (yarp data dumper)
        #     # move to safed coordinates (x_ini, y_ini, z_ini)

        # #if (input()=='q'):
        # send_stop(rpc_client_commands)
        # rpc_client_commands.close()

    except (KeyboardInterrupt):
        send_stop(rpc_client_commands)
        rpc_client_commands.close()


if __name__ == '__main__':
    main()
