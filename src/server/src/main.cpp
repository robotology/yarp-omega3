/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <cstdlib>

#include <iostream>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <Server.h>

#include <drdc.h>
#include <thread>
int main(int argc, char** argv)
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        std::cout << "Error: YARP is not available." << std::endl;

        return EXIT_FAILURE;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("yarp-omega-3");
    rf.configure(argc, argv);

    Server server;
    server.runModule(rf);
    // std::cout << "Initialization " << dhdGetSystemName() << std::endl;

    // if (drdOpen () < 0)
    //     return EXIT_FAILURE;

    // if (!drdIsSupported ())
    //     return EXIT_FAILURE;

    // std::cout << "Using device " << dhdGetSystemName() << std::endl;

    // if (!drdIsInitialized () && drdAutoInit () < 0)
    //     return EXIT_FAILURE;
    // else if (drdStart () < 0)
    //     return EXIT_FAILURE;

    // std::cout << "Test position control." << std::endl;

    // drdRegulatePos(true);
    // drdMoveToPos(0.0, 0.0, 0.0, false);

    // std::this_thread::sleep_for (std::chrono::seconds(5));

    // std::cout << "Test force control." << std::endl;
    // drdRegulatePos(false);
    // drdRegulateGrip(false);
    // drdRegulateRot(false);
    // drdEnableFilter(false);

    // // while (true)
    // // {
    // //     std::this_thread::sleep_for (std::chrono::milliseconds(10));
    // //     drdSetForceAndTorqueAndGripperForce(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0);
    // // }

    // std::this_thread::sleep_for (std::chrono::seconds(5));

    // std::cout << "Closing." << std::endl;
    // drdStop ();
    // drdClose ();

    return EXIT_SUCCESS;
}
