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

    return EXIT_SUCCESS;
}
