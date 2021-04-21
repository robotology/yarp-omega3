/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iostream>

#include <Server.h>

#include <drdc.h>

#include <yarp/os/Value.h>

using namespace yarp::os;
using namespace yarp::sig;

bool Server::close()
{
    /* Disengage the robot. */
    stop_motion();
    drdClose();

    return true;
}

bool Server::configure(ResourceFinder &rf)
{
    /* Collect configuration parameters. */
    period_ = rf.check("period", Value("0.001")).asDouble(); // max can be 3kHz (1/3k), original was 0.01

    /* Set up port for commands. */
    if (!port_rpc_.open("/yarp-omega3-server/rpc:i"))
    {
        std::cout << "Error: cannot open RPC port." << std::endl;

        return false;
    }

    if (!(this->yarp().attachAsServer(port_rpc_)))
    {
        std::cout << "Error: cannot attach RPC port." << std::endl;

        return false;
    }

    /* Set up ports for streaming. */
    if (!port_position_.open("/yarp-omega3-server/position:o"))
    {
        std::cout << "Error: cannot open output port for position." << std::endl;

        return false;
    }

    if (!port_velocity_.open("/yarp-omega3-server/velocity:o"))
    {
        std::cout << "Error: cannot open output port for velocity." << std::endl;

        return false;
    }

    if (!port_force_.open("/yarp-omega3-server/force:o"))
    {
        std::cout << "Error: cannot open output port for force." << std::endl;

        return false;
    }

    /* Initialize the device. */
    if (drdOpen() < 0)
    {
        std::cout << "Error: cannot initialize the Omega.3 robot." << std::endl;

        return false;
    }

    if (!drdIsSupported())
    {
        std::cout << "Error: a robot was found but is not supported." << std::endl;

        return false;
    }

    if (!drdIsInitialized() && drdAutoInit() < 0)
    {
        std::cout << "Error: cannot initialize the Omega.3 robot." << std::endl;

        return false;
    }
    else if (drdStart() < 0)
    {
        std::cout << "Error: cannot start the Omega.3 robot." << std::endl;

        return false;
    }

    /* Move in a safe position and idle the robot. */

    enable_position_control();
    drdMoveToPos(0.0, 0.0, 0.0, false); // stay at initial position

    state_ = State::Idle;

    std::cout << "Server running..." << std::endl;

    return true;
}

double Server::getPeriod()
{
    return period_;
}

bool Server::updateModule()
{
    stream_robot_state();

    State state = get_state();

    if (state == State::Close)
    {
        return false;
    }
    else if (state == State::PositionControl)
    {
        /* Nothing to do. */
        ;
    }
    else if (state == State::PositionTracking)
    {
        drdTrackPos(x_, y_, z_);
    }
    else if (state == State::SetPosition)
    {
        drdMoveToPos(x_, y_, z_, false);

        set_state(State::PositionControl);
    }
    /*
    else if (state == State::SetPosTrackParam)
    {
        drdSetPosTrackParam(amax_, vmax_, jerk_);
    }
    */
    else if (state == State::SetPosMoveParam)
    {
        drdSetPosMoveParam(amax_, vmax_, jerk_);
    }
    else if (state == State::ForceControl)
    {
        drdSetForceAndTorqueAndGripperForce(f_x_, f_y_, f_z_, 0.0, 0.0, 0.0, 0.0);
    }

    return true;
}

std::string Server::set_force(const double f_x, const double f_y, const double f_z)
{
    State state = get_state();

    if (state == State::Idle)
        drdStart();

    if (state != State::ForceControl)
        enable_force_control();

    set_state(State::ForceControl);

    f_x_ = f_x;
    f_y_ = f_y;
    f_z_ = f_z;

    return "OK";
}

std::string Server::set_position(const double x, const double y, const double z)
{
    State state = get_state();

    if (state == State::Idle)
        drdStart();

    if (state != State::PositionControl)
        enable_position_control();

    set_state(State::SetPosition);

    x_ = x;
    y_ = y;
    z_ = z;

    return "OK";
}

std::string Server::track_position(const double x, const double y, const double z)
{
    State state = get_state();

    if (state == State::Idle)
        drdStart();

    if (state != State::PositionTracking)
        enable_position_tracking();

    set_state(State::PositionTracking);

    x_ = x;
    y_ = y;
    z_ = z;

    return "OK";
}

std::string Server::set_position_track_param(const double amax, const double vmax, const double jerk)
{
    State state = get_state();

    set_state(State::SetPosTrackParam);

    amax_ = amax;
    vmax_ = vmax;
    jerk_ = jerk;

    return "OK";
}

std::string Server::set_position_move_param(const double amax, const double vmax, const double jerk)
{
    State state = get_state();

    if (state == State::Idle)
        drdStart();

    if (state != State::PositionControl)
        enable_position_control();

    set_state(State::SetPosMoveParam);

    amax_ = amax;
    vmax_ = vmax;
    jerk_ = jerk;

    return "OK";
}

std::string Server::stop()
{
    set_state(State::Idle);

    stop_motion();

    return "OK";
}

std::string Server::quit()
{
    set_state(State::Close);

    return "OK";
}

void Server::enable_position_control()
{
    drdRegulatePos(true);
    drdEnableFilter(false);
}

void Server::enable_position_tracking()
{
    drdRegulatePos(true);
    drdEnableFilter(true);
}

void Server::enable_force_control()
{
    drdRegulatePos(false);
    drdRegulateGrip(false);
    drdRegulateRot(false);
    drdEnableFilter(false);
}

Server::State Server::get_state()
{
    State state;

    mutex_.lock();
    state = state_;
    mutex_.unlock();

    return state;
}

void Server::stop_motion()
{
    drdStop();
}

void Server::set_state(const State &state)
{
    mutex_.lock();
    state_ = state;
    mutex_.unlock();
}

void Server::stream_robot_state()
{
    double position[3]; // position px, py, pz

    dhdGetPosition(&position[0], &position[1], &position[2]);

    Vector &position_out = port_position_.prepare();
    position_out = Vector(3, position);
    port_position_.write();

    double velocity[3]; // velocity vx, vy, vz

    dhdGetLinearVelocity(&velocity[0], &velocity[1], &velocity[2]);

    Vector &velocity_out = port_velocity_.prepare();
    velocity_out = Vector(3, velocity);
    port_velocity_.write();

    double force[3]; // force fx, fy, fz

    dhdGetForce(&force[0], &force[1], &force[2]);

    Vector &force_out = port_force_.prepare();
    force_out = Vector(3, force);
    port_force_.write();
}
