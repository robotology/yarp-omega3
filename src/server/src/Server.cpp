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


bool Server::close()
{
    /* Disengage the robot. */
    stop_motion();
    drdClose();

    return true;
}


bool Server::configure(ResourceFinder& rf)
{
    /* Collect configuration parameters. */
    period_ = rf.check("period", Value("0.01")).asDouble();

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

    /* Initialize the device. */
    if (drdOpen() < 0)
    {
        std::cout << "Error: cannot initialize the robot." << std::endl;

        return false;
    }

    if (!drdIsSupported())
    {
        std::cout << "Error: a robot was found but is not supported." << std::endl;

        return false;
    }

    if (!drdIsInitialized () && drdAutoInit () < 0)
    {
        std::cout << "Error: cannot initialize the robot." << std::endl;

        return false;
    }
    else if (drdStart () < 0)
    {
        std::cout << "Error: cannot initialize the robot." << std::endl;

        return false;
    }

    /* Move in a safe position and idle the robot. */
    enable_position_control();
    drdMoveToPos(0.0, 0.0, 0.0, false);
    state_ = State::PositionControl;

    std::cout << "Server running..." << std::endl;

    return true;
}


double Server::getPeriod()
{
    return period_;
}


bool Server::updateModule()
{
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

        // state_ = State::PositionControl;
        set_state(State::PositionControl);
    }
    else if (state == State::ForceControl)
    {
        drdSetForceAndTorqueAndGripperForce(f_x_, f_y_, f_z_, 0.0, 0.0, 0.0, 0.0);
    }

    return true;
}


std::string Server::set_force(const double f_x, const double f_y, const double f_z)
{
    // std::lock_guard<std::mutex> lock(mutex_);

    State state = get_state();

    if (state == State::Idle)
        drdStart();

    if (state != State::ForceControl)
        enable_force_control();

    // state_ = State::ForceControl;
    set_state(State::ForceControl);

    f_x_ = f_x;
    f_y_ = f_y;
    f_z_ = f_z;

    return "OK";
}


std::string Server::set_position(const double x, const double y, const double z)
{
    // std::lock_guard<std::mutex> lock(mutex_);

    State state = get_state();

    if (state == State::Idle)
        drdStart();

    if (state != State::PositionControl)
        enable_position_control();

    // state_ = State::SetPosition;
    set_state(State::SetPosition);

    x_ = x;
    y_ = y;
    z_ = z;

    return "OK";
}


std::string Server::track_position(const double x, const double y, const double z)
{
    // std::lock_guard<std::mutex> lock(mutex_);

    State state = get_state();

    if (state == State::Idle)
        drdStart();

    if (state != State::PositionTracking)
        enable_position_tracking();

    // state_ = State::PositionTracking;
    set_state(State::PositionTracking);

    x_ = x;
    y_ = y;
    z_ = z;

    return "OK";
}


std::string Server::stop()
{
    // std::lock_guard<std::mutex> lock(mutex_);

    // state_ = State::Idle;
    set_state(State::Idle);

    stop_motion();

    return "OK";
}


std::string Server::quit()
{
    // std::lock_guard<std::mutex> lock(mutex_);

    // state_ = State::Close;
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


void Server::stop_motion()
{
    drdStop();
}


void Server::set_state(const State& state)
{
    mutex_.lock();
    state_ = state;
    mutex_.unlock();
}


Server::State Server::get_state()
{
    State state;

    mutex_.lock();
    state = state_;
    mutex_.unlock();

    return state;
}
