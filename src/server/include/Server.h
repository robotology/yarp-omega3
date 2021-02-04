/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef SERVER_H
#define SERVER_H

#include <thrift/ServiceIDL.h>

#include <mutex>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <yarp/sig/Vector.h>

class Server : public yarp::os::RFModule,
               public ServiceIDL
{
public:
    /**
     * RFModule-side API
     */

    bool close() override;

    bool configure(yarp::os::ResourceFinder&) override;

    double getPeriod() override;

    bool updateModule() override;

    /**
     * Thrift-side API
     */

    std::string set_force(const double f_x, const double f_y, const double f_z) override;

    std::string set_position(const double x, const double y, const double z) override;

    std::string set_position_move_param(const double amax, const double vmax, const double jerk) override;

    std::string track_position(const double x, const double y, const double z) override;

    std::string set_position_track_param(const double amax, const double vmax, const double jerk) override;

    std::string stop() override;

    std::string quit() override;

private:
    enum class State
    {
        Close,
        Idle,
        PositionControl,
        PositionTracking,
        ForceControl,
        SetPosition,
        SetPosMoveParam,
        SetPosTrackParam
    };

    /**
     * Private API
     */
    void enable_position_control();

    void enable_position_tracking();

    void enable_force_control();

    Server::State get_state();

    void set_state(const State& state);

    void stop_motion();

    void stream_robot_state();

    /**
     * Parameters.
     */
    double period_;

    /**
     * Current state.
     */

    State state_ = State::Idle;

    double x_ = 0.0;

    double y_ = 0.0;

    double z_ = 0.0;

    double f_x_ = 0.0;

    double f_y_ = 0.0;

    double f_z_ = 0.0;

    double amax_ = 0.0;

    double vmax_ = 0.0;

    double jerk_ = 0.0;

    /**
     * RPC server.
     */
    std::mutex mutex_;

    yarp::os::Port port_rpc_;

    /**
     * Output ports.
     */
    yarp::os::BufferedPort<yarp::sig::Vector> port_robot_state_;
};

#endif /* SERVER_H */
