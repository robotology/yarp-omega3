/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

service ServiceIDL
{
    string setForce(1:double f_x, 2:double f_y, 3:double f_z);
    string moveToPos(1:double x, 2:double y, 3:double z);
    string trackPos(1:double x, 2:double y, 3:double z);
    string getPosMoveParam();
    string setPosMoveParam(1:double amax, 2:double vmax, 3:double jerk);
    string getPosTrackParam();
    string setPosTrackParam(1:double amax, 2:double vmax, 3:double jerk);
    string stop();
    string quit();
}
