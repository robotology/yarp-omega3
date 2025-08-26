/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

service ServiceIDL
{
    string set_force(1:double f_x, 2:double f_y, 3:double f_z);
    string move_to_pos(1:double x, 2:double y, 3:double z);
    string track_pos(1:double x, 2:double y, 3:double z);
    string get_pos_move_param();
    string set_pos_move_param(1:double amax, 2:double vmax, 3:double jerk);
    string get_pos_tracking_param();
    string set_pos_track_param(1:double amax, 2:double vmax, 3:double jerk);
    string stop();
    string quit();
}
