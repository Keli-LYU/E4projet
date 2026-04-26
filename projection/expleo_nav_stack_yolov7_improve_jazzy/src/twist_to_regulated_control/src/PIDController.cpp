// =====================================================================================
//
//       Filename:  PIDController.cpp
//
//    Description:  PID controller class
//
//        Version:  1.0
//        Created:  11/10/2022 15:58:11
//       Revision:  none
//       Compiler:  g++
//
//         Author:  Thibaud Dubautbout (TD), thibaud.duhautbout@expleogroup.com
//   Organization:  Expleo Group
//
// =====================================================================================
#include "carla_twist_to_regulated_control/PIDController.hpp"

#include <iostream>

PIDController::PIDController(double Kp,
                             double Ki,
                             double Kd,
                             double int_sat_min,
                             double int_sat_max,
                             double u_sat_min,
                             double u_sat_max)
    : _Kp(Kp), _Ki(Ki), _Kd(Kd), _e(0), _Ie(0), _De(0), _vRef(0), _t(0)
{
    _Isat[0] = int_sat_min;
    _Isat[1] = int_sat_max;
    _usat[0] = u_sat_min;
    _usat[1] = u_sat_max;
}

void PIDController::addMeasure(double v, double t)
{
    // compute the new error based on the current reference
    double newError = _vRef - v;

    // compute the time since the last measurement
    double dt = t - _t;

    // integrate the error with a trapeze method
    _Ie = std::clamp(_Ie + (_e + newError) * dt / 2, _Isat[0], _Isat[1]);

    // derivate the error
    _De = (newError - _e) / dt;

    // update the error and the time
    _e = newError;
    _t = t;
}

double PIDController::computeControl(double v_ref)
{
    // update the reference value for later
    _vRef = v_ref;

    // compute the control value
    return std::clamp(_Kp * _e + _Ki * _Ie + _Kd * _De, _usat[0], _usat[1]);
}
